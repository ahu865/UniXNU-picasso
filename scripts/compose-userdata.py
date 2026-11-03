#!/usr/bin/env python3
"""Compose userdata-root.img from an IPSW without hdiutil (Linux/CI fallback).

The pinned UniXNU revision only knows the macOS path (`hdiutil attach` + `/dev/rdiskN`);
the newer upstream has a host-independent UDIF reader. This script carries that reader
plus the manifest writer so the image can be produced on a Linux runner.

Output layout, as `drivers/NativeUfs/RootLayout.h` expects it:
    0x000000           4096-byte root manifest, magic "SHNGROOT"
    0x100000 (1 MiB)   window 0 (OS), 1 MiB aligned
    ...                window 1 (Cryptex1,SystemOS), 1 MiB aligned

Manifest: magic[8] version u32 count u32 imageBytes u64, then 128-byte rows
(name[32], offset u64, size u64, blockBytes u32, flags u32, sha256[32], pad[40]),
and sha256 of the first 4064 bytes at 4064.
"""
from __future__ import annotations

import argparse
import bz2
import hashlib
import json
import os
import plistlib
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import tomllib
import zipfile
import zlib
from pathlib import Path

SECTOR = 512
TRAILER = 512
COPY = 8 << 20
MIB = 1 << 20

ZERO_FILL, RAW, IGNORE = 0x00000000, 0x00000001, 0x00000002
COMMENT, TERMINATOR = 0x7FFFFFFE, 0xFFFFFFFF
ZLIB, BZIP2, LZFSE = 0x80000005, 0x80000006, 0x80000007


def log(text):
    print(text, flush=True)


# --------------------------------------------------------------------------- UDIF


class DiskImage:
    """Raw disk of a .dmg: `size` in bytes, `blocks()` yields it in order."""

    def __init__(self, path: Path):
        self.path = path
        self.chunks = None
        length = path.stat().st_size
        koly = b''
        if length >= TRAILER:
            with path.open('rb') as image:
                image.seek(length - TRAILER)
                koly = image.read(TRAILER)
        if koly[:4] != b'koly':
            self.size = length
            return
        data_fork, = struct.unpack_from('>Q', koly, 0x18)
        plist_offset, plist_length = struct.unpack_from('>QQ', koly, 0xD8)
        sectors, = struct.unpack_from('>Q', koly, 0x1EC)
        with path.open('rb') as image:
            image.seek(plist_offset)
            plist = plistlib.loads(image.read(plist_length))
        self.size = sectors * SECTOR
        chunks = []
        for table in plist.get('resource-fork', {}).get('blkx', []):
            mish = table['Data']
            if mish[:4] != b'mish':
                raise SystemExit(f'{path}: blkx {table.get("Name")!r} is not a mish table')
            first, = struct.unpack_from('>Q', mish, 8)
            base, = struct.unpack_from('>Q', mish, 0x18)
            count, = struct.unpack_from('>I', mish, 0xC8)
            for index in range(count):
                kind, _, sector, length, offset, stored = struct.unpack_from('>IIQQQQ', mish, 0xCC + 40 * index)
                if kind not in (COMMENT, TERMINATOR):
                    chunks.append(((first + sector) * SECTOR, length * SECTOR, kind,
                                   data_fork + base + offset, stored))
        chunks.sort()
        position = 0
        for start, length, kind, _, _ in chunks:
            if start != position:
                raise SystemExit(f'{path}: no chunk covers bytes {position}..{start}')
            if kind not in (ZERO_FILL, RAW, IGNORE, ZLIB, BZIP2, LZFSE):
                raise SystemExit(f'{path}: unsupported chunk type {kind:#x} at byte {start}')
            position += length
        if position != self.size:
            raise SystemExit(f'{path}: chunks cover {position} bytes, the trailer says {self.size}')
        self.chunks = chunks

    def blocks(self):
        decoder = None
        with self.path.open('rb') as image:
            if self.chunks is None:
                while block := image.read(COPY):
                    yield block
                return
            for start, length, kind, offset, stored in self.chunks:
                if kind in (ZERO_FILL, IGNORE):
                    data = bytes(length)
                else:
                    image.seek(offset)
                    data = image.read(stored)
                    if kind == ZLIB:
                        data = zlib.decompress(data)
                    elif kind == BZIP2:
                        data = bz2.decompress(data)
                    elif kind == LZFSE:
                        decoder = decoder or shutil.which('lzfse')
                        if not decoder:
                            raise SystemExit('lzfse is not on PATH (needed to decode LZFSE chunks)')
                        data = subprocess.run([decoder, '-decode'], input=data,
                                              capture_output=True, check=True).stdout
                if len(data) != length:
                    raise SystemExit(f'{self.path}: chunk at {start} gives {len(data)} bytes, not {length}')
                yield data


# ----------------------------------------------------------------------- IPSW


def manifest_paths(ipsw: Path) -> dict[str, str]:
    with zipfile.ZipFile(ipsw) as archive:
        bm = plistlib.loads(archive.read('BuildManifest.plist'))
    identity = bm['BuildIdentities'][0]
    out = {}
    for key, entry in identity['Manifest'].items():
        path = entry.get('Info', {}).get('Path')
        if path:
            out[key] = path
    return out


def extract(ipsw: Path, member: str, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    with zipfile.ZipFile(ipsw) as archive, archive.open(member) as src, destination.open('wb') as dst:
        total = 0
        while block := src.read(COPY):
            dst.write(block)
            total += len(block)
    log(f'  extracted {member} -> {destination.name} ({total / 2**30:.2f} GiB, {time.time() - started:.0f}s)')
    return destination


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--ipsw', required=True, type=Path)
    parser.add_argument('--board', type=Path, default=Path('board.toml'))
    parser.add_argument('--output', type=Path, default=Path('out/picasso/userdata'))
    args = parser.parse_args()

    board = tomllib.loads(args.board.read_text())
    spec = board['userdata']
    capacity = (spec['last_lba'] - spec['first_lba'] + 1) * spec.get('lba_bytes', 4096)
    windows = spec['windows']
    paths = manifest_paths(args.ipsw)

    args.output.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix='userdata-', dir=args.output))
    sources = []
    layout = []
    cursor = MIB
    try:
        for window in windows:
            component = window['component']
            member = paths.get(component)
            if not member:
                if window.get('optional'):
                    log(f"skip {window['name']}: {component} is not in this IPSW")
                    continue
                raise SystemExit(f'{component} is not in {args.ipsw}')
            dmg = extract(args.ipsw, member, work / Path(member).name)
            image = DiskImage(dmg)
            log(f'  {window["name"]}: {member} raw {image.size / 2**30:.2f} GiB')
            layout.append((window['name'], cursor, image.size))
            sources.append((window['name'], dmg, image))
            cursor += (image.size + MIB - 1) // MIB * MIB

        total = cursor
        log(f'image: {total} bytes ({total / 2**30:.2f} GiB) of a {capacity} byte partition')
        if total > capacity:
            raise SystemExit('the image is larger than the userdata partition')

        out = args.output / 'userdata-root.img'
        entries = []
        with out.open('wb') as target:
            target.truncate(total)
            for (name, offset, size), (_, _, image) in zip(layout, sources):
                target.seek(offset)
                digest = hashlib.sha256()
                done = 0
                started = time.time()
                for data in image.blocks():
                    digest.update(data)
                    target.write(data)
                    done += len(data)
                    if done % (512 << 20) < COPY:
                        log(f'  {name}: {done / 2**30:.1f}/{size / 2**30:.1f} GiB')
                if done != size:
                    raise SystemExit(f'{name}: {done} bytes of raw disk, expected {size}')
                entries.append((name, offset, size, digest.digest()))
                log(f'  {name} at {offset:#x} sha256 {digest.hexdigest()} ({time.time() - started:.0f}s)')

            header = bytearray(4096)
            header[0:8] = b'SHNGROOT'
            struct.pack_into('<IIQ', header, 8, 1, len(entries), total)
            for index, (name, offset, size, digest) in enumerate(entries):
                struct.pack_into('<32sQQII32s40s', header, 24 + 128 * index,
                                 name.encode(), offset, size, 512, 1, digest, bytes(40))
            header[4064:4096] = hashlib.sha256(bytes(header[:4064])).digest()
            target.seek(0)
            target.write(header)

        image_sha = hashlib.sha256()
        with out.open('rb') as handle:
            while block := handle.read(COPY):
                image_sha.update(block)
        report = {
            'image_bytes': total,
            'image_sha256': image_sha.hexdigest(),
            'windows': [{'name': n, 'offset': o, 'size': s, 'sha256': d.hex()} for n, o, s, d in entries],
            'userdata_bytes': capacity,
            'flash_command': f'fastboot flash userdata {out}',
            'source': str(args.ipsw),
            'hardware_tested': False,
        }
        (args.output / 'userdata-root.json').write_text(json.dumps(report, indent=2) + '\n')
        log(json.dumps(report, indent=2))
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
