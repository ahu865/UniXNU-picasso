#!/usr/bin/env python3
"""Verify one staged Adreno 620 firmware file against its expected size and CRC.

compose-bootimg.yml downloads the vendor firmware set from public dumps and
runs this per file, so a moved or rewritten blob fails the build loudly
instead of shipping into the kernel image silently.

    verify-adreno-fw.py <name> <expected-bytes> <expected-crc-hex>

The file is read from ADRENO620_FW_DIR (default /tmp/picasso-gpu-fw), the
same directory scripts/build-adreno-picasso-firmware.py consumes.
"""
import os
import sys
import zlib
from pathlib import Path


def main():
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    name, want_size, want_crc = sys.argv[1], int(sys.argv[2]), int(sys.argv[3], 16)
    directory = Path(os.environ.get('ADRENO620_FW_DIR') or '/tmp/picasso-gpu-fw')
    path = directory / name
    data = path.read_bytes()
    got_size = len(data)
    got_crc = zlib.crc32(data) & 0xffffffff
    if (got_size, got_crc) != (want_size, want_crc):
        raise SystemExit(f'{name}: staged {got_size} bytes crc={got_crc:08x}, '
                         f'expected {want_size} bytes crc={want_crc:08x}')
    print(f'adreno620 fw {name}: {got_size} bytes crc={got_crc:08x} ok')


if __name__ == '__main__':
    main()
