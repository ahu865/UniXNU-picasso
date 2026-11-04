#!/usr/bin/env python3
"""Build the FocalTech FT3518 touch driver for vangogh.

Compiles the GENI I2C engine and the staged FT probe with this tree's recorded
XNU kext flags. The FT3518 keeps its firmware in internal flash, so unlike the
picasso Novatek driver there is no firmware image and no download stage.

Output: build/novatek-touch/libNativeNovatekTouch.a -- the same path the
board's kernel patch links when XNU_NOVATEK_TOUCH is set. The two touch
drivers are drop-in replacements for each other; only one is ever linked.
"""
import argparse
import hashlib
import importlib.util
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if not (ROOT / 'build/xnu').is_dir() and (ROOT / 'out/picasso/tree/build/xnu').is_dir():
    ROOT = ROOT / 'out/picasso/tree'

DRIVER = ROOT / 'drivers/NativeFocaltechTouch'
# The PMIC button driver rides in this archive because this is the one platform archive the vangogh kernel
# force-loads for input; a second archive would need a new -force_load line in build-xnu-4k-baseline.sh.
BUTTONS = ROOT / 'drivers/NativeBoardButtons'
RECORD = 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'
OUT = ROOT / 'build/novatek-touch'
ARCHIVE_NAME = 'libNativeNovatekTouch.a'


def run(command, cwd=None):
    print('+', ' '.join(map(str, command)))
    subprocess.run([str(value) for value in command], check=True, timeout=900, cwd=cwd)


def flags(record_path, source, obj, language):
    """Recorded IOPlatformExpert compile line, retargeted to our source."""
    info = json.loads(Path(record_path).read_text().strip().rstrip(','))
    command, index, arguments = [], 0, info['arguments']
    while index < len(arguments):
        value = arguments[index]
        if value == info['file']:
            command.append(str(source))
        elif value == '-o':
            command.extend(['-o', str(obj)])
            index += 1
        elif value in ('-MF', '-MT', '-MQ', '-MJ', '--serialize-diagnostics'):
            index += 1
        elif value in ('-MD', '-MMD', '-MP', '--driver-mode=g++', '-fno-odr-hash-protocols',
                       '-fno-c++-static-destructors'):
            pass
        elif value == '-xc++':
            command.append('-xc++' if language == 'c++' else '-xc')
        elif value == '-std=gnu++2b':
            command.append('-std=gnu++2b' if language == 'c++' else '-std=gnu11')
        elif value == '-Xclang':
            index += 1
        elif value.startswith('-W'):
            pass
        else:
            command.append(value)
        index += 1
    command.extend(['-Wall', '-Wextra', '-Wno-unused-parameter', '-Wno-unknown-warning-option',
                    '-Wno-missing-designated-field-initializers',
                    '-I', str(ROOT / '.cache/xnu-work/osfmk'), '-I', str(source.parent),
                    '-I', str(DRIVER)])
    return command, info['directory']


def inspect_objects(objects):
    def load(path):
        spec = importlib.util.spec_from_file_location(path.stem.replace('-', '_'), path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    platform = load(ROOT / 'scripts/build-picasso-pmgr.py')
    native = load(ROOT / 'kits/NativePrelink/native_prelink.py')
    reports = {}
    for obj in objects:
        _, sections, symbols = platform.inspect_object(obj, native)
        platform.require(not any(s['name'] == '__mod_init_func' for s in sections),
                         'Touch archive must have no automatic initializer')
        if obj.name == 'FtProbe.o':
            for name in ('seq', 'count', 'writer_lock', 'xs', 'ys', 'ids'):
                matches = [s for s in symbols if s['name'] == '_native_touch_' + name]
                platform.require(len(matches) == 1 and matches[0]['kind'] == 1,
                                 'HID must own the shared touch storage: ' + name)
        reports[obj.name] = hashlib.sha256(obj.read_bytes()).hexdigest()
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', default=str(ROOT / RECORD))
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    objects = []
    for name, language in (('GeniI2c.c', 'c'), ('FtProbe.cpp', 'c++')):
        obj = OUT / (name.rsplit('.', 1)[0] + '.o')
        compile_command, cwd = flags(args.record, DRIVER / name, obj, language)
        run(compile_command, cwd=cwd)
        objects.append(obj)
    # The board buttons (side key and both volume keys) hang off the PMICs behind the SPMI arbiter; the touch
    # probe is the entry point the boot monitor reaches, so it starts the button thread too.
    buttons = OUT / 'ButtonsProbe.o'
    compile_command, cwd = flags(args.record, BUTTONS / 'ButtonsProbe.cpp', buttons, 'c++')
    run(compile_command, cwd=cwd)
    objects.append(buttons)

    object_hashes = inspect_objects(objects)
    archive = OUT / ARCHIVE_NAME
    archive.unlink(missing_ok=True)
    run(['xcrun', 'ar', 'rcs', archive, *objects])
    print('focaltech-touch:', archive)
    report = dict(complete=True, firmware=None,
                  objects=object_hashes,
                  archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                  sources={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                           for path in sorted(DRIVER.glob('*')) if path.is_file()},
                  hardware_tested=False)
    (OUT / 'build-report.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
