"""Runs every offline check: C tests, Python tests, and the copies that must stay identical.

  python tests/run_tests.py            needs gcc and Python 3.8+ (no pyserial, no hardware)
  python tests/run_tests.py --sync     first copy common/ C files into handheld_rp2040/src/
  python tests/run_tests.py --rp2040   also compile the Feather RP2040 sketch with arduino-cli
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

TESTS = Path(__file__).resolve().parent
ROOT = TESTS.parent
COMMON = ROOT / 'common'
SKETCH = ROOT / 'handheld_rp2040'
BUILD = TESTS / 'build'
UPSTREAM = ROOT.parent / 'src'  # the rc_lora repo's src/, where rc_proto and joy_api were developed and tested
STRICT = ['-std=c99', '-Wall', '-Wextra', '-Wpedantic', '-Wconversion', '-Wshadow', '-Werror']
C_TESTS = {'test_joy_api': ['rc_proto.c', 'joy_api.c'],
           'test_handheld': ['rc_proto.c', 'joy_api.c', 'rc_handheld.c']}
SKETCH_COPIES = ['rc_proto.h', 'rc_proto.c', 'rc_handheld.h', 'rc_handheld.c']
FQBN = 'rp2040:rp2040:adafruit_feather_usb_host:usbstack=tinyusb,freq=120'


def c_tests():
    BUILD.mkdir(exist_ok=True)
    ok = True
    for name, sources in C_TESTS.items():
        exe = BUILD / (name + ('.exe' if os.name == 'nt' else ''))
        command = ['gcc', *STRICT, f'-I{COMMON}', *(str(COMMON / s) for s in sources), str(TESTS / f'{name}.c'),
                   '-o', str(exe), '-lm']
        built = subprocess.run(command, capture_output=True, text=True)
        if built.returncode:
            print(f'✗ {name}：編譯失敗\n{built.stderr}')
            ok = False
            continue
        ran = subprocess.run([str(exe)], capture_output=True, text=True)
        lines = ran.stdout.strip().splitlines()
        print(('✓' if ran.returncode == 0 else '✗') + f' {name}：{lines[-1] if lines else "沒有輸出"}')
        for line in lines[:-1]:
            print('   ' + line)
        ok = ok and ran.returncode == 0
    return ok


def python_tests():
    ran = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-p', 'test_*.py'], cwd=TESTS, capture_output=True,
                         text=True, encoding='utf-8', env=dict(os.environ, PYTHONIOENCODING='utf-8'))
    lines = [line for line in ran.stderr.strip().splitlines() if line]
    summary = ' '.join(lines[-2:]) if len(lines) >= 2 else ran.stderr
    print(('✓' if ran.returncode == 0 else '✗') + f' test_system.py：{summary}')
    if ran.returncode:
        print(ran.stderr)
    return ran.returncode == 0


def copies(sync):
    ok = True
    for name in SKETCH_COPIES:
        source, copy = COMMON / name, SKETCH / 'src' / name
        if sync:
            shutil.copyfile(source, copy)
        if not copy.exists() or copy.read_bytes() != source.read_bytes():
            print(f'✗ handheld_rp2040/src/{name} 和 common/{name} 不同：執行 run_tests.py --sync')
            ok = False
    if ok:
        print('✓ handheld_rp2040/src/ 和 common/ 一致')
    if UPSTREAM.exists():
        differ = [n for n in ('rc_proto.h', 'rc_proto.c', 'joy_api.h', 'joy_api.c')
                  if (UPSTREAM / n).read_bytes() != (COMMON / n).read_bytes()]
        print('⚠ 和上一層的 src/ 不同：' + '、'.join(differ) if differ else '✓ common/ 和上一層的 src/ 一致')
    return ok


def find_arduino_cli():
    found = shutil.which('arduino-cli')
    bundled = Path(os.environ.get('LOCALAPPDATA', '')) / 'Programs/Arduino IDE/resources/app/lib/backend/resources'
    for candidate in [found, bundled / 'arduino-cli.exe']:
        if candidate and Path(candidate).exists():
            return str(candidate)
    return None


def rp2040():
    cli = find_arduino_cli()
    if not cli:
        print('✗ 找不到 arduino-cli（安裝 Arduino IDE 2 或 arduino-cli）')
        return False
    ran = subprocess.run([cli, 'compile', '--fqbn', FQBN, str(SKETCH)], capture_output=True, text=True,
                         encoding='utf-8', errors='replace')
    if ran.returncode:
        print('✗ RP2040 編譯失敗（需要 arduino-pico 核心和「Pico PIO USB」程式庫）\n' + ran.stdout + ran.stderr)
        return False
    size = [line for line in ran.stdout.splitlines() if line.strip()]
    print('✓ RP2040 程式編譯成功' + (f'：{size[0]}' if size else ''))
    return True


def main():
    parser = argparse.ArgumentParser(description='離線測試：不需要硬體')
    parser.add_argument('--sync', action='store_true', help='先把 common/ 的 C 檔複製到 handheld_rp2040/src/')
    parser.add_argument('--rp2040', action='store_true', help='也用 arduino-cli 編譯 RP2040 程式')
    args = parser.parse_args()
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')

    results = [copies(args.sync), c_tests(), python_tests()]
    if args.rp2040:
        results.append(rp2040())
    print('\n全部通過' if all(results) else '\n有項目沒通過')
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
