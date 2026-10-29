"""Sets the laptop's RYLR998 (CH340 adapter) to the same radio settings as the Orin's module.

  python lora_setup.py              gs profile (923 MHz), the channel the remote uses
  python lora_setup.py --profile rtcm

The transparent firmware only takes AT commands in the first seconds after power-on, and this
adapter's DTR is not wired to the module's reset, so: start this, then unplug and replug the
module. Values and the verify-by-readback follow the teammate's lora/setting/usb/lora_init.py.
"""

import argparse
import sys
import time

from handheld_lib import tools

PROFILES = {  # teammate's lora/setting/params/*.json
    'gs': {'band': 923000000, 'parameter': (5, 9, 1, 24), 'crfop': 22},
    'rtcm': {'band': 920500000, 'parameter': (5, 9, 1, 24), 'crfop': 22},
}
CH340 = (0x1A86, 0x7523)
AT_WINDOW = 8.5     # seconds after power-on we still send AT commands (the firmware allows ~10)
BOOT_WINDOW = 11.0  # keep the port until the module has left its AT window
WAIT_LIMIT = 600


def adapters():
    from serial.tools import list_ports
    return [p.device for p in list_ports.comports() if (p.vid, p.pid) == CH340]


def command(port, text, expected, deadline):
    """Sends one AT command and waits for the exact reply line; +ERR stops the setup."""
    port.reset_input_buffer()
    port.write((text + '\r\n').encode('ascii'))
    buffer = bytearray()
    while time.monotonic() < deadline:
        buffer.extend(port.read(port.in_waiting or 1))
        while b'\n' in buffer:
            line, _, rest = buffer.partition(b'\n')
            buffer = bytearray(rest)
            reply = line.decode('ascii', errors='replace').strip()
            if reply.casefold() == expected.casefold():
                return
            if reply.startswith('+ERR'):
                sys.exit(f'{text} 失敗：{reply}')
    raise TimeoutError(text)


def wait_for_replug(name):
    limit = time.monotonic() + WAIT_LIMIT
    present = lambda: name in adapters() if name else bool(adapters())
    if present():
        print('請把 LoRa 拔掉…', flush=True)
        while present():
            if time.monotonic() > limit:
                sys.exit('等太久，沒有偵測到拔除。')
            time.sleep(0.05)
    print('請插回去…', flush=True)
    while not present():
        if time.monotonic() > limit:
            sys.exit('等太久，沒有偵測到插回。')
        time.sleep(0.02)
    return time.monotonic(), name or adapters()[0]


def open_port(name, since):
    import serial
    while True:  # Windows may need a moment before the new COM opens
        port = serial.Serial()
        port.port, port.baudrate, port.timeout = name, tools.BAUD, 0.02
        port.dtr = port.rts = False
        try:
            port.open()
            return port
        except serial.SerialException:
            if time.monotonic() - since > 3:
                raise
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description='把筆電的 LoRa 設成跟 Orin 那顆相同的無線設定')
    parser.add_argument('--profile', choices=sorted(PROFILES), default='gs', help='預設 gs（923 MHz）')
    parser.add_argument('--port', help='例如 COM5；不填就自動找 CH340')
    args = parser.parse_args()
    tools.utf8_console()
    profile = PROFILES[args.profile]
    print(f'{args.profile}：{profile["band"] / 1e6:g} MHz、參數 {",".join(map(str, profile["parameter"]))}、'
          f'功率 {profile["crfop"]}、{tools.BAUD}', flush=True)

    plugged, name = wait_for_replug(args.port)
    print(f'偵測到 {name}，設定中…', flush=True)
    port = open_port(name, plugged)
    try:
        window = plugged + AT_WINDOW
        while True:
            try:
                command(port, 'AT', '+OK', min(window, time.monotonic() + 0.25))
                break
            except TimeoutError:
                if time.monotonic() >= window:
                    sys.exit('模組沒有回應 AT：確認接線和供電後再試一次。')
        parameter = ','.join(map(str, profile['parameter']))
        for key, value, readback in (('BAND', f'{profile["band"]},M', str(profile['band'])),
                                       ('PARAMETER', parameter, parameter),
                                       ('CRFOP', str(profile['crfop']), str(profile['crfop']))):
            try:
                command(port, f'AT+{key}={value}', '+OK', window)
                command(port, f'AT+{key}?', f'+{key}={readback}', window)
            except TimeoutError:
                sys.exit(f'{key} 沒有完成：開機設定時間已過，請重來一次。')
            print(f'  {key} 已確認', flush=True)
        try:
            command(port, 'AT+IPR?', f'+IPR={tools.BAUD}', window)
        except TimeoutError:
            sys.exit(f'模組的鮑率不是 {tools.BAUD}，這支程式沒辦法處理。')
        print('  鮑率已確認', flush=True)
        time.sleep(max(0.0, plugged + BOOT_WINDOW - time.monotonic()))
    finally:
        port.close()
    print(f'完成：{name} 已回到透傳模式，可以開始測試。', flush=True)


if __name__ == '__main__':
    main()
