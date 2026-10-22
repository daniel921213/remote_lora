"""Check that two transparent RYLR998 modules carry binary SBUS frames unchanged.

  python lora_binary_test.py --port COM7 --car-port COM8
  python lora_binary_test.py --port COM7 --car-port COM8 --count 1000
  python lora_binary_test.py --port COM7 --car-port COM8 --reverse

Sends standard 25-byte SBUS frames (what the handheld MCU will send) from --port and
compares what comes out of --car-port byte for byte. It answers two questions before the
handheld format is fixed:
  1. Does transparent mode pass binary bytes such as 0x00, 0x0A and 0x0D unchanged?
  2. Does the module ever deliver a damaged packet, or does it always drop it?
"""

import argparse
import random
import sys
import time

BAUD = 115200
FRAME_SIZE = 25
NEUTRAL = 1002
TRICKY = (0x0A, 0x0D)  # line ends: the bytes most likely to split a text-oriented module
QUIET = 0.03           # a frame is complete after this long with no new bytes


def sbus_pack(channels, flags=0):
    """Same bit layout as MowerL476 App/Src/sbus.c sbus_pack()."""
    out = bytearray(FRAME_SIZE)
    out[0] = 0x0F
    for c, value in enumerate(channels):
        for bit in range(11):
            p = c * 11 + bit
            out[1 + p // 8] |= ((value >> bit) & 1) << (p % 8)
    out[23] = flags & 0x0F
    return bytes(out)


def make_frame(index, rng):
    """R12-like channel values. CH16 carries the frame number, so a late frame is recognisable.
    Every other frame is redrawn until it contains a line-end byte."""
    while True:
        ch = [rng.randint(282, 1722) for _ in range(4)]          # sticks
        ch += [rng.choice((282, 1002, 1722)) for _ in range(2)]  # three-position switches
        ch += [rng.choice((282, 1722)) for _ in range(4)]        # buttons A-D
        ch += [NEUTRAL] * 5
        ch.append(index % 2048)
        frame = sbus_pack(ch)
        if index % 2 or any(b in TRICKY for b in frame):
            return frame


def open_port(name):
    try:
        import serial
    except ImportError:
        sys.exit('需要 pyserial：pip install pyserial')
    try:
        port = serial.serial_for_url(name, do_not_open=True)
        port.baudrate, port.timeout = BAUD, 0
        port.dtr = port.rts = False  # do not drive DTR/RTS; they can hold a module in reset
        port.open()
    except serial.SerialException as error:
        sys.exit(f'無法開啟 {name}：{error}\n（終端機或其他程式是否正在使用這個 COM？）')
    port.reset_input_buffer()
    return port


def collect(port, deadline):
    """Reads until `deadline`, or until QUIET seconds pass after the last byte.
    Returns (data, number of reads that returned data, time of the last byte)."""
    data, chunks, last_at = b'', 0, None
    while True:
        now = time.monotonic()
        if now >= deadline or (last_at is not None and now - last_at >= QUIET):
            return data, chunks, last_at
        got = port.read(port.in_waiting or 1)
        if got:
            data += got
            chunks += 1
            last_at = time.monotonic()
        else:
            time.sleep(0.001)


def run(tx, rx, count, period, timeout, seed, log=print):
    rng = random.Random(seed)
    stats = {'ok': 0, 'lost': 0, 'corrupt': 0, 'length': 0, 'stray': 0}
    tricky = {'sent': 0, 'ok': 0}
    latencies, split, sent = [], 0, 0
    rx.reset_input_buffer()
    try:
        for index in range(count):
            stray = rx.read(rx.in_waiting or 0)
            if stray:
                stats['stray'] += 1
                log(f'#{index:4d} 送出前收到多餘的 {len(stray)} bytes：{stray.hex(" ").upper()}')

            frame = make_frame(index, rng)
            has_tricky = any(b in TRICKY for b in frame)
            start = time.monotonic()
            tx.write(frame)
            data, chunks, last_at = collect(rx, start + timeout)

            if data == frame:
                stats['ok'] += 1
                latencies.append((last_at - start) * 1000)
                split += chunks > 1
                if has_tricky:
                    tricky['ok'] += 1
            elif not data:
                stats['lost'] += 1
                log(f'#{index:4d} 遺失')
            elif len(data) == FRAME_SIZE:
                stats['corrupt'] += 1
                diff = ' '.join(f'[{i}]{a:02X}→{b:02X}' for i, (a, b) in enumerate(zip(frame, data)) if a != b)
                log(f'#{index:4d} 內容錯誤：{diff}')
            else:
                stats['length'] += 1
                log(f'#{index:4d} 長度 {len(data)}（應為 {FRAME_SIZE}）\n'
                    f'      送出 {frame.hex(" ").upper()}\n      收到 {data.hex(" ").upper()}')
            tricky['sent'] += has_tricky
            sent += 1

            if sent % 20 == 0:
                log(f'… 已送 {sent}／{count}，一致 {stats["ok"]}')
            time.sleep(max(0.0, start + period - time.monotonic()))
    except KeyboardInterrupt:
        log('\n已中止，以下是中止前的結果')
    return stats, tricky, latencies, split, sent


def report(stats, tricky, latencies, split, count, log=print):
    log('\n==== 結果 ====')
    if not count:
        log('一包都還沒送完。')
        return
    log(f'送出 {count} 包：一致 {stats["ok"]}、遺失 {stats["lost"]}、內容錯誤 {stats["corrupt"]}、'
        f'長度不對 {stats["length"]}、多餘資料 {stats["stray"]}')
    log(f'含 0x0A／0x0D 的封包：{tricky["sent"]} 包，其中一致 {tricky["ok"]} 包')
    if latencies:
        log(f'延遲（送出到收完，含 USB 轉接延遲）：最短 {min(latencies):.0f} ms、'
            f'平均 {sum(latencies) / len(latencies):.0f} ms、最長 {max(latencies):.0f} ms')
        log(f'電腦分成多段才收完的封包：{split} 包（USB 轉接器會自己分段，只供參考）')

    log('')
    if stats['corrupt'] or stats['length'] or stats['stray']:
        if tricky['ok'] < tricky['sent'] and stats['ok'] > tricky['ok']:
            log('✗ 含 0x0A／0x0D 的封包出問題：透傳不能直接送二進位，要改送十六進位文字。')
        if stats['corrupt']:
            log('✗ 模組會吐出內容錯誤的封包：SBUS 外面要自己加 CRC。')
        if stats['length'] or stats['stray']:
            log('✗ 封包被切開、併在一起或多出資料：看上面的十六進位找原因。')
    elif stats['ok'] == 0:
        log('✗ 一包都沒收到：確認兩顆都是透傳模式、UART 115200，以及 COM 有沒有接反。')
    else:
        log('✓ 收到的封包全部原封不動，包含 0x0A／0x0D：可以直接送 SBUS 二進位。')
        log('  壞封包會不會被模組丟掉，要拉遠距離或調低功率（AT+CRFOP）多跑幾次（--count 1000），'
            '內容錯誤一直是 0 才算數。')
    if stats['lost']:
        log(f'  掉包率 {stats["lost"] / count:.1%}。')


def main():
    parser = argparse.ArgumentParser(description='確認透傳 LoRa 能否原封不動傳送 SBUS 二進位封包')
    parser.add_argument('--port', required=True, help='手把那顆模組的 COM，例如 COM7')
    parser.add_argument('--car-port', required=True, help='車上那顆模組的 COM，例如 COM8')
    parser.add_argument('--count', type=int, default=200, help='送幾包，預設 200')
    parser.add_argument('--period', type=float, default=0.2, help='每包間隔秒數，預設 0.2')
    parser.add_argument('--timeout', type=float, default=0.5, help='每包最多等幾秒，預設 0.5')
    parser.add_argument('--reverse', action='store_true', help='反方向：從車上那顆送到手把那顆')
    parser.add_argument('--seed', type=int, default=1, help='亂數種子，同一個種子會送出同一組封包')
    args = parser.parse_args()
    if args.count < 1 or args.period <= 0 or args.timeout <= QUIET:
        parser.error('--count 至少 1；--period 要大於 0；--timeout 要大於 0.03')
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')

    # Must match MowerL476 sbus_pack() byte for byte (values printed by the C code).
    assert sbus_pack([NEUTRAL] * 16).hex().upper() == '0FEA539FFAD4A73EF5A94F7DEA539FFAD4A73EF5A94F7D0000'
    assert sbus_pack([282, 1722, 1002, 300, 282, 1722, 282, 1722, 1722, 282] + [NEUTRAL] * 5 + [2047]) \
        .hex().upper() == '0F1AD1B5FA58A2115D6B44D7BAD688FAD4A73EF5A9EFFF0000'

    first = open_port(args.port)
    second = first if args.car_port == args.port else open_port(args.car_port)
    tx, rx = (second, first) if args.reverse else (first, second)
    names = (args.car_port, args.port) if args.reverse else (args.port, args.car_port)
    print(f'{names[0]} 送出 → {names[1]} 接收，共 {args.count} 包，每 {args.period:.2f} 秒一包')
    report(*run(tx, rx, args.count, args.period, args.timeout, args.seed))


if __name__ == '__main__':
    main()
