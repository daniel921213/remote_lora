"""lora_mux 離線測試：假的組員 LoRa 程式（同樣的 API、資料埠一次一個程式）＋真的 socket。"""

import base64
import json
import select
import socket
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / 'orin'), str(ROOT / 'handheld_pc'), str(ROOT / 'common'), str(ROOT / 'tests')]

import lora_mux as lm  # noqa: E402
import remote_logic as rl  # noqa: E402
from handheld_lib import SLOT, Handheld, tools  # noqa: E402
from joy_lib import Joy  # noqa: E402
from lora_ros_node import Remote, open_lora  # noqa: E402
from test_lora_ros_node import FakeOut  # noqa: E402

# 地面站的 LoRa2 封包（A5 車號 02 種類 長度 …）：payload 裡故意放 "+RC"，不是遙控的開頭
HEARTBEAT = bytes([0xA5, 1, 2, 5, 5, 7, 1, 0]) + b'+RC\x00\x01' + b'\x12\x34'
STATUS = bytes([0xA5, 1, 2, 1, 54, 8, 1, 0]) + bytes(54) + b'\x56\x78'
LINE = b'+RCV=1,32,01110100000001000000000000007AC5,0,0\r\n'  # 手把那句（Orin 收到）
REPLY = b'AT+SEND=1,16,0102030405060708\r\n'  # 車子回的那句（筆電收到）


def wait_until(condition, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.005)
    return condition()


def http(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as error:
        with error:
            return error.code, json.load(error)


class FakeBridge:
    """組員 lora_bridge 的替身。air：寫進 LoRa 模組、要送上空中的資料；receive()：空中收到的資料。
    peer：另一頭的 FakeBridge，這頭送上空中的，那頭就收到。"""

    HELLO = b'{"type":"hello","protocol":"kcp-bridge-v1"}\n'

    def __init__(self):
        self.mode = 'raw'
        self.air = bytearray()
        self.incoming = deque()  # 不用鎖：兩頭互相送時才不會卡住
        self.peer = None
        self.client = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.listener = socket.create_server(('127.0.0.1', 0))
        self.data_port = self.listener.getsockname()[1]
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), self.handler())
        self.api = f'http://127.0.0.1:{self.http.server_address[1]}'
        self.threads = [threading.Thread(target=self.http.serve_forever, args=(0.05,), daemon=True),
                        threading.Thread(target=self.loop, daemon=True)]
        for thread in self.threads:
            thread.start()

    def status(self):
        return {'mode': self.mode, 'data': f'tcp://127.0.0.1:{self.data_port}', 'worker_alive': True,
                'state': 'ready', 'uart_open': True, 'device': '/dev/lora_gs'}

    def set_mode(self, mode):
        with self.lock:
            changed = mode != self.mode
            if changed:
                self.drop()
                self.mode = mode
        return {'ok': True, 'message': 'Mode switched' if changed else 'Already in requested mode', **self.status()}

    def receive(self, data):
        self.incoming.append(bytes(data))

    def transmit(self, data):
        self.air += data
        if self.peer is not None:
            self.peer.receive(data)

    def drop(self):
        if self.client is not None:
            self.client.close()
            self.client = None

    def loop(self):
        text = bytearray()
        while not self.stop.is_set():
            try:
                self.step(text)
            except (OSError, ValueError):  # 換模式時 client 被另一個執行緒關掉
                with self.lock:
                    self.drop()

    def step(self, text):
        readers = [self.listener] + ([self.client] if self.client else [])
        readable = select.select(readers, [], [], 0.005)[0]
        with self.lock:
            if self.listener in readable:
                sock, _ = self.listener.accept()
                if self.client is not None:
                    sock.close()  # 一次只給一個程式
                else:
                    self.client = sock
                    text.clear()
                    if self.mode == 'kcp':
                        sock.sendall(self.HELLO)
            elif self.client is not None and self.client in readable:
                data = self.client.recv(4096)
                if not data:
                    self.drop()
                elif self.mode == 'raw':
                    self.transmit(data)
                else:
                    text += data
                    while b'\n' in text:
                        line, _, rest = bytes(text).partition(b'\n')
                        text[:] = rest
                        request = json.loads(line)
                        self.transmit(base64.b64decode(request['data']))
                        self.client.sendall(json.dumps({'type': 'ack', 'id': request['id']}).encode() + b'\n')
            while self.incoming:
                data = self.incoming.popleft()
                if self.client is None:
                    continue  # 沒人連著：丟掉，跟真的一樣
                if self.mode == 'kcp':
                    data = json.dumps({'type': 'rx', 'data': base64.b64encode(data).decode()}).encode() + b'\n'
                self.client.sendall(data)

    def handler(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def reply(self, obj):
                data = json.dumps(obj).encode()
                self.send_response(200)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                self.reply(bridge.status())

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                self.reply(bridge.set_mode(body['mode']))

            def log_message(self, *args):
                pass

        return Handler

    def close(self):
        self.stop.set()
        self.http.shutdown()
        self.http.server_close()
        for thread in self.threads:
            thread.join()
        with self.lock:
            self.drop()
        self.listener.close()


class Peer:
    """連到分流程式的一支程式（地面站或遙控）。"""

    def __init__(self, port):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=2)
        self.got = bytearray()
        self.closed = False

    def read(self):
        while select.select([self.sock], [], [], 0)[0]:
            try:
                data = self.sock.recv(4096)
            except OSError:
                data = b''
            if not data:
                self.closed = True
                break
            self.got += data
        return bytes(self.got)

    def wait_for(self, expected, timeout=2.0):
        return wait_until(lambda: self.read() == expected, timeout)

    def send(self, data):
        self.sock.sendall(data)

    def close(self):
        self.sock.close()


class TestSplitter(unittest.TestCase):
    def test_line_between_frames(self):
        lines, ground = lm.Splitter(b'+RCV=').feed(HEARTBEAT + LINE + STATUS, 0)
        self.assertEqual((lines, ground), ([LINE], HEARTBEAT + STATUS))

    def test_any_split(self):
        splitter, lines, ground = lm.Splitter(b'+RCV='), [], b''
        for byte in HEARTBEAT + LINE + STATUS + LINE:
            got = splitter.feed(bytes([byte]), 0)
            lines += got[0]
            ground += got[1]
        self.assertEqual((lines, ground), ([LINE, LINE], HEARTBEAT + STATUS))

    def test_cut_line_does_not_swallow_the_frame_behind(self):
        lines, ground = lm.Splitter(b'+RCV=').feed(LINE[:20] + STATUS + LINE, 0)
        self.assertEqual((lines, ground), ([LINE], STATUS))

    def test_held_start_is_released(self):
        splitter = lm.Splitter(b'+RCV=')
        self.assertEqual(splitter.feed(STATUS + b'+R', 0), ([], STATUS))  # "+R" 可能是遙控的開頭：先留著
        self.assertEqual(splitter.flush(0.05), b'')
        self.assertEqual(splitter.flush(0.2), b'+R')  # 後面沒來：還給地面站

    def test_laptop_marker(self):
        lines, ground = lm.Splitter(b'AT+SEND=').feed(STATUS + REPLY + LINE, 0)
        self.assertEqual((lines, ground), ([REPLY], STATUS + LINE))


class TestFrameCutter(unittest.TestCase):
    def test_whole_frames_only(self):
        cutter = lm.FrameCutter()
        self.assertEqual(cutter.feed(STATUS[:7], 0), [])
        self.assertEqual(cutter.feed(STATUS[7:] + HEARTBEAT[:3], 0.01), [STATUS])
        self.assertEqual(cutter.take(0.05), [])
        self.assertEqual(cutter.take(0.2), [HEARTBEAT[:3]])  # 剩下的一直沒來：照原樣送

    def test_other_bytes_pass(self):
        self.assertEqual(lm.FrameCutter().feed(b'xyz' + HEARTBEAT, 0), [b'xyz', HEARTBEAT])


class MuxCase(unittest.TestCase):
    side = 'orin'

    def setUp(self):
        self.bridge = FakeBridge()
        self.logs = []
        self.mux = lm.Mux(self.side, self.bridge.api, api_port=0, data_port=0, remote_port=0,
                          remote_window=0.5, log=self.logs.append)
        self.api = f'http://127.0.0.1:{self.mux.api_port}'
        self.thread = threading.Thread(target=self.mux.run, daemon=True)
        self.thread.start()
        self.peers = []
        self.assertTrue(wait_until(lambda: self.bridge.client is not None))

    def tearDown(self):
        for peer in self.peers:
            peer.close()
        self.mux.stop.set()
        self.thread.join()
        self.bridge.close()

    def connect(self, port, client_attr):
        peer = Peer(port)
        self.peers.append(peer)
        self.assertTrue(wait_until(lambda: getattr(self.mux, client_attr) is not None))
        return peer

    def ground(self):
        return self.connect(self.mux.data_port, 'ground')

    def remote(self):
        return self.connect(self.mux.remote_port, 'remote')


class TestRaw(MuxCase):
    def test_routes_both_ways(self):
        ground, remote = self.ground(), self.remote()
        self.bridge.receive(HEARTBEAT + LINE[:9])
        self.bridge.receive(LINE[9:] + STATUS)
        self.assertTrue(remote.wait_for(LINE))
        self.assertTrue(ground.wait_for(HEARTBEAT + STATUS))
        remote.send(REPLY)
        self.assertTrue(wait_until(lambda: self.bridge.air == REPLY))

    def test_status_looks_like_the_bridge(self):
        code, info = http(self.api + '/status')
        self.assertEqual(code, 200)
        self.assertEqual(info['mode'], 'raw')
        self.assertEqual(info['data'], f'tcp://127.0.0.1:{self.mux.data_port}')
        self.assertTrue(info['worker_alive'])
        self.assertEqual(info['device'], '/dev/lora_gs')
        self.assertEqual(info['upstream']['data'], f'tcp://127.0.0.1:{self.bridge.data_port}')
        self.assertEqual(info['mux']['side'], 'orin')

    def test_ground_goes_at_once_when_remote_is_quiet(self):
        ground = self.ground()
        ground.send(STATUS)
        self.assertTrue(wait_until(lambda: self.bridge.air == STATUS, 0.1))

    def test_ground_waits_for_the_remote_turn_and_stays_whole(self):
        ground, remote = self.ground(), self.remote()
        self.bridge.receive(LINE)
        self.assertTrue(remote.wait_for(LINE))
        remote.send(REPLY)
        self.assertTrue(wait_until(lambda: self.bridge.air == REPLY))
        ground.send(STATUS[:7])  # 遙控正在動：地面站的封包先排隊
        time.sleep(0.02)
        ground.send(STATUS[7:])
        time.sleep(0.05)
        self.assertEqual(self.bridge.air, REPLY)
        remote.send(REPLY)  # 遙控下一句：先送遙控，地面站的整包緊跟在後
        self.assertTrue(wait_until(lambda: self.bridge.air == REPLY + REPLY + STATUS))

    def test_ground_waits_at_most_the_hold_time(self):
        ground, remote = self.ground(), self.remote()
        remote.send(REPLY)
        self.assertTrue(wait_until(lambda: self.bridge.air == REPLY))
        sent = time.monotonic()
        ground.send(HEARTBEAT)
        self.assertTrue(wait_until(lambda: self.bridge.air == REPLY + HEARTBEAT, 1.0))
        self.assertLess(time.monotonic() - sent, lm.GROUND_HOLD + 0.1)

    def test_one_program_per_port(self):
        self.ground()
        second = Peer(self.mux.data_port)
        self.peers.append(second)
        self.assertTrue(wait_until(lambda: second.read() == b'' and second.closed))

    def test_reconnects_to_the_bridge(self):
        ground = self.ground()
        with self.bridge.lock:
            self.bridge.drop()  # 組員的程式重開、USB 拔插…
        self.assertTrue(wait_until(lambda: self.bridge.client is not None, 3.0))
        self.bridge.receive(HEARTBEAT)
        self.assertTrue(ground.wait_for(HEARTBEAT))


class TestTransfer(MuxCase):
    def start_transfer(self):
        code, info = http(self.api + '/mode', {'mode': 'kcp'})
        self.assertEqual(code, 200)
        self.assertTrue(info['ok'] and info['worker_alive'])
        self.assertEqual((info['mode'], info['data']), ('kcp', f'tcp://127.0.0.1:{self.mux.data_port}'))
        self.assertEqual(self.bridge.mode, 'kcp')
        transfer = self.ground()
        self.assertTrue(transfer.wait_for(FakeBridge.HELLO))
        return transfer

    def test_refused_while_remote_is_active(self):
        self.bridge.receive(LINE)  # 手把開著
        self.assertTrue(wait_until(lambda: self.mux.remote_active(time.monotonic())))
        code, info = http(self.api + '/mode', {'mode': 'kcp'})
        self.assertEqual(code, 409)
        self.assertFalse(info['ok'])
        self.assertIn('remote control has priority', info['message'])
        self.assertEqual((self.bridge.mode, info['mode'], info['mux']['transfers_refused']), ('raw', 'raw', 1))
        time.sleep(0.6)  # 遙控關掉超過 remote_window：可以傳
        self.start_transfer()

    def test_transfer_then_back_to_raw(self):
        old = self.ground()
        transfer = self.start_transfer()
        self.assertTrue(old.closed or wait_until(lambda: old.read() == b'' and old.closed))  # 換模式：舊的被斷掉
        payload = b'\x01kcp-frame'
        transfer.send(json.dumps({'type': 'tx', 'id': 1, 'data': base64.b64encode(payload).decode()}).encode() + b'\n')
        self.assertTrue(wait_until(lambda: self.bridge.air == payload))
        self.bridge.receive(b'\x02from-air')
        rx = json.dumps({'type': 'rx', 'data': base64.b64encode(b'\x02from-air').decode()}).encode() + b'\n'
        self.assertTrue(wait_until(lambda: transfer.read().endswith(rx)))

        code, info = http(self.api + '/mode', {'mode': 'raw'})
        self.assertEqual((code, info['ok'], info['mode'], self.bridge.mode), (200, True, 'raw', 'raw'))
        self.assertTrue(wait_until(lambda: transfer.read() is not None and transfer.closed))
        ground = self.ground()
        self.assertTrue(wait_until(lambda: self.bridge.client is not None))
        self.bridge.receive(STATUS)
        self.assertTrue(ground.wait_for(STATUS))

    def test_remote_on_air_aborts_the_transfer(self):
        remote = self.remote()
        transfer = self.start_transfer()
        self.bridge.receive(LINE)  # 傳到一半，手把開機
        self.assertTrue(wait_until(lambda: self.bridge.mode == 'raw'))
        self.assertTrue(wait_until(lambda: transfer.read() is not None and transfer.closed))
        _, info = http(self.api + '/status')
        self.assertEqual((info['mode'], info['mux']['transfers_aborted']), ('raw', 1))
        self.assertTrue(wait_until(lambda: self.mux.up is not None and self.bridge.client is not None))
        self.bridge.receive(LINE)
        self.assertTrue(remote.wait_for(LINE))  # 遙控接手


class TestLaptop(MuxCase):
    side = 'laptop'

    def test_routes_replies_and_sends_lines(self):
        ground, remote = self.ground(), self.remote()
        self.bridge.receive(STATUS + REPLY)
        self.assertTrue(remote.wait_for(REPLY))
        self.assertTrue(ground.wait_for(STATUS))
        remote.send(LINE)
        self.assertTrue(wait_until(lambda: self.bridge.air == LINE))

    def test_handheld_sending_aborts_the_transfer(self):
        remote = self.remote()
        code, _ = http(self.api + '/mode', {'mode': 'kcp'})
        self.assertEqual((code, self.bridge.mode), (200, 'kcp'))
        remote.send(LINE)  # 筆電上的 handheld_pc 開始送
        self.assertTrue(wait_until(lambda: self.bridge.mode == 'raw' and self.mux.mode == 'raw'))
        self.assertTrue(wait_until(lambda: self.bridge.client is not None))
        remote.send(LINE)
        self.assertTrue(wait_until(lambda: self.bridge.air == LINE))


class TestTwoEnds(unittest.TestCase):
    """一組 LoRa：筆電這頭是手把（真的 C 程式）＋人機介面，Orin 那頭是 lora_ros_node＋main_control，各有一個分流程式。"""

    def setUp(self):
        self.bridges = FakeBridge(), FakeBridge()
        self.bridges[0].peer, self.bridges[1].peer = self.bridges[1], self.bridges[0]
        self.muxes = [lm.Mux(side, bridge.api, api_port=0, data_port=0, remote_port=0, log=lambda text: None)
                      for side, bridge in zip(('laptop', 'orin'), self.bridges)]
        self.threads = [threading.Thread(target=mux.run, daemon=True) for mux in self.muxes]
        for thread in self.threads:
            thread.start()
        laptop, orin = self.muxes
        self.assertTrue(wait_until(lambda: laptop.up is not None and orin.up is not None))
        # 跟實際一樣的開法：handheld_pc 用 tools.open_serial，node 用 open_lora
        self.handheld_port = tools.open_serial(f'socket://127.0.0.1:{laptop.remote_port}')
        self.out = FakeOut()
        self.remote = Remote(open_lora(f'socket://127.0.0.1:{orin.remote_port}'), Joy(), rl.RemoteLogic(5, 3),
                             self.out)
        self.remote.state = rl.AUTO
        self.handheld = Handheld(0)
        self.gateway, self.main_control = Peer(laptop.data_port), Peer(orin.data_port)
        self.assertTrue(wait_until(lambda: all(m.ground is not None and m.remote is not None for m in self.muxes)))
        self.start = time.monotonic()
        self.ground_sent = 0

    def tearDown(self):
        self.remote.close(publish=False)
        self.handheld.close()
        self.handheld_port.close()
        self.gateway.close()
        self.main_control.close()
        for mux in self.muxes:
            mux.stop.set()
        for thread in self.threads:
            thread.join()
        for bridge in self.bridges:
            bridge.close()

    def run_for(self, seconds, drive=0, buttons=()):
        """手把每 0.01 秒取樣；兩邊的地面站程式每 0.25 秒各送一包。"""
        mask = sum(1 << SLOT[name] for name in buttons)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now_ms = int((time.monotonic() - self.start) * 1000)
            self.handheld.sample(True, drive, 0, mask)
            line = self.handheld.take_line(now_ms)
            if line:
                self.handheld_port.write(line)
            self.handheld.feed(self.handheld_port.read(4096), now_ms)
            self.handheld.poll(now_ms)
            self.remote.step()
            if now_ms >= self.ground_sent * 250:
                self.gateway.send(HEARTBEAT)  # 人機介面的心跳
                self.main_control.send(STATUS)  # 車況
                self.ground_sent += 1
            time.sleep(0.01)

    def test_remote_and_ground_share_one_radio(self):
        self.run_for(0.5)
        self.run_for(1.0, drive=100)
        self.assertEqual(self.out.wheel_log[-1], (5.0, 5.0))
        self.assertTrue(self.handheld.info().link_up)
        self.run_for(0.15, drive=100, buttons=['RB'])  # 按鍵也要過得去
        self.run_for(0.4, drive=100)
        self.assertEqual((self.out.modes[-1], self.out.wheel_log[-1]), (rl.ORIN, (3.0, 3.0)))
        self.assertNotIn('遙控器失聯：停車，回到 JOY', self.out.logs)
        # 地面站兩個方向都整包、照順序收到，沒有混進遙控的句子
        self.assertTrue(wait_until(lambda: self.main_control.read() == HEARTBEAT * self.ground_sent))
        self.assertTrue(wait_until(lambda: self.gateway.read() == STATUS * self.ground_sent))
        self.assertGreater(self.ground_sent, 7)


if __name__ == '__main__':
    unittest.main()
