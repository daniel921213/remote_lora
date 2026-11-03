#!/usr/bin/env python3
"""分流程式：遙控和人機介面共用一顆 LoRa。夾在組員的 LoRa 程式（lora_bridge／pc_lora_bridge）和兩邊的程式中間。

  Orin： python3 lora_mux.py orin
         組員的 gs 服務照常在 8767/8768；main_control 加 --bridge-api http://127.0.0.1:8787；遙控連 tcp 8789
  筆電： python lora_mux.py laptop --upstream-api http://127.0.0.1:8797 --api-port 8767
         組員的 pc_lora_bridge 改開在 8797/8798；人機介面看到 8767 已經有人，就直接用分流程式

收：遙控的句子（Orin 收 "+RCV="、筆電收 "AT+SEND="）給遙控程式，其他（地面站的 LoRa2 封包）給地面站程式。
送：遙控優先。遙控在動的時候，地面站的封包等遙控那句送出去，緊接在後面送，最多等 0.2 秒；遙控沒在動就馬上送。
傳檔（KCP）：最近 3 秒有遙控的資料就拒絕；傳到一半遙控出現，就中斷傳檔、切回一般模式。

對地面站程式來說，它就是組員的 LoRa 程式：一樣的 GET /status、POST /mode、POST /reset，資料埠一樣一次只給
一個程式連，換模式時一樣會把它斷掉。所以 main_control、人機介面、傳檔程式都不用改。
"""

import argparse
import base64
import json
import queue
import select
import socket
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

MARKERS = {'orin': b'+RCV=', 'laptop': b'AT+SEND='}  # 收到的哪一種句子是遙控的
MAX_LINE = 128        # 遙控的句子最長幾個 bytes（實際 31～48）
MAX_PAYLOAD = 54      # LoRa2 封包的 payload 上限
STALE = 0.1           # 秒：卡在一半的句子或封包，等這麼久就放棄
GROUND_HOLD = 0.2     # 秒：地面站的封包最多等遙控這麼久
REMOTE_IDLE = 0.3     # 秒：遙控這麼久沒送東西，就當它沒在動
HELLO_DELAY = 0.05    # 秒：傳檔程式連上後，等這麼久才把 KCP 的 hello 給它
BUFFER_LIMIT = 65536


class Splitter:
    """Radio bytes -> remote lines and ground bytes.

    A remote line is the marker, printable ASCII, then '\\n'. Any other byte ends a line early, so a
    line cut short by a lost packet never swallows the LoRa2 frame behind it."""

    def __init__(self, marker):
        self.marker = marker
        self.held = bytearray()  # a possible marker start, or the line so far
        self.in_line = False
        self.since = 0.0

    def feed(self, data, now):
        lines, ground = [], bytearray(self.flush(now))
        for byte in data:
            if self.in_line:
                if byte == 0x0A:
                    self.held.append(byte)
                    lines.append(bytes(self.held))
                    self.held.clear()
                    self.in_line = False
                    continue
                if (0x20 <= byte < 0x7F or byte == 0x0D) and len(self.held) < MAX_LINE:
                    self.held.append(byte)
                    continue
                self.held.clear()  # cut short: drop it; this byte belongs to someone else
                self.in_line = False
            if byte == self.marker[len(self.held)]:
                if not self.held:
                    self.since = now
                self.held.append(byte)
                self.in_line = len(self.held) == len(self.marker)
            else:
                ground += self.held
                self.held.clear()
                if byte == self.marker[0]:
                    self.since = now
                    self.held.append(byte)
                else:
                    ground.append(byte)
        return lines, bytes(ground)

    def flush(self, now):
        """Ground bytes held too long as a possible marker start; a stalled line is dropped."""
        ground = b''
        if self.held and now - self.since >= STALE:
            if not self.in_line:
                ground = bytes(self.held)
            self.held.clear()
            self.in_line = False
        return ground


class FrameCutter:
    """Ground program bytes -> whole LoRa2 frames (A5 id 02 type len seq session payload crc), so a
    remote line never goes out in the middle of one. Other bytes pass through as they came."""

    def __init__(self):
        self.buffer = bytearray()
        self.since = 0.0

    def feed(self, data, now):
        if not self.buffer:
            self.since = now
        self.buffer += data
        return self.take(now)

    def take(self, now):
        frames = []
        while self.buffer:
            if self.buffer[0] != 0xA5:
                end = self.buffer.find(0xA5)
                end = len(self.buffer) if end < 0 else end
            elif len(self.buffer) < 5:
                end = 0
            elif self.buffer[4] > MAX_PAYLOAD:
                end = 1
            else:
                end = 10 + self.buffer[4]
                if end > len(self.buffer):
                    end = 0
            if not end:
                if now - self.since < STALE:
                    break
                end = len(self.buffer)  # the rest never came
            frames.append(bytes(self.buffer[:end]))
            del self.buffer[:end]
            self.since = now
        return frames


def take_lines(buffer):
    """Whole lines from the remote program; an over-long piece without '\\n' goes as it is."""
    lines = []
    while True:
        end = buffer.find(b'\n')
        if end < 0:
            break
        lines.append(bytes(buffer[:end + 1]))
        del buffer[:end + 1]
    if len(buffer) > MAX_LINE:
        lines.append(bytes(buffer))
        buffer.clear()
    return lines


class Mux:
    def __init__(self, side, upstream_api='http://127.0.0.1:8767', host='127.0.0.1', api_port=8787,
                 data_port=None, remote_port=8789, remote_window=3.0, log=print):
        self.side, self.marker = side, MARKERS[side]
        self.upstream_api = upstream_api.rstrip('/')
        self.host = host
        self.remote_window = remote_window
        self.log = log
        self.mode = 'raw'
        self.requests = queue.Queue()
        self.stop = threading.Event()
        self.http = ThreadingHTTPServer((host, api_port), make_handler(self))
        try:
            self.ground_listener = listen(host, api_port + 1 if data_port is None else data_port)
            self.remote_listener = listen(host, remote_port)
        except BaseException:
            self.http.server_close()
            raise
        self.api_port = self.http.server_address[1]
        self.data_port = self.ground_listener.getsockname()[1]
        self.remote_port = self.remote_listener.getsockname()[1]
        self.up = self.ground = self.remote = None
        self.up_out, self.ground_out, self.remote_out = bytearray(), bytearray(), bytearray()
        self.splitter, self.cutter, self.remote_in = Splitter(self.marker), FrameCutter(), bytearray()
        self.held = deque()  # ground frames waiting for the remote's turn: (frame, queued at)
        self.kcp_text, self.kcp_splitter = bytearray(), Splitter(self.marker)
        self.remote_heard = self.remote_sent = -1e9
        self.next_connect = self.dropped_at = self.ground_at = 0.0
        self.restore_raw = False
        self.upstream_note = None
        self.last_abort = None
        self.counts = dict(remote_lines_in=0, remote_lines_out=0, ground_bytes_in=0, ground_frames_out=0,
                           transfers_refused=0, transfers_aborted=0)

    # ---- HTTP thread ----

    def api(self, method, path, body=None):
        """(HTTP status, JSON) from the teammate's bridge API."""
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.upstream_api + path, data=data, method=method,
                                         headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            with error:
                return error.code, json.load(error)

    def status(self):
        try:
            info = self.api('GET', '/status')[1]
        except (OSError, ValueError) as exc:
            info = {'state': 'waiting', 'worker_alive': False, 'uart_open': False,
                    'last_error': f'LoRa bridge not reachable: {exc}'}
        info['upstream'] = dict(api=self.upstream_api, mode=info.get('mode'), data=info.get('data'))
        info.update(mode=self.mode, data=f'tcp://{self.host}:{self.data_port}')
        info['mux'] = dict(side=self.side, remote=f'tcp://{self.host}:{self.remote_port}',
                           upstream_connected=self.up is not None, ground_client=self.ground is not None,
                           remote_client=self.remote is not None,
                           remote_active=self.remote_active(time.monotonic()), last_abort=self.last_abort,
                           **self.counts)
        return info

    def request_mode(self, mode):
        """Runs on the HTTP thread; the loop thread owns the sockets and does the change."""
        reply = queue.Queue(maxsize=1)
        self.requests.put((mode, reply))
        try:
            return reply.get(timeout=15)
        except queue.Empty:
            return False, 'mux did not answer'

    # ---- loop thread ----

    def remote_active(self, now):
        return now - self.remote_heard < self.remote_window

    def run(self):
        threading.Thread(target=self.http.serve_forever, args=(0.05,), name='mux-api', daemon=True).start()
        self.log(f'分流程式（{self.side}）：API http://{self.host}:{self.api_port}、'
                 f'地面站資料 tcp://{self.host}:{self.data_port}、遙控 tcp://{self.host}:{self.remote_port}；'
                 f'組員的 LoRa 程式 {self.upstream_api}')
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                self.handle_requests(now)
                if self.up is None and now >= self.next_connect:
                    self.connect_upstream(now)
                self.poll(now)
        finally:
            self.http.shutdown()
            self.http.server_close()
            for sock in (self.up, self.ground, self.remote, self.ground_listener, self.remote_listener):
                if sock is not None:
                    sock.close()

    def handle_requests(self, now):
        while True:
            try:
                mode, reply = self.requests.get_nowait()
            except queue.Empty:
                return
            reply.put(self.change_mode(mode, now))

    def change_mode(self, mode, now):
        if mode == 'kcp' and self.mode == 'raw' and self.remote_active(now):
            self.counts['transfers_refused'] += 1
            self.log('遙控優先：遙控開著，拒絕傳檔（遙控關掉 3 秒後再傳）')
            return False, 'remote control has priority: remote is active, file transfer refused'
        try:
            code, info = self.api('POST', '/mode', {'mode': mode})
        except (OSError, ValueError) as exc:
            return False, f'LoRa bridge not reachable: {exc}'
        if not info.get('ok'):
            return False, info.get('message') or f'LoRa bridge refused ({code})'
        if mode != self.mode:
            self.log('切到傳檔模式（KCP）' if mode == 'kcp' else '切回一般模式')
            self.mode = mode
            self.drop_upstream(now)
        return True, info.get('message', 'ok')

    def abort_transfer(self, now, why):
        self.counts['transfers_aborted'] += 1
        self.last_abort = time.strftime('%H:%M:%S')
        self.log(f'遙控優先：{why}，中斷傳檔、切回一般模式')
        self.mode = 'raw'
        self.restore_raw = True
        self.drop_upstream(now)

    def note_upstream(self, text):
        if text != self.upstream_note:
            self.upstream_note = text
            self.log(text)

    def connect_upstream(self, now):
        # Right after a mode switch the bridge restarts its data port: retry quickly for a while.
        self.next_connect = now + (0.1 if now - self.dropped_at < 3.0 else 1.0)
        try:
            if self.restore_raw:
                code, info = self.api('POST', '/mode', {'mode': 'raw'})
                if not info.get('ok'):
                    raise ConnectionError(f'切不回一般模式：{info.get("message") or code}')
                self.restore_raw = False
            info = self.api('GET', '/status')[1]
            if info.get('mode') != self.mode:
                raise ConnectionError(f'它在 {info.get("mode")} 模式，等它回到 {self.mode}')
            url = urlparse(info.get('data', ''))
            if url.scheme != 'tcp' or not url.hostname or not url.port:
                raise ConnectionError(f'不支援的資料位址 {info.get("data")}')
            sock = socket.create_connection((url.hostname, url.port), timeout=1)
        except (OSError, ValueError) as exc:
            self.note_upstream(f'等組員的 LoRa 程式（{self.upstream_api}）：{exc}')
            return
        sock.setblocking(False)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.up = sock
        self.note_upstream(f'已連上組員的 LoRa 程式（{"傳檔模式" if self.mode == "kcp" else "一般模式"}）')

    def drop_upstream(self, now):
        """Mode change or lost connection. Like the bridge, a mode change also drops the ground program."""
        if self.up is not None:
            self.up.close()
            self.up = None
        self.next_connect = self.dropped_at = now
        self.up_out.clear()
        self.held.clear()
        self.splitter, self.cutter = Splitter(self.marker), FrameCutter()
        self.kcp_text.clear()
        self.kcp_splitter = Splitter(self.marker)
        self.close_ground()

    def close_ground(self):
        if self.ground is not None:
            self.ground.close()
            self.ground = None
        self.ground_out.clear()
        self.cutter = FrameCutter()

    def poll(self, now):
        # pyserial's socket:// open() throws away what already arrived: give a new transfer program
        # a moment before its buffered KCP hello goes out.
        ground = None if self.mode == 'kcp' and now - self.ground_at < HELLO_DELAY else self.ground
        readers = [s for s in (self.ground_listener, self.remote_listener, self.up, self.ground, self.remote) if s]
        writers = [s for s, out in ((self.up, self.up_out), (ground, self.ground_out),
                                    (self.remote, self.remote_out)) if s is not None and out]
        readable, writable, _ = select.select(readers, writers, [], 0.01)
        if self.up is not None and self.up in readable:
            self.from_upstream(now)
        if self.ground is not None and self.ground in readable:
            self.from_ground(now)
        if self.remote is not None and self.remote in readable:
            self.from_remote(now)
        if self.ground_listener in readable:
            old, self.ground = self.ground, self.accept(self.ground_listener, self.ground, '地面站程式')
            if self.ground is not old:
                self.ground_at = now
        if self.remote_listener in readable:
            old, self.remote = self.remote, self.accept(self.remote_listener, self.remote, '遙控程式')
            if self.remote is not old:
                self.remote_out.clear()
        if self.mode == 'raw':
            self.ground_tick(now)
        for sock, out in ((self.up, self.up_out), (self.ground, self.ground_out), (self.remote, self.remote_out)):
            if sock is not None and sock in writable and out and not send(sock, out):
                self.lost(sock, now)

    def accept(self, listener, current, name):
        try:
            sock, _ = listener.accept()
        except OSError:
            return current
        if current is not None:
            sock.close()
            self.log(f'{name}：已經有一個連著，拒絕新的連線')
            return current
        sock.setblocking(False)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.log(f'{name}：已連上')
        return sock

    def lost(self, sock, now):
        if sock is self.up:
            self.note_upstream('組員的 LoRa 程式斷線（模式切換、USB 拔掉，或有別的程式先連了它的資料埠？）')
            if self.mode == 'kcp':
                self.drop_upstream(now)  # the transfer is broken
            else:
                self.up.close()
                self.up = None
                self.dropped_at = now
                self.up_out.clear()
                self.held.clear()
                self.splitter = Splitter(self.marker)
        elif sock is self.ground:
            self.log('地面站程式：斷線')
            self.close_ground()
            if self.mode == 'kcp' and self.up is not None:
                self.up.close()  # the next transfer program gets a fresh hello
                self.up = None
                self.next_connect = self.dropped_at = now
                self.up_out.clear()
                self.kcp_text.clear()
        elif sock is self.remote:
            self.log('遙控程式：斷線')
            self.remote.close()
            self.remote = None
            self.remote_out.clear()
            self.remote_in.clear()

    def from_upstream(self, now):
        data = receive(self.up)
        if data is None:
            return self.lost(self.up, now)
        if self.mode == 'kcp':
            if len(self.ground_out) < BUFFER_LIMIT:
                self.ground_out += data  # kept until the transfer program connects (its hello first)
            return self.watch_transfer(data, now)
        lines, ground = self.splitter.feed(data, now)
        for line in lines:
            self.remote_heard = now
            self.counts['remote_lines_in'] += 1
            if self.remote is not None and len(self.remote_out) < BUFFER_LIMIT:
                self.remote_out += line
        self.to_ground(ground)

    def watch_transfer(self, data, now):
        """KCP JSON lines from the bridge: a remote line inside the radio data stops the transfer."""
        self.kcp_text += data
        while True:
            end = self.kcp_text.find(b'\n')
            if end < 0:
                break
            line = bytes(self.kcp_text[:end])
            del self.kcp_text[:end + 1]
            try:
                message = json.loads(line)
                radio = base64.b64decode(message['data']) if message.get('type') == 'rx' else b''
            except (ValueError, KeyError, TypeError, AttributeError):
                continue
            if self.kcp_splitter.feed(radio, now)[0]:
                self.remote_heard = now
                return self.abort_transfer(now, '傳檔中收到遙控的資料')
        if len(self.kcp_text) > BUFFER_LIMIT:
            self.kcp_text.clear()

    def to_ground(self, data):
        if data and self.ground is not None and len(self.ground_out) < BUFFER_LIMIT:
            self.counts['ground_bytes_in'] += len(data)
            self.ground_out += data

    def from_ground(self, now):
        data = receive(self.ground)
        if data is None:
            return self.lost(self.ground, now)
        if self.mode == 'kcp':
            if len(self.up_out) < BUFFER_LIMIT:
                self.up_out += data  # kept while the bridge's data port restarts after the switch
            return
        self.queue_ground(self.cutter.feed(data, now), now)

    def queue_ground(self, frames, now):
        for frame in frames:
            if self.up is None:
                continue  # no radio: dropped, as the bridge does
            self.counts['ground_frames_out'] += 1
            if self.held or now - self.remote_sent < REMOTE_IDLE:
                self.held.append((frame, now))
            else:
                self.up_out += frame

    def release_held(self):
        while self.held:
            self.up_out += self.held.popleft()[0]

    def ground_tick(self, now):
        self.to_ground(self.splitter.flush(now))
        self.queue_ground(self.cutter.take(now), now)
        if self.held and (now - self.remote_sent >= REMOTE_IDLE or now - self.held[0][1] >= GROUND_HOLD):
            self.release_held()

    def from_remote(self, now):
        data = receive(self.remote)
        if data is None:
            return self.lost(self.remote, now)
        self.remote_in += data
        for line in take_lines(self.remote_in):
            self.remote_heard = now
            if self.mode == 'kcp':
                return self.abort_transfer(now, '傳檔中遙控要送資料')
            if self.up is None:
                continue
            self.counts['remote_lines_out'] += 1
            self.up_out += line
            self.remote_sent = now
            self.release_held()  # ground frames go right behind the remote line


def listen(host, port):
    sock = socket.create_server((host, port), backlog=1)
    sock.setblocking(False)
    return sock


def receive(sock):
    """Bytes, b'' when nothing is ready, None when the peer closed."""
    try:
        return sock.recv(4096) or None
    except BlockingIOError:
        return b''
    except OSError:
        return None


def send(sock, out):
    try:
        count = sock.send(out)
    except BlockingIOError:
        return True
    except OSError:
        return False
    del out[:count]
    return True


def make_handler(mux):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, code, obj):
            data = json.dumps(obj).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/status':
                self.send_json(200, mux.status())
            else:
                self.send_json(404, {'error': 'GET /status; POST /mode'})

        def do_POST(self):
            if self.path == '/reset':
                try:
                    return self.send_json(*mux.api('POST', '/reset'))
                except (OSError, ValueError) as exc:
                    return self.send_json(502, {'ok': False, 'message': f'LoRa bridge not reachable: {exc}'})
            if self.path != '/mode':
                return self.send_json(404, {'error': 'not found'})
            try:
                n = int(self.headers.get('Content-Length', '-1'))
                if not 0 <= n <= 4096:
                    raise ValueError('invalid content length')
                mode = json.loads(self.rfile.read(n)).get('mode')
                if mode not in ('raw', 'kcp'):
                    raise ValueError('mode must be raw or kcp')
            except (ValueError, UnicodeDecodeError, AttributeError) as exc:
                return self.send_json(400, {'error': str(exc)})
            ok, message = mux.request_mode(mode)
            self.send_json(200 if ok else 409, {**mux.status(), 'ok': ok, 'message': message})

        def log_message(self, *args):
            pass  # /status is polled; keep the terminal for events

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description='分流程式：遙控和人機介面共用一顆 LoRa（遙控優先）')
    parser.add_argument('side', choices=sorted(MARKERS), help='orin：裝在車上；laptop：裝在筆電')
    parser.add_argument('--upstream-api', default='http://127.0.0.1:8767', help='組員 LoRa 程式的 API')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--api-port', type=int, default=8787, help='給地面站程式的 API；資料埠是它 +1')
    parser.add_argument('--remote-port', type=int, default=8789, help='給遙控程式的資料埠')
    parser.add_argument('--remote-window', type=float, default=3.0, help='最近幾秒有遙控資料就不准傳檔')
    args = parser.parse_args(argv)
    try:
        mux = Mux(args.side, args.upstream_api, args.host, args.api_port, remote_port=args.remote_port,
                  remote_window=args.remote_window, log=lambda text: print(text, flush=True))
    except OSError as exc:
        parser.exit(1, f'✗ 開不了 port（是不是已經有一個分流程式在跑？）：{exc}\n')
    try:
        mux.run()
    except KeyboardInterrupt:
        print('分流程式結束', flush=True)


if __name__ == '__main__':
    main()
