"""Test panel for the laptop handheld: the same loop as handheld_pc.py, shown in a window.

  python handheld_gui.py --port COM7          F710 on this laptop (switch on the back set to X)
  python handheld_gui.py --port COM7 --demo   scripted presses, no F710 needed

Shows what the handheld knows: the F710 inputs, what was sent, the LoRa link, and the Orin
mode and STM32 state carried by each reply. Track RPM only appears on the Orin screen.
"""

import argparse
import ctypes
import queue
import sys
import threading
import time
import tkinter as tk

from handheld_lib import SLOTS, STATE_NAMES, tools
from handheld_pc import F710, SAMPLE_PERIOD, DemoGamepad, Session

GREEN, YELLOW, RED, GRAY, BLUE = '#2e7d32', '#f9a825', '#c62828', '#9e9e9e', '#1565c0'
LIT, UNLIT, BAR = '#ffb300', '#e0e0e0', '#1565c0'
FONT = 'Microsoft JhengHei UI'
REFRESH_MS = 50
LOG_LINES = 500
BAR_WIDTH, BAR_HEIGHT = 420, 26

MODE_TILE = {0: ('ORIN', BLUE), 1: ('JOY', GREEN), 2: ('等待 AUTO', YELLOW), 3: ('異常', RED)}
STATE_COLOR = {1: YELLOW, 2: YELLOW, 3: GREEN}  # STOPPED and MANUAL wait for the R12; others are faults
BUTTON_TEXT = {'十字鍵上': '↑', '十字鍵下': '↓'}


class Panel:
    """The radio loop runs on its own thread, so a dragged or busy window never delays a
    packet (the Orin stops the tracks after 0.3 s without one). The window only reads."""

    def __init__(self, root, gamepad, port, title):
        self.root = root
        self.lines = queue.Queue()
        self.session = Session(gamepad, port, self.lines.put)
        self.failed = False
        self.stopping = threading.Event()
        self.worker = threading.Thread(target=self.loop, daemon=True)
        root.title(title)
        root.configure(bg='white', padx=12, pady=12)
        root.minsize(760, 560)

        tiles = tk.Frame(root, bg='white')
        tiles.pack(fill='x')
        self.tiles = {key: self.tile(tiles, caption) for key, caption in
                      (('lora', 'LoRa 連線'), ('mode', 'Orin 模式'), ('stm32', 'STM32'), ('f710', 'F710'))}

        sticks = tk.Frame(root, bg='white', pady=12)
        sticks.pack(fill='x')
        self.bars = {key: self.bar(sticks, row, caption) for row, (key, caption) in
                     enumerate((('drive', '前後'), ('turn', '轉向')))}

        buttons = tk.Frame(root, bg='white')
        buttons.pack(fill='x')
        self.buttons, self.counts = [], []
        for column, (name, _) in enumerate(SLOTS):
            box = tk.Label(buttons, text=BUTTON_TEXT.get(name, name), width=6, relief='ridge', bd=2,
                           bg=UNLIT, font=(FONT, 11, 'bold'))
            box.grid(row=0, column=column, padx=2)
            count = tk.Label(buttons, text='0', bg='white', font=(FONT, 10))
            count.grid(row=1, column=column)
            self.buttons.append(box)
            self.counts.append(count)
        tk.Label(buttons, text='（亮起＝按著；數字＝按了幾次）', bg='white', fg='#616161',
                 font=(FONT, 9)).grid(row=2, column=0, columnspan=len(SLOTS), sticky='w')

        self.stats = tk.Label(root, bg='white', anchor='w', font=(FONT, 11), pady=8)
        self.stats.pack(fill='x')

        logs = tk.Frame(root, bg='white')
        logs.pack(fill='both', expand=True)
        self.log = tk.Text(logs, height=12, font=('Consolas', 10), state='disabled', wrap='none')
        scroll = tk.Scrollbar(logs, command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.log.pack(side='left', fill='both', expand=True)

    @staticmethod
    def tile(parent, caption):
        frame = tk.Frame(parent, bg='white', padx=4)
        frame.pack(side='left', expand=True, fill='x')
        tk.Label(frame, text=caption, bg='white', fg='#616161', font=(FONT, 10)).pack(anchor='w')
        value = tk.Label(frame, text='—', bg=GRAY, fg='white', font=(FONT, 18, 'bold'), pady=10)
        value.pack(fill='x')
        return value

    @staticmethod
    def bar(parent, row, caption):
        tk.Label(parent, text=caption, bg='white', font=(FONT, 12)).grid(row=row, column=0, padx=(0, 8))
        canvas = tk.Canvas(parent, width=BAR_WIDTH, height=BAR_HEIGHT, bg=UNLIT, highlightthickness=1,
                           highlightbackground='#757575')
        canvas.grid(row=row, column=1, pady=3)
        middle = BAR_WIDTH // 2
        fill = canvas.create_rectangle(middle, 2, middle, BAR_HEIGHT, fill=BAR, width=0)
        canvas.create_line(middle, 0, middle, BAR_HEIGHT, fill='black', width=2)
        value = tk.Label(parent, text='+0', width=5, anchor='e', bg='white', font=('Consolas', 14, 'bold'))
        value.grid(row=row, column=2, padx=(8, 0))
        return canvas, fill, value

    def append(self, line):
        self.log.configure(state='normal')
        self.log.insert('end', line + '\n')
        excess = int(self.log.index('end-1c').split('.')[0]) - LOG_LINES
        if excess > 0:
            self.log.delete('1.0', f'{excess + 1}.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def set_tile(self, key, text, color):
        self.tiles[key].configure(text=text, bg=color)

    def refresh(self):
        s, info = self.session, self.session.info
        connected, drive, turn, buttons = s.reading

        if self.failed:
            self.set_tile('lora', '串口錯誤', RED)
        elif info.link_up:
            self.set_tile('lora', '正常', GREEN)
        else:
            self.set_tile('lora', '失聯' if info.replies else '尚未連上', RED if info.replies else GRAY)
        if info.link_up:
            self.set_tile('mode', *MODE_TILE.get(info.indicator, ('?', RED)))
            self.set_tile('stm32', STATE_NAMES.get(info.state, str(info.state)), STATE_COLOR.get(info.state, RED))
        else:
            self.set_tile('mode', '—', GRAY)
            self.set_tile('stm32', '—', GRAY)
        self.set_tile('f710', '已連接' if connected else '未連接', GREEN if connected else RED)

        for key, value in (('drive', drive), ('turn', turn)):
            canvas, fill, text = self.bars[key]
            middle = BAR_WIDTH // 2
            end = middle + value * (middle - 2) // 100
            canvas.coords(fill, min(middle, end), 2, max(middle, end), BAR_HEIGHT)
            text.configure(text=f'{value:+d}')

        for slot, (box, count) in enumerate(zip(self.buttons, self.counts)):
            box.configure(bg=LIT if buttons & (1 << slot) else UNLIT)
            count.configure(text=str(s.presses[slot]))

        rate = f'{info.replies / s.sent:.0%}' if s.sent else '—'
        age = '—' if s.last_reply_at is None else f'{(time.monotonic() - s.last_reply_at) * 1000:.0f} ms 前'
        self.stats.configure(text=f'已送出 {s.sent} 筆｜收到回覆 {info.replies} 筆（{rate}）｜'
                                  f'最後回覆 {age}｜震動 {"開" if s.rumbling else "關"}')

    def loop(self):
        while not self.stopping.is_set():
            began = time.monotonic()
            try:
                self.session.step()
            except Exception as error:  # e.g. the LoRa adapter was unplugged
                self.failed = True
                self.lines.put(f'串口錯誤，已停止：{error}（重新插好後請重開程式）')
                return
            time.sleep(max(0.0, SAMPLE_PERIOD - (time.monotonic() - began)))

    def tick(self):
        while True:
            try:
                self.append(self.lines.get_nowait())
            except queue.Empty:
                break
        self.refresh()
        self.root.after(REFRESH_MS, self.tick)

    def start(self):
        self.worker.start()
        self.tick()

    def close(self):
        self.stopping.set()
        self.worker.join(timeout=1.0)
        self.session.close()


def main():
    parser = argparse.ArgumentParser(description='遙控器測試面板：筆電假裝手把 MCU，用視窗顯示狀態')
    parser.add_argument('--port', required=True, help='手把那顆 LoRa 模組的 COM，例如 COM7')
    parser.add_argument('--demo', action='store_true', help='不接 F710，自動跑一段示範操作')
    parser.add_argument('--pad', type=int, default=0, help='XInput 手把編號 0～3')
    args = parser.parse_args()
    if sys.platform != 'win32':
        sys.exit('這支程式只支援 Windows（用 XInput 讀 F710）。')
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)  # sharp text on scaled displays
    except (AttributeError, OSError):
        pass

    port = tools.open_serial(args.port)
    gamepad = DemoGamepad() if args.demo else F710(args.pad)
    root = tk.Tk()
    panel = Panel(root, gamepad, port, f'遙控器測試面板 － {args.port}' + ('（示範）' if args.demo else ''))
    panel.append(f'手把端：{args.port}，每 0.1 秒送一筆。關掉視窗就結束。')
    if not args.demo and not gamepad.read()[0]:
        panel.append('⚠ 找不到 F710：背面開關要切到 X，並插上接收器。')

    def close():
        panel.close()
        port.close()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)
    panel.start()
    root.mainloop()


if __name__ == '__main__':
    main()
