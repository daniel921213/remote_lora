# 遙控器 → LoRa → Orin → STM32（remote_orin）

F710 手把經過 LoRa 送到 Orin，由 Orin 決定要聽遙控器（JOY）還是自動導航（ORIN），再把左右轉速送給 STM32。

```text
F710 ─▶ 手把 MCU ─▶ RYLR998 ～LoRa～ RYLR998 ─▶ Orin ─▶ STM32（一直在 AUTO）
        現在：筆電                              orin_bridge.py   組員的 mower_link.py
        之後：Feather RP2040
```

- 封包沿用 [RC_LORA_PROTOCOL.md](../RC_LORA_PROTOCOL.md) 的 v0.3，2026-10-22 已經用兩顆真的 LoRa 模組測通。
- 兩顆 RYLR998 都要設成**透傳模式、UART 115200**，跟 10/22 測試時一樣。
- 需要 Python 3.8 以上、`pyserial`，以及 gcc（Windows 用 MinGW-w64；C 程式會在第一次執行時自動編譯）。
- 下面的指令都從 repo 的根目錄開始。

## 資料夾

| 資料夾 | 跑在哪裡 | 內容 |
|---|---|---|
| `common/` | 共用 | `rc_proto`：封包格式；`joy_api`：接收端（Orin 用）；`rc_handheld`：手把端邏輯（**筆電和 RP2040 跑同一份**）；`tools.py` |
| `handheld_pc/` | 筆電 | `handheld_pc.py`：筆電假裝手把 MCU，F710 → LoRa；`handheld_gui.py`：同樣功能的視窗版測試面板 |
| `handheld_rp2040/` | Feather RP2040 USB Host | Arduino 程式；`src/` 是 `common/` 的複本 |
| `orin/` | Orin | `orin_bridge.py`：收 LoRa → 判斷 JOY／ORIN → 送轉速給 STM32；`fake_stm32.py`：沒有 STM32 時用的模擬 |
| `tests/` | 電腦 | `run_tests.py`：全部離線測試 |

## 第一次上機的按鍵

| F710 | Orin 做什麼 |
|---|---|
| 左搖桿前後、右搖桿左右 | JOY 模式：換算成左右轉速，推到底 = `--max-rpm`（預設 5 RPM） |
| RB | JOY ⇄ ORIN 切換。ORIN 模式下履帶以 `--auto-rpm`（預設 5 RPM）往前，代替還沒有的導航程式。切回 JOY 時，搖桿要先回中間才會動 |
| B | 停車，回到 JOY |
| BACK | 熄火並鎖定停車（STM32 進 ESTOP，要用 R12 恢復） |
| LB | ARM：解開行走鎖，等同在 Orin 輸入 `ARM`（見 [ARM_USAGE.md](ARM_USAGE.md)）。要在 AUTO、JOY、搖桿置中時按 |
| START、X、Y、A、十字鍵 | 這次不支援，Orin 只會印出訊息 |

自動停車的情況：

- 遙控器失聯超過 0.3 秒，或 F710 斷線：停車並回到 JOY。
- STM32 離開 AUTO：停車並回到 JOY。
- `orin_bridge.py` 結束：送 ESTOP（要用 R12 恢復）。
- `orin_bridge.py` 當掉：STM32 自己在 0.2 秒後停車。

F710 的震動：等待 STM32 進 AUTO 震 2 下、JOY 震 1 下、ORIN 震 3 下、異常長震 1 秒、跟 Orin 失聯則每 3 秒長震一次。

## 測試一：離線測試（不需要任何硬體）

```bash
cd remote_orin
python tests/run_tests.py
```

最後一行印出「全部通過」就沒問題。這一步會跑：

- C 測試：協定 72 項、手把邏輯 49 項。
- Python 測試：Orin 判斷邏輯、假 STM32、端到端流程（手把 C 程式 → 模擬無線電 → Orin → 假 STM32）。
- 確認 RP2040 的 `src/` 和 `common/` 內容一致。

加 `--rp2040` 會再用 arduino-cli 編譯 RP2040 程式（要先安裝「Pico PIO USB」程式庫，見最後一節）。

## 測試二：筆電單機（兩顆 LoRa 都插筆電，不接車）

這是上機前的彩排：真的 LoRa、真的 F710，只有 STM32 是模擬的。

**視窗 A：扮演 Orin**

```bash
cd remote_orin/orin
python orin_bridge.py --lora-port COM8 --fake-stm32
```

**視窗 B：扮演手把**（F710 背面開關切到 **X**）

```bash
cd remote_orin/handheld_pc
python handheld_pc.py --port COM7
```

想用視窗看，改執行 `python handheld_gui.py --port COM7`，功能完全一樣。最上面四個色塊顯示 LoRa 連線、Orin 模式、STM32 狀態和 F710 是否連接：綠色是正常、黃色是等待、紅色是異常或失聯、藍色是 ORIN、灰色是還沒有資料。下面依序是搖桿位置、按鍵（按著時亮起，數字是按了幾次）、送出和回覆的筆數，以及事件紀錄。

操作順序：

1. F710 震 2 下：已經連上，Orin 在等 STM32 進 AUTO。
2. 在**視窗 A** 按 `a`，模擬按 R12 的 A。搖桿放開，等半秒後按 **LB**（ARM），F710 震 1 下，進入 JOY。
3. 推搖桿，視窗 A 的「命令」跟著變。
4. 按 RB：F710 震 3 下，進入 ORIN，命令變成固定的測試轉速。
5. 再按 RB 回到 JOY。按 B 停車、按 BACK 熄火（假 STM32 進 ESTOP，在視窗 A 按 `r` 恢復）。
6. 拔掉其中一顆 LoRa：0.3 秒內兩邊都顯示失聯，命令歸零，F710 長震。

不想拿手把時，可以讓兩邊都自動跑：視窗 A 加 `--fake-auto`（1 秒後自己進 AUTO），視窗 B 加 `--demo`（13 秒的自動操作）。

### 改到 Orin 上跑，STM32 還是用模擬的

這一步確認 Orin 上的環境沒問題，包括 Python、gcc、串口權限和 LoRa 的 USB 轉接頭。`--fake-stm32` 完全不會打開 STM32 的串口，所以不會動到車子。

在 Orin 的主機上執行就好，不用進 humble 容器：

```bash
sudo apt install python3-serial   # 已經有 pyserial 就跳過
cd remote_orin
python3 tests/run_tests.py        # 確認 gcc 和 Python 都正常，最後要印出「全部通過」
ls /dev/ttyUSB*                   # 插 LoRa 前後各執行一次，多出來的那個就是 LoRa
cd orin
python3 orin_bridge.py --lora-port /dev/ttyUSB0 --fake-stm32 --fake-auto
```

`--fake-auto` 會讓模擬的 STM32 在 1 秒後自己進 AUTO，Orin 那邊不需要有人按鍵。筆電照樣執行 `python handheld_pc.py --port COM7`，再照上面的表格檢查一次。

## 測試三：上機

**接線**

- 筆電：F710 接收器（開關切 **X**）、手把那顆 LoRa（COM7）。
- Orin：車上那顆 LoRa，用 USB 轉 UART 接，例如 `/dev/ttyUSB0`；STM32 用 USB 接 ST-LINK 那個接頭，例如 `/dev/ttyACM0`。
- R12 要開著：目前的韌體要靠 R12 才能進 AUTO，沒有 R12 訊號會判定故障。

**Orin 上的準備（第一次）**

1. 把整個 `remote_orin` 資料夾複製到 Orin。
2. 安裝套件：`pip3 install pyserial`；如果沒有 gcc，執行 `sudo apt install build-essential`。第一次執行會自動編譯 `orin/build/joy.so`。
3. 跟組員拿**最新的** `mower_link.py`。程式會先找 `/home/user/MowerL476_project/MowerL476/jetson/`（組員文件寫的路徑），找不到就找 `orin/` 資料夾；也可以用 `--mower-link-dir` 指定。
4. 串口權限：把使用者加入 `dialout` 群組，然後重新登入。

**先在台架上測：履帶離地，刀盤沒有動力，旁邊有人能直接斷電**

1. 先跑組員的 monitor，確認 STM32 有回應而且 `faults=0`，然後**把 monitor 關掉**：同一個串口只能有一個程式開著。
2. Orin 執行：

   ```bash
   cd remote_orin/orin
   python3 orin_bridge.py --lora-port /dev/ttyUSB0 --stm32-port /dev/ttyACM0
   ```

3. 筆電執行 `python handheld_gui.py --port COM7`（或文字版 `handheld_pc.py`），F710 震 2 下。
4. R12 搖桿回中間、液壓開關在保持、熄火開關在允許位置，等 1 秒後按 **A**。F710 搖桿放開，按 **LB**（ARM），Orin 顯示「ARM 成功」、F710 震 1 下，進入 JOY。
5. 輕推搖桿，履帶跟著轉。
6. 按 **RB** 切到 ORIN，履帶以測試轉速往前；再按 RB 回到 JOY。
7. 測停止：按 B 停車；拔掉筆電那顆 LoRa，0.3 秒內停；在 Orin 按 `q` 結束程式，會送 ESTOP。

台架測試都正常之後，才放到地上用低速測。

## Feather RP2040（之後）

- **板子**：Adafruit Feather RP2040 **USB Host**。F710 接收器插在板子的 USB-A 孔。
- **Arduino IDE 設定**：
  - Boards Manager 安裝 Raspberry Pi Pico/RP2040（Earle Philhower），2026-10-22 用 6.3.0 編譯過。
  - 板子選 Adafruit Feather RP2040 USB Host。
  - Tools → CPU Speed **120 MHz**、USB Stack **Adafruit TinyUSB**。
  - Library Manager 安裝 **Pico PIO USB**，問要不要一起裝相依程式庫時選全部安裝。
- **接線**：RYLR998 的 TXD 接 Feather 的 RX（GPIO1），RXD 接 TX（GPIO0），再接 3V3、GND。模組設成透傳、115200。
- **F710**：背面開關切到 **D**，MODE 燈要熄滅。
- **狀態**：2026-10-22 已經用 arduino-cli 編譯成功（核心 6.3.0、Pico PIO USB 0.7.2、Adafruit TinyUSB 3.7.7），**還沒在實板上跑過**。手把邏輯跟筆電是同一份，已經測過；要在實板上確認的是硬體這幾部分：
  - F710 在 D 模式的報告格式：先把程式裡的 `DEBUG_REPORTS` 設成 1，按每顆鍵確認輸出對不對。
  - 震動指令（照 Linux hid-lg2ff 驅動寫的）。
  - F710 關機時，接收器回報什麼。
- **改過 `common/` 的 C 檔之後**，執行 `python tests/run_tests.py --sync`，把它們複製到 `handheld_rp2040/src/`。

## 已知限制

- 組員的韌體已經比我們手上 9/29 的版本新。上機前要確認 `mower_link.py` 和韌體是最新的。
- STM32 進 AUTO 還是要靠 R12；Orin 不能解鎖，也不能啟動引擎。
- RB 目前是單純切換。規格書寫的「釋放後等地面站接手」之後再做。
- 封包還是 v0.3 的文字格式，單程約 40 ms。改成二進位、加停止位元，等第一次上機測完再做。
- 預估從按下停止到 PWM 回中立：LoRa 約 40 ms＋等下一包最多 100 ms＋Orin 約 10 ms＋STM32 最多 30 ms。
