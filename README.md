# 遙控器 LoRa 鏈路（rc_lora）

F710 手把 → 手把端 MCU → RYLR998 ～ LoRa ～ RYLR998 → STM32。協定定義見 [RC_LORA_PROTOCOL.md](RC_LORA_PROTOCOL.md)。

**接收端改放在 Orin 的版本見 [remote_orin/](remote_orin/README.md)**：F710 → LoRa → Orin → STM32，包含筆電手把程式、Feather RP2040 程式、Orin 程式和上機測試步驟。

## 檔案

| 檔案 | 用在哪裡 | 內容 |
|---|---|---|
| `src/rc_proto.h/.c` | 手把端 MCU **和** STM32 共用 | 封包打包／解析、CRC、十六進位、`AT+SEND` 產生、`+RCV` 解析 |
| `src/joy_api.h/.c` | STM32 | 收 LoRa 資料、判斷失聯、按鍵計數轉成事件、回傳狀態（手把用來震動提示）|
| `tests/test_joy_api.c` | 電腦 | 離線測試 |
| `pc_test/rc_pc_test.py`、`joy_sim.c` | 電腦 | 用真的 F710 測試整條路，不需要 LoRa 和 MCU（見下方）|
| `pc_test/lora_binary_test.py` | 電腦＋兩顆 LoRa | 確認透傳模式能不能原封不動傳二進位資料，並量延遲 |
| `remote_orin/` | 筆電、Orin、Feather RP2040 | 接收端放在 Orin 的完整版本，見 [remote_orin/README.md](remote_orin/README.md) |
| `stm32_test/` | 測試用的 NUCLEO-L476RG | 不接馬達的獨立測試專案，收到什麼就印出來；步驟見 [stm32_test/README.md](stm32_test/README.md) |

只用標準 C99，不依賴 HAL 或 FreeRTOS。名稱都加 `rc_`／`joy_` 前綴，不會和現有韌體的 `wire_u16` 等函式衝突。

## 在電腦上跑測試

```bash
cd D:/car/rc_lora
mkdir -p build
gcc -std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -Isrc \
  src/rc_proto.c src/joy_api.c tests/test_joy_api.c -o build/test_joy_api.exe -lm
./build/test_joy_api.exe
```

最後一行印出 `0 failed` 即通過。

## 用 F710 在電腦上測試（不需要 LoRa）

```text
F710 ─USB─▶ 電腦：Python 扮演手把端 MCU ─▶ 模擬 LoRa ─▶ 真正的 joy_api（扮演 STM32）
              ▲                                               │
              └──────────── F710 震動 ◀── 回覆 ◀──────────────┘
```

只模擬 LoRa 無線傳輸和 STM32 主控，其他都是真的：F710 按鍵、封包格式、`joy_api` 的 C 程式、震動回饋。

**需要**：Windows、Python 3、F710 和它的 USB 接收器。gcc 用來編譯 `joy_sim.dll`，資料夾裡已經有編好的，沒有 gcc 也能跑。

**步驟**

1. F710 頂端的開關切到 **X**。電腦用 XInput 讀手把，XInput 只支援 X 模式。之後接 MCU 時要改回 **D**。
2. 接收器插到電腦。
3. 執行：

   ```bash
   cd D:/car/rc_lora/pc_test
   python rc_pc_test.py
   ```

4. 按手把上的鍵、推搖桿，畫面會顯示：
   - `手把送出`：手把端送出的 `AT+SEND` 指令
   - `車子事件`、`車子收到`：STM32 的 `joy_api` 解讀出來的結果
   - `手把收到 … → 震動`：F710 會真的震動

**鍵盤操作**

| 鍵 | 作用 |
|---|---|
| `o` | 模擬 Orin 接手（要先按 RB 進入釋放等待）|
| `f` | 模擬故障 |
| `l` | 切斷／恢復 LoRa，測試失聯 |
| `q` | 結束 |

**三種接法**

| 指令 | 無線 | 車子 |
|---|---|---|
| `python rc_pc_test.py` | 模擬 | 電腦上的 `joy_api` |
| `python rc_pc_test.py --port COM7 --car-port COM8` | **真的 LoRa**（兩顆都插在電腦上）| 電腦上的 `joy_api`，透過 COM8 收發 |
| `python rc_pc_test.py --port COM7` | 真的 LoRa，或用線直接接 | **真的 STM32** |

- `--port` 模式送出的是透傳格式 `+RCV=1,32,…,0,0`（見 [RC_LORA_PROTOCOL.md](RC_LORA_PROTOCOL.md) 第 2.4 節），模組要設成透傳、UART 115200。需要 `pip install pyserial`。
- COM 號碼依實際電腦而定，到裝置管理員的「連接埠」查看。
- 開著終端機（Tera Term 等）的 COM 不能同時給這支程式用，要先關掉。

**其他選項**

| 選項 | 作用 |
|---|---|
| `--demo` | 不接手把，自動跑一段示範。可以和上面任一種接法一起用 |
| `--loss 0.3` | 模擬 30% 掉包，看按鍵會不會漏（只能用在模擬模式）|

2026-10-22 實測：`--demo --port COM7 --car-port COM8` 透過兩顆真實的透傳 LoRa 模組，所有按鍵事件、搖桿數值、狀態回覆與震動觸發都正確。

模擬主控（`FakeMainControl`）的規則只是為了讓震動能測試：RB 在 JOY 和釋放等待之間切換、失聯就進入異常、LB 解除異常。**真正的規則要等規格書 3.1 節定案。**

## STM32 主控怎麼串接

把 `src/` 的四個檔案加進 STM32 專案。主控在**同一個 task** 內依序呼叫：

```c
#include "joy_api.h"

static JoyApi joy;
static const JoyConfig joy_config = {
    .peer_address = 1,        /* 手把端 RYLR998 的 ADDRESS */
    .link_timeout_ms = 300,
    .deadband = 8,            /* 搖桿死區 8% */
    .drive_sign = 1,          /* 實測方向相反時改成 -1 */
    .turn_sign = 1,
};

void control_init(void) {
    if (!joy_api_init(&joy, &joy_config)) {
        /* 設定錯誤：joy_api 永遠回報失聯，主控應停在安全狀態 */
    }
}

void control_step(void) {                     /* 例如每 10 ms */
    uint32_t now = HAL_GetTick();

    /* 1. UART 中斷只把位元組放進 ring buffer，這裡取出交給 joy_api */
    uint8_t rx[64];
    size_t n = lora_ring_read(rx, sizeof rx); /* 自行實作 */
    if (lora_ring_overflowed()) joy_api_report_overflow(&joy);
    joy_api_feed(&joy, rx, n, now);

    /* 2. 有收到合法封包就回一筆狀態 */
    char reply[JOY_REPLY_MAX];
    size_t m = joy_api_take_reply(&joy, reply, sizeof reply);
    if (m) lora_uart_write(reply, m);         /* 自行實作 */

    /* 3. 取得遙控器輸出 */
    JoyOutput rc;
    joy_api_poll(&joy, now, &rc);

    /* 4. 主控依規格書 3.1 節處理，例如：
     *    rc.link_ok         遙控鏈路是否正常
     *    rc.drive, rc.turn  -1～+1，JOY 模式下的行走輸入
     *    rc.sticks_neutral  兩支搖桿是否在中立
     *    rc.events          JOY_EV_*，每次按壓只出現一次
     */
    if (rc.events & JOY_EV_PARK) { /* 停車＋空檔 */ }
    if (rc.events & JOY_EV_CONTROL_TOGGLE) { /* JOY／ORIN 切換 */ }
    if (rc.events & JOY_EV_LINK_LOST) { /* 遙控失聯 */ }

    /* 5. 告訴 joy_api 目前的狀態指示，下一筆回覆會帶過去；值改變時手把會震動 */
    joy_api_set_status(&joy, indicator_for_current_state(), current_state());
}
```

| 事件 | 按鍵 | 規格書功能 |
|---|---|---|
| `JOY_EV_ENGINE_ON` | START | 引擎通電 |
| `JOY_EV_ENGINE_OFF` | BACK | 引擎斷電 |
| `JOY_EV_GEAR_BLADE` | X | 刀具檔位 |
| `JOY_EV_GEAR_FAN` | Y | 風機檔位 |
| `JOY_EV_GEAR_NEUTRAL` | A | 離合空檔 |
| `JOY_EV_PARK` | B | 停車＋空檔 |
| `JOY_EV_CONTROL_TOGGLE` | RB | JOY／ORIN 切換請求 |
| `JOY_EV_UNLOCK` | LB | 解除停止鎖定 |
| `JOY_EV_BLADE_UP` | 十字鍵上 | 刀頭上升 |
| `JOY_EV_BLADE_DOWN` | 十字鍵下 | 刀頭下降 |
| `JOY_EV_LINK_LOST` | — | 鏈路從正常變成失聯 |
| `JOY_EV_LINK_RESTORED` | — | 鏈路恢復，包含開機後第一次連上 |
| `JOY_EV_OVERFLOW` | — | 可能漏了資料：UART 緩衝區滿、單行過長，或主控來不及取走事件 |

### `joy_api` 已經處理的事

- CRC、格式、位址錯誤的封包直接丟棄。
- 重複或比較舊的序號丟棄，不會重複觸發，也不會延長連線時效。
- 300 ms 沒有合法封包，或手把回報 F710 斷線，`link_ok` 變成 false，搖桿輸出歸零。
- **連線恢復後，搖桿要先回中立一次才會輸出**，避免恢復的瞬間車子衝出去。
- 恢復後的第一筆封包只當基準，斷線前的按壓不會補觸發。
- 十字鍵上、下同時有新按壓時，兩個都不觸發。

### 留給主控決定的事

`joy_api` 只回報「發生了什麼」，**不決定車子要怎麼反應**。下面這些依規格書 3.1 節，由主控實作：

- 失聯時是否 ESTOP，在 JOY、ORIN 模式下是否不同。
- 哪些事件在哪個模式下有效。
- JOY／ORIN／PAUSE／ESTOP 的切換條件。
- 狀態指示（`JoyIndicator`：ORIN／JOY／釋放等待／異常）對應主控的哪個狀態。

## 硬體備註

- RYLR998 為 3.3 V 邏輯，可直接接 STM32 的 UART：115200、8N1、**不反相**。
- 如果 R12 的 SBUS 不再使用，原本的 USART1（PA9／PA10）可以改接 RYLR998。SBUS 用的是 100000 baud、8E2、反相，改接時要重新設定。
- 模組參數（頻率、網路 ID、密碼等）建議先用 USB 轉 UART 設定好，見 [RC_LORA_PROTOCOL.md](RC_LORA_PROTOCOL.md) 第 2.2 節。設定是否會保存在模組內，以 datasheet 為準。

## 驗證狀態

2026-10-21：Windows／gcc 16.2，以 `-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror` 編譯，72 項檢查全部通過。

**尚未驗證**：ARM 交叉編譯、STM32 實板、RYLR998 實際收發、延遲與掉包率、手把端 MCU 程式（尚未撰寫）。
