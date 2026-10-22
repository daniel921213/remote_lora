# STM32 獨立測試專案（NUCLEO-L476RG）

用一塊**不接馬達的 NUCLEO-L476RG** 測試 `joy_api`，不需要動組員的主控韌體。板子收到手把封包後，會把結果印到電腦上，並回覆手把。

| 腳位 | 用途 |
|---|---|
| UART4：PA0（TX）、PA1（RX）| 接 RYLR998，或接筆電的 USB 轉 UART |
| USART2（板上 ST-LINK 的 USB）| 印出測試結果到電腦終端機，115200 |
| LD2（綠色 LED）| 遙控器連線正常時亮 |
| B1（藍色按鈕）| 在「釋放等待」狀態按下，模擬 Orin 接手 |

板子上用到的狀態規則（RB 切換、失聯變異常、LB 解除）只是模擬，**真正的規則由主控決定**。

## 一、建立專案

1. 安裝 **STM32CubeIDE**（ST 官網免費下載）。
2. **File → New → STM32 Project**。
3. 切到 **Board Selector** 分頁，搜尋 `NUCLEO-L476RG`，選取後按 **Next**。
4. Project Name 填 `rc_lora_test`，按 **Finish**。
5. 出現「Initialize all peripherals with their default Mode?」，按 **Yes**。

## 二、設定 UART4

專案建好後會自動打開 `rc_lora_test.ioc`：

1. 左邊選 **Connectivity → UART4**，Mode 選 **Asynchronous**。
2. 看右邊的晶片圖，確認 **PA0 = UART4_TX**、**PA1 = UART4_RX**。如果被放在其他腳，點 PA0 選 `UART4_TX`，再點 PA1 選 `UART4_RX`。
3. 下方 **Parameter Settings**：Baud Rate `115200`、Word Length `8 Bits`、Parity `None`、Stop Bits `1`。
4. 下方 **NVIC Settings**：勾選 **UART4 global interrupt**。
5. 按 **Ctrl+S** 存檔，問要不要產生程式碼時按 **Yes**。

USART2（ST-LINK）和 LD2、B1 在第一步選 Yes 時已經自動設定好，不用改。

## 三、加入程式檔

| 從 | 檔案 | 放到專案的 |
|---|---|---|
| `D:\car\rc_lora\src\` | `rc_proto.h`、`joy_api.h` | `Core/Inc/` |
| `D:\car\rc_lora\src\` | `rc_proto.c`、`joy_api.c` | `Core/Src/` |
| `D:\car\rc_lora\stm32_test\` | `rc_test_app.h` | `Core/Inc/` |
| `D:\car\rc_lora\stm32_test\` | `rc_test_app.c` | `Core/Src/` |

用檔案總管複製進去後，在 CubeIDE 左邊的專案上按右鍵 → **Refresh**。

## 四、修改 `main.c`（3 個地方）

打開 `Core/Src/main.c`，在**已經存在的** `USER CODE` 註解之間加上：

```c
/* USER CODE BEGIN Includes */
#include "rc_test_app.h"
/* USER CODE END Includes */
```

```c
  /* USER CODE BEGIN 2 */
  rc_test_init();
  /* USER CODE END 2 */
```

```c
    /* USER CODE BEGIN 3 */
    rc_test_loop();
  }
  /* USER CODE END 3 */
```

一定要寫在 `USER CODE BEGIN` 和 `END` 中間，否則下次產生程式碼時會被刪掉。

## 五、編譯和燒錄

1. **Project → Build Project**（槌子圖示），下方 Console 顯示 `0 errors` 才繼續。
2. NUCLEO 用 USB 接電腦，接板子上 ST-LINK 那個 USB 接頭。
3. **Run → Run As → STM32 C/C++ Application**。第一次會跳出設定視窗，直接按 **OK**。
4. 燒錄完成後程式會自動開始執行。之後板子只要通電，就會自動執行這支程式。

## 六、看輸出

1. 打開「裝置管理員 → 連接埠」，找到 **STMicroelectronics STLink Virtual COM Port (COMx)**。
2. 用 Tera Term 或 PuTTY 開這個 COM，速度 `115200`。
3. 按板子上的黑色重置鈕，應該看到：

   ```
   rc_lora STM32 test: waiting for handheld packets on UART4 (PA0 TX, PA1 RX)
   ```

## 七、測試

### 用 USB 轉 UART 直接接（不用 LoRa）

| USB 轉 UART（3.3 V）| NUCLEO |
|---|---|
| TX | **A1**（PA1）|
| RX | **A0**（PA0）|
| GND | GND |

A0、A1 在板子左下方的 Arduino 排針上。

**快速確認有沒有通**：再開一個終端機視窗，開 USB 轉 UART 的 COM（115200），並把換行設定成 **CR+LF**（Tera Term：Setup → Terminal → New-line → Transmit 選 CR+LF）。貼上這行後按 Enter：

```
+RCV=1,32,01110100000001000000000000007AC5,-45,10
```

- ST-LINK 的視窗會顯示 `event: LINK_RESTORED`、`link OK`，綠燈亮起。
- USB 轉 UART 的視窗會收到 `AT+SEND=1,16,...`，這是 STM32 的回覆。
- 0.3 秒後顯示 `event: LINK_LOST`，綠燈熄滅，因為手動貼一次之後就沒有後續封包了。這是正常的。

**用 F710 連續操作**：需要筆電程式的「接真的 COM port」模式，目前還沒加。

### 用 LoRa

把車上那顆 RYLR998（`ADDRESS=2`）接到 A0、A1、3.3 V、GND，筆電那顆（`ADDRESS=1`）接到筆電。模組設定見 [../RC_LORA_PROTOCOL.md](../RC_LORA_PROTOCOL.md) 第 2.2 節。

## 預期輸出範例

```
event: LINK_RESTORED
link OK  drive +0%  turn +0%  rssi -52
event: ENGINE_ON (START)
link OK  drive +100%  turn +0%  rssi -52
link OK  drive +100%  turn -46%  rssi -52
event: CONTROL_TOGGLE (RB)
indicator -> RELEASED
indicator -> ORIN            ← 按了藍色按鈕
event: LINK_LOST
indicator -> FAULT
link LOST  drive +0%  turn +0%  rssi -52
```

`turn -46%`：右搖桿推一半（-50），扣掉 8% 死區後是 -46%，這是正常的。

## 驗證狀態

2026-10-22：在電腦上用假的 HAL 函式模擬 UART 逐位元組接收，`rc_test_app.c` 的輸出、回覆與 LED 行為都符合預期。**尚未在實際的 NUCLEO 板上編譯和執行。**
