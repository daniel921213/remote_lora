# 遙控器 → LoRa → Orin → STM32（remote_orin）

用 F710 手把遙控割草車。手把的訊號經過 LoRa 送到 Orin，由 Orin 決定要聽遙控器（JOY）還是自動導航（ORIN），再把左右履帶的轉速送給 STM32。

```text
F710 ─▶ 手把 MCU ─▶ RYLR998 ～LoRa 923 MHz～ RYLR998 ─▶ Orin ───────────▶ STM32（10/28 韌體）
        現在：筆電                                      orin_bridge.py
        之後：Feather RP2040                            new_mower_link.py
```

封包格式見 [RC_LORA_PROTOCOL.md](../RC_LORA_PROTOCOL.md)（v0.3）。

## 目前進度（2026-10-29）

| 項目 | 狀態 |
|---|---|
| LoRa 鏈路（兩顆 RYLR998） | ✅ 10/22 實測通過 |
| Orin 收到 F710 的訊號 | ✅ 10/29 實測通過 |
| Orin 接假 STM32（真的 LoRa、真的 F710） | ✅ 10/29 實測通過 |
| 實車用 Orin 鍵盤輸入 `ARM` 解鎖 | ✅ 10/29 ARM 成功 |
| 改用 F710 的 **LB** 解鎖 | 🟡 程式寫好，還沒上車 |
| 筆電的 LoRa 設定工具 | 🟡 程式寫好，還沒在硬體上跑 |
| 遠距離測試 | ⬜ 還沒做 |
| Feather RP2040 手把 | ⬜ 程式編譯過，還沒上實板 |

## 各個程式在做什麼

| 檔案 | 跑在哪裡 | 做什麼 |
|---|---|---|
| `handheld_pc/` | 筆電 | 讀 F710，假裝成手把 MCU，經 LoRa 送出；`lora_setup.py` 用來設定筆電那顆 LoRa |
| `handheld_rp2040/` | Feather RP2040 | 之後取代筆電的手把程式 |
| `common/` | 共用 | 封包格式、手把邏輯（筆電和 RP2040 用同一份） |
| `orin/orin_bridge.py` | Orin | 主程式：收 LoRa → 判斷 JOY／ORIN → 送轉速給 STM32 |
| `orin/remote_logic.py` | Orin | 搖桿換算轉速、各按鍵的規則 |
| `orin/motion_authority.py` | Orin | 行走鎖（ARM）的流程 |
| `orin/new_mower_link.py` | Orin | 組員 10/28 版的 STM32 通訊程式 |
| `orin/fake_stm32.py` | Orin 或筆電 | 沒接 STM32 時用的模擬 |

`orin_bridge.py` 不會用到組員 `~/fucking_dog` 裡的任何檔案。

## F710 按鍵

| F710 | Orin 做什麼 |
|---|---|
| 左搖桿前後、右搖桿左右 | JOY 模式：換算成左右轉速，推到底 = `--max-rpm`（預設 10） |
| **LB** | **解開行走鎖（ARM）**，跟在 Orin 輸入 `ARM` 按 Enter 一樣 |
| RB | JOY ⇄ ORIN 切換。ORIN 模式下履帶以固定低速往前，代替還沒有的導航程式 |
| B | 停車，回到 JOY |
| BACK | 熄火並鎖定停車（STM32 進 ESTOP） |
| START、X、Y、A、十字鍵 | 目前沒有功能 |

## 行走鎖（ARM）

10/28 版韌體開機會自己進 AUTO，但這時只是待命，收到轉速也不會動，要先解鎖：

1. 放開搖桿，按 **LB**。
2. Orin 等 STM32 回報 arm_ready（STM32 連續收到 0 轉速 0.5 秒就會回報），然後只送一次 ARM。
3. STM32 回報 run_request，F710 震 1 下，搖桿就能開車。

按 LB 被拒絕時，手把不會有反應，原因只顯示在 Orin 畫面上，最常見的是搖桿沒回中間。

下面任何一種情況都會重新上鎖，要放開搖桿**再按一次 LB**：LoRa 失聯、按 BACK、STM32 離開 AUTO 或故障、STM32 取消行走許可。Orin 不會自己重送 ARM，也不會自己解除 ESTOP。

詳細流程見 [ARM_USAGE.md](ARM_USAGE.md)。

## 自動停車

- 遙控器失聯超過 0.3 秒，或 F710 斷線：停車、回到 JOY、重新上鎖。
- STM32 離開 AUTO：停車並回到 JOY。
- `orin_bridge.py` 結束：送 ESTOP，車子鎖住。
- `orin_bridge.py` 當掉：STM32 0.2 秒沒收到命令會自己進 ESTOP。

F710 的震動：等待 AUTO 震 2 下、JOY 震 1 下、ORIN 震 3 下、異常長震 1 秒、跟 Orin 失聯則每 3 秒長震一次。

## LoRa 設定

兩顆模組要設成一樣：**透傳模式、UART 115200、923 MHz、PARAMETER 5,9,1,24、功率 22 dBm**，跟組員地面站的 gs 設定相同。

Orin 用 `/dev/lora_gs` 那顆。不要用 `/dev/lora_rtcm`（920.5 MHz）：RTK 基地台每秒會在那個頻道送資料，手把會一直誤判失聯。

## 待辦和已知限制

**下次上車要確認**

- 用 LB 解鎖、開車、停車、失聯後再解鎖，在真的 STM32 上都正常。

**要跟組員討論**

- **ESTOP 要不要讓 F710 解除**：現在 LB 只解行走鎖，ESTOP 要用 R12 解除。如果讓 LB 也能解，等於遙控器可以遠端解除急停。
- **不用 R12（SBUS）**：要改韌體。現在沒有 SBUS 會一直判定故障，ESTOP 也解不開。拿掉 R12 之後就沒有備用的停車方法，建議車上加實體急停按鈕。
- 923 MHz 跟組員地面站是同一個頻道，地面站開著的時候可能會互相干擾。

**還沒做**

- 筆電面板的文字還是舊的：LB 標成「解除停止鎖定」，等待狀態寫「按 R12 的 A」。
- RB 目前只是單純切換，「釋放後等地面站接手」之後再做。
- 刀頭升降（液壓）還沒接到按鍵。
- 封包還是文字格式，單程約 40 ms，之後改成二進位。
