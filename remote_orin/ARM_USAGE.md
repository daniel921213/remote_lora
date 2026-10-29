# 新版 STM32 ARM 通訊

`orin/new_mower_link.py` 是使用者提供的新版協定客戶端，獨立於組員
`~/fucking_dog/stm_link/mower_link.py`；沒有改動組員的檔案或服務。

啟動（不要帶舊的 `--mower-link-dir ~/fucking_dog/stm_link`）：

```sh
cd ~/remote_lora/remote_orin/orin
python3 orin_bridge.py --lora-port /dev/lora_gs --stm32-port /dev/ttyACM0 --max-rpm 30 --log /tmp/handheld-stm32-new.jsonl
```

R12 切 AUTO、F710 搖桿回中，按 F710 的 **LB**；或於執行 bridge 的終端機輸入大寫 `ARM` 並按 Enter（備用），兩者效果相同。
LB 被拒絕時手把不會有反應，原因只顯示在 Orin 畫面（例如搖桿沒回中）。
程式維持零速等待 arm_ready，僅送一次 ARM；收到新遙測的行走許可及序號確認後才放行。
畫面出現 `ARM ACTIVE` 或 `ARM 成功` 後，再由操作人員測試搖桿。
等待過程仍維持 LoRa 回覆與 STM32 心跳，不會卡住數秒。

失聯、遙測過期、故障、離開 AUTO 或 MCU 撤銷行走許可，會取消本程式的
行走許可。恢復後需再次回 JOY、搖桿置中，再按一次 LB（或輸入 `ARM`）。不自動清除 ESTOP。
lease 是每筆遙測更新的通訊 token，每個命令沿用最新值；正常更新不取消 ARM。
`q` 或 Ctrl+C 結束仍會送 ESTOP，恢復依現場 R12 流程。
RB 保留原本固定轉速 ORIN 測試模式；ARM 申請只能在 JOY 置中時進行。

畫面分開顯示搖桿「要求」、最後成功「送出」、STM32「實測」、PWM、ARM 階段、
ready、run 和 health。JSONL 記錄實際完整寫出的封包與收到的遙測。
不支援新版 explicit ARM 的韌體會保持鎖定，不能只用 AUTO 狀態繞過。

離線驗證：`python3 tests/run_tests.py`。實機唯讀通訊檢查不代表履帶已運轉成功。
