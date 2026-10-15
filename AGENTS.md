# robot_remote — Codex / Claude 共用工作規則

## 唯一共用入口

- Codex 與 Claude 每次開始工作都先讀本檔、`README.md` 與 `git status --short`。
- 本資料夾的需求、決策、目前狀態、驗證證據與待辦，更新在本檔；不要只留在聊天、個別代理的私有記憶或 `/tmp`。
- 原始碼、設定、測試、文件、日誌及新資料集都放在本專案內。日誌在 `logs/`，設定在 `config/`，新資料集預設在 `datasets/`。不要任意搬移既有資料集或大型環境。
- 外部既有依賴保留原處；使用時在本檔或 README 註明實際位置與用途。`.venv_quest/`、`igus_ws/` 為現有本地環境。
- 既有外部執行環境：`/opt/ros/humble`（ROS Humble）、`/home/hucenrotia/.cloudxr`（CloudXR 既有 runtime/cache）；資料錄製另依 README / `recording/run_record.sh` 的 LeRobot 環境。這些是依賴，不是交接資料存放處。
- `CLAUDE.md` 連結至本檔，讓兩個工具讀到同一份規則，不維護兩份互相矛盾的交接文件。
- 同時工作前先查看下方進度及檔案變更；不要覆蓋其他代理或使用者未提交的修改。大範圍修改先在此登記負責檔案，交接時更新完成項目、測試、限制及下一步。
- 未經使用者要求，不自行新增分身代理或啟動其他聊天；共用檔案不代表可以同時操作同一台機器人。

## 實體機器人

- 左臂 `192.168.3.11`、右臂 `192.168.3.12`。CRI 控制 / 狀態 TCP 預設 3920；Position Interface 埠必須向 CRI 查詢，不硬猜。
- 預設實體路徑為 Ethernet CRI + Position Interface；ROS 只供本機 IK、視覺化及模擬使用，不透過 ROS topic 發送實體手臂命令。
- 實體驗證先用 `--check-only` 或 `--dry-run`。禁止把「能連線／模擬測試通過」寫成「實體動作已驗證」。
- 本次開發允許讀取兩臂狀態，未執行任何實體運動、馬達啟用、歸零或參考點搜尋。後續運動測試必須由現場操作者看守急停並明確啟動。
- 不自動啟用馬達、啟動時不自動復歸、不搶其他 CRI 客戶端的控制權、不因斷線自行恢復運動。單臂故障須讓本次選用的兩臂停止串流。
- 使用者決定（2026-10-16）：正常結束（Stop all / Ctrl+C）前，實體手臂以 10°/s 自動回到全軸 0°（`--park-deg`）；再按一次即停。dry run、故障（斷線、回授逾時、急停等）或 `--no-park` 時不回位。
- 模擬中的底座位姿與工具幾何尚非實體校正結果；點距離 guard 不等同完整碰撞檢查，不能作為人員安全保護。

## 目前工作與交接（2026-10-15，Codex）

- 需求：將 Claude 的單臂 ROS topic 實體版本改成雙臂 Ethernet 直連，保留 Quest 雙手 clutch / IK；工作資料集中在本專案。
- 已完成：`quest_teleop/ethernet_rebel.py` 提供兩臂獨立 CRI + Position Interface、查詢埠號、弧度／角度轉換、只傳六個手臂軸、啟動檢查、ACK／錯誤處理、命令看門狗與雙臂故障停流。
- 已完成：`quest_dual_arm_teleop.py --backend real` 使用 Ethernet；`--backend real-ros` 保留舊單臂 topic 版本；模擬仍使用 `isaac`。實體版從實測姿態開始，禁止自動 home；tracking loss 後須放開握把再握持。
- 已完成：GUI 的雙臂選擇、左右 IP 與首次 dry-run 預設；`config/real_arms.json` 為 CLI 端點來源，GUI 明確輸入的 IP 另存 `config/launcher.json`。舊 launcher 設定首次讀取時複製至專案，舊預設資料夾改為本專案 `datasets/`；不搬移舊資料或使用者自選路徑。
- 已完成：README 與 HTML 手冊更新為雙臂 Ethernet 操作方式；新資料集預設專案內 `datasets/`，既有錄製功能仍僅支援模擬。
- 原有未追蹤的 `dual_rebel_task/`、`j6_swing_test.py` 屬使用者既有內容，未變更。
- 唯讀連線觀測：本機來源 `192.168.3.54`，兩臂 TCP 3920 可達；皆回報 `V980-15-000-2`、實體 `REBEL-6DOF-03`、Position Interface `3940 false false`。
- 當時左臂 `ERROR NoError` / `KINSTATE 0` / `ESTOP 3`；右臂 `ERROR _MNE`、六軸錯誤碼 4、`KINSTATE 99` / `ESTOP 3`，馬達未啟用。此為時間點觀測，啟動時須重新檢查。
- 驗證：`./quest_teleop/run_quest_real_arm.sh --check-only` 已對兩台實體控制器成功執行；時間與完整狀態在 `logs/real_connection_check.json`（2026-10-15 21:53:44 +08:00）。只送查詢、零 jog 心跳及 QUIT。
- 驗證：23 項測試通過，記錄在 `logs/ethernet_tests.log`。測試僅連接 `127.0.0.1` 的假控制器，包含完整假 Quest → IK → TCP 控制、兩臂獨立回授、放開握把／tracking loss／重新握持、速度限制、拒絕命令、斷線、feedback stale、watchdog、motor-off/passive/program-running 啟動拒絕；ROS 整合驗證實體後端不建立手臂命令 topic。
- 完整測試重跑（bash，專案根目錄）：`source ros_env.sh && source /opt/ros/humble/setup.bash && source igus_ws/install/setup.bash && .venv_quest/bin/python -m unittest discover -s tests -v`。系統 Python 可跑 TCP 測試；缺少 ROS 時整合測試會略過。代理的 sandbox 預設封鎖 socket，執行 localhost 測試也需要允許 socket 的工具權限；此非程式測試失敗。
- 驗證：隱藏 Tk 視窗元件測試確認雙／單臂選擇、IP 欄位、dry-run 預設與重複 IP 拒絕；Python 編譯、bash 語法及 `git diff --check` 通過。未宣稱 GUI 視覺排版已驗收。
- 實體限制：尚未連接真實 Quest 做運動驗證、未啟用 Position Interface 或馬達；實機 TCP 串流與方向確認待現場操作者執行。右臂仍需由操作人員處理馬達未啟用狀態。左臂回報工具設定 `knivetool1_clear.xml`，不可當成模擬 `eef` 工具已校正；預設控制點為 flange。右臂無工具時，URDF FK 與唯讀 Cartesian 觀測相差小於 3 mm，只是幾何一致性檢查，不能代替完整校正。
- 尚未實作：實體夾爪、實體相機／錄製、完整連桿與支架／桌面碰撞檢查。雙臂 guard 沿用模擬幾何。即時命令 / 回授的超時與速度限制是軟體保護，不能代替硬體急停。
- 後續起點：先跑 `--check-only` → `--dry-run`；現場人員準備馬達及彎曲工作姿態後，才以 `./quest_teleop/run_quest_real_arm.sh --arms left right` 啟動實機控制。不要自行為了驗證開發成果而啟用馬達或移動。

## 原廠協定來源

- CRI V17：<https://wiki.cpr-robots.com/images/8/81/CPR_RobotInterfaceCRI_V17.pdf>
- Position Interface V1：<https://wiki.cpr-robots.com/images/7/70/CPR_PositionInterface_V1.pdf>
- 本地舊驅動參考：`igus_ws/src/iRC_ROS/irc_ros_hardware/src/irc_ros_cri.cpp`；勿直接更改這份忽略追蹤的外部依賴來實作本專案功能。

## 審查與修改（2026-10-16，Claude）

- 依原廠 CRI V17 / Position Interface V1 文件逐項核對 `ethernet_rebel.py`：訊框、ALIVEJOG（0.2 s、全 0）、STATUS（POSJOINTCURRENT / ESTOP / ERROR + 關節碼 / KINSTATE / OPMODE）、RUNSTATE MAIN 第 6 欄、CMD Active、CMDACK / CMDERROR、GetPositionInterface → SetPositionInterface → UsePositionInterface 順序、`MSGSTART Pos J a1..a6 MSGEND`（度數、省略 E/C）皆符合。角度換算與 `Igus_rebel` ROS driver 相同（度 → 弧度，無偏移、無正負號翻轉）。
- 新增：`EthernetArms.park()` 與 `quest_dual_arm_teleop.park_on_exit()`，正常結束前回到 `--park-deg`（預設全 0），每拍不超過速度上限 90%，套用原有就緒／追隨誤差／看門狗檢查，兩臂距離 < `--min-arm-dist` 時提前停止；回位期間 SIGINT/SIGTERM 改為立即停止。GUI 勾選框「Return both arms to all-joints 0 deg before exit」（預設開），Stop all 等待回位完成，再按一次即停。
- 修正：遙控端每拍關節步長改為傳輸層上限的 90%，避免握持瞬間編碼器微小漂移觸發「per-tick joint speed limit」使兩臂一起斷線。
- 修正測試競態：兩臂看門狗同時到期時，`pair.fault` 可能在兩條連線都關閉後才寫入；測試改為等待 fault 寫入。
- 驗證（皆為假控制器，未連實體手臂）：28 項 unittest 通過（新增 park 單元／整合測試）；端對端以 127.0.0.1 / 127.0.0.2 兩台跟隨目標的假控制器執行實際程式：單次 SIGINT → 兩臂 10 s 回到 0° 並釋放；兩次 SIGINT → 中途停止；GUI Stop all → 回位完成後關閉。
- 注意：全 0° 是直立的奇異姿態；從工作姿態回 0° 為關節空間直線，未檢查與支架、桌面、相機的碰撞，現場需確認路徑淨空。`real-ros` 舊後端不回位。
