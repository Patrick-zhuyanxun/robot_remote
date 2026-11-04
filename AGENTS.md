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
- 預設實體路徑（2026-10-16 使用者決定）為 igus ROS driver（`real_robot/dual_arm_driver.launch.py`，/left、/right）+ teleop `--backend real-ros` 以 ROS topic 送軌跡命令，與 Isaac 模擬相同做法。Ethernet CRI + Position Interface 直連保留為實驗選項 `--backend real`。
- 實體驗證先用 `--check-only` 或 `--dry-run`。禁止把「能連線／模擬測試通過」寫成「實體動作已驗證」。
- 本次開發允許讀取兩臂狀態，未執行任何實體運動、馬達啟用、歸零或參考點搜尋。後續運動測試必須由現場操作者看守急停並明確啟動。
- （已更新，見文末「實機改走 igus ROS driver」：現在連上後先回 home、結束回全軸 0°；driver 連線時自行啟用馬達。）Ethernet 直連版：馬達啟用只在使用者選擇 `--enable-motors`（GUI「Enable motors at start」預設勾選，2026-10-16 使用者要求）時，且只在唯一問題是 MNE 時送 Reset + Enable、不搶其他 CRI 客戶端的控制權、不因斷線自行恢復運動。單臂故障須讓本次選用的兩臂停止串流。
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
- 注意：全 0° 是直立的奇異姿態；從工作姿態回 0° 為關節空間直線，未檢查與支架、桌面、相機的碰撞，現場需確認路徑淨空。（`real-ros` 現在也會啟動回 home、退出回 0°，見文末。）

## 第二輪審查（2026-10-16，Sonnet 唯讀審查 → Claude 修正）

未發現會讓手臂意外移動或無法停止的問題；以下皆為「安全方向」的失敗（跳過回位、誤停兩臂、清理指令沒送到），已修正並各有回歸測試（修正前的程式碼上會失敗）：
- Ctrl+C / Stop all 若剛好落在送命令途中，傳輸層會整個關閉 → 跳過回位。改為 SIGINT/SIGTERM 只設停止旗標，迴圈在 tick 邊界結束；`EthernetArms.send()` 遇 KeyboardInterrupt 不再關閉連線。啟動階段 SIGTERM 改為直接中止（rclpy 原本只把 ok() 設 False）。
- 握持第一拍：快照與即時讀值不同步，只有 0.067° 餘裕。`target()` 在串流開始的第一拍額外容許 0.5°；實機加入握把遲滯（≥ clutch+0.1 接合、< clutch-0.1 放開），且啟動後須先放開一次才能第一次接合。
- `abort()` 直接 close 帶未讀資料的 socket 會變 RST，QUIT 常送不到（40 次中只到 13–17 次）、`SetPositionInterface false` 偶爾遺失（持久化設定）。改為半關閉後讀到 EOF（≤ 0.3 s）再關。
- 回位途中的 OSError 等例外會跳過 `xr.close()`：`park_on_exit` 改捕捉所有 Exception，收尾用巢狀 finally。`--park-deg` 超出關節限位時拒絕移動。
- CRI 以 strict ASCII 解碼，非 ASCII 程式名稱會讓兩臂停機：改 latin-1；RUNSTATE 執行狀態改從欄位尾端取（程式名稱可含空白）。
- GUI：SIGTERM 3 s 後升級 SIGKILL；回位中關閉視窗會先詢問。
- 驗證：34 項測試通過；實際程式 + 兩台跟隨假控制器，SIGINT 與 SIGTERM 各一次 → 兩臂 10 s 回到 0°、釋放 Position Interface、送出 QUIT。

尚未處理（需實機確認）：
- `UsePositionInterface true` 後立即送目標，控制器的平均間隔尚未建立；可能需要先以 tick 頻率重送數次目前姿態暖機。
- 每拍上限以「拍」計而非實際時間；IK 耗時抖動可能讓瞬間速度略高於名目值（0.9 係數僅部分抵銷）。
- 追隨誤差用 Position Interface 的 `Pos J` 比較；若該值是插值設定點而非編碼器，檢查會失效，可再與 STATUS `POSJOINTCURRENT` 交叉比對。
- 全 0° 為奇異姿態，控制器若在接近時回報 KINSTATE 21–24，回位會在差幾度處中止（"failed, arms stopped"）。
- 每次執行會切換持久化的 `SetPositionInterface`；若可接受常開，可考慮不關。
- 單臂 ACK 慢於約 0.2 s 會觸發另一臂看門狗 → 兩臂安全停止，但不回位。

## 預設姿態（2026-10-16，使用者決定）

- home 姿態改為 (0, 0, 65, 0, 50, 0)°：`quest_arm_teleop.HOME_DEG`（模擬起始姿態、`--real-home`）、Ethernet 退出回位 `--park-deg` 預設值、MoveIt SRDF 的 `home` named state（另保留 `zero` = 全 0°）。SRDF 修改存於 `igus_ws/patches/Igus_rebel-home_pose.patch`，`setup/install_ros_ws.sh` 會套用所有 `Igus_rebel-*.patch`。
- 左臂（192.168.3.11）以舊 ROS driver + RViz 實機操控已可運作（2026-10-16）：driver 啟動時那一輪 Enable 未生效，需再呼叫 `/igus_rebel/reset_and_enable` 才變 `NoError, KinOK`。右臂由其他人使用，勿連線。

## RViz 實機事前測試與馬達啟用（2026-10-16，Claude）

- `ros2 launch real_robot/rviz_pretest.launch.py arm:=left|right [enable_motors:=false] [hardware_protocol:=mock_hardware]`（使用者要求用 ros2 launch，不用 sh）：include `igus_rebel/rebel.launch.py`（IP 取自 `config/real_arms.json`）+ 6 s 後 `ros2 service call /igus_rebel/reset_and_enable` + `move_group.launch.py`（RViz），全部以 SetEnvironmentVariable 設為 ROS_DOMAIN_ID 91、localhost only。mock 已驗證（driver 環境變數正確、/joint_states 可讀、Ctrl+C 無殘留）；啟用步驟只能實機驗證。不再檢查 Quest 遙控是否同時執行，操作者須自行先關閉。
- 使用者以 Quest 乙太網路版啟動時遇到 `_MNE`（舊 ROS driver 關閉後馬達停用）。新增 `EthernetArm.enable_motors()` / `--enable-motors`：僅在 ESTOP=3、CRI active、無程式執行、錯誤只有 MNE（關節碼 0/4、KINSTATE 0/99）時送 Reset → Enable，並等待 STATUS 變為無問題（≤ 6 s），否則拒絕。Reset 也會重置機器人的數位輸出。新增 3 項測試，共 37 項通過。
- 手冊 Real robot 頁新增「1 · RViz pre-test」分頁（按鈕燈色、一條指令流程、不動時的排查）。

## 兩臂相對碰撞防護（2026-10-16，Claude）

- 使用者提供手眼校正 `config/calibration/dualcal_20261012_left_hovercorr.calib`、`dualcal_20261009_right_single.calib`（camera_link 在各臂 base_link，皆標示未驗證）。`config/arm_calibration.json` 整理兩者；由此推得右臂底座在左臂座標系 (−0.007, −0.448, 0.005) m、約 3° yaw / 1° roll（文件記載實體臂距 0.448 m 吻合；模擬 USD 為 0.460 m、無偏轉）。
- 新 `quest_teleop/arm_collision.py`：每臂以膠囊表示（J1→J2→…→J6→法蘭，法蘭→XEG-32 夾爪中心、法蘭→刀尖；半徑 `--arm-radius 0.06`、`--tool-radius 0.04`），兩臂表面最小間距 < `--min-arm-dist`（預設改為 0.05 m 表面間距；舊版為 12 cm 點距離）且下一步會更近時拒絕該步並要求重新握持。實體後端用校正底座（world = camera_link），模擬用 USD 底座。回位（park）也使用同一 guard。
- Ethernet 只選一臂時，另一臂以唯讀 CRI 連線（`watch_arm()`：只讀 STATUS，若成為 active 立即 `SetActive false`，從不設定 Position Interface、不送 Reset/Enable）提供姿態給 guard；其狀態逾時則被驅動的手臂保持不動。`--no-watch-other` 關閉（無兩臂保護）。
- 驗證：segment 距離函式 300 組隨機暴力比對；新增 3 項整合／單元測試（左臂被假 Quest 推向右臂：有旁觀時停在 4.7 cm、旁觀臂只收到查詢與 SetActive false；不旁觀時越過 5 cm；校正臂距 0.448 m、home 間距 > 0.2 m、J1 −90° 伸展時重疊），共 40 項通過。未在實機驗證。未建模：支架、桌面、相機；膠囊半徑為估計值。
- 使用者決定（2026-10-16）：只開一臂時，另一臂預設**假設維持全軸 0°**（`--other-arm-deg`，預設全 0），不連線其控制器（因為常常不會兩臂都開）。唯讀旁觀改為選用 `--watch-other`。測試：左臂推向假設直立的右臂時停在 4.9 cm，右臂控制器未收到任何連線；`--watch-other` 時停在 4.7 cm。40 項通過。

## 實機第一次握持即斷線 + 實機 RViz（2026-10-16，Claude）

- 實機錯誤：`right: unexpected Position Interface message: ['OK']`。RobotControl V15 的 Position Interface 對每個目標回 `MSGSTART OK MSGEND`（V1 文件未記載），舊程式視為未知訊息而讓兩臂故障停止，所以一握持就斷線、手臂沒動。修正：`OK` 略過；`ERROR …` 視為拒絕並停止；其他未知訊息忽略但記錄在 `unknown_position_messages`。假控制器現在預設對每個目標回 OK（更貼近實機），新增 2 項測試。
- 注意：該次使用者同時選了左右兩臂，右臂也被 Reset + Enable（先前右臂由他人使用）。
- 實機 RViz：teleop 發布 `/quest_teleop/joint_states`（兩臂量測關節；未驅動臂為假設／旁觀姿態）、`/quest_teleop/{side}/hand`（PoseStamped，frame `quest` = 操作者座標系）、`/quest_teleop/state`（JSON 握持狀態）。`teleop_viz.py --source real`：world = 左臂 base_link，右臂底座由手眼校正放置，控制器 TF `{side}/controller`，綠／白／紅球表示握持／放開／追蹤遺失。GUI Real 模式勾選「RViz: live arms + Quest controllers」（預設開）會在手臂連上後開啟，Stop all 時在回位後關閉。以兩台假控制器 + 實際 teleop + 無視窗 viz 驗證資料流（關節、TF、狀態）與 GUI 式關閉；42 項測試通過。未在實機與實際頭盔驗證畫面。

## KINSTATE 51、串流暖機、啟動回 home（2026-10-16，Claude）

- 實機（右臂）一握持即 `KINSTATE=51`（igus 文件未定義；50 = 關節位置 NaN）。推測原因：開始串流時先送種子姿態、幾毫秒後即送第一個目標，控制器以「平均間隔」內插，間隔約 ms 導致速度檢查觸發運動錯誤（Position Interface 文件：間隔短而目標遠 → kinematic error）。修正：`begin_stream()`（種子 + UsePositionInterface true）後，所有串流中的手臂一起以 tick 頻率重送目前姿態 4 次（`warmup_ticks`）再跟隨目標；第一個目標在送出任何東西前先驗證。尚未實機驗證此推測。
- `enable_motors` 也會 Reset 鎖住的運動錯誤（KINSTATE 13/14/50/51/99，關節碼僅 0/4）；`problem()` 附上 KINSTATE 文字說明。
- 使用者決定：Ethernet 連上後先以 10°/s 移到 home (0, 0, 65, 0, 50, 0)°（`start_home()`，同 park 的串流與兩臂 guard；`--no-start-home` 略過，GUI 勾選預設開），關閉時回全軸 0°（`--park-deg` 預設改回全 0）。44 項測試通過（含啟動姿態 = home、退出 = 0 的整合測試）。
- 使用者問 sh 是否 source ROS：有（ros_env.sh、/opt/ros/humble、igus_ws），但 Ethernet 控制不經 ROS，KINSTATE 為控制器回報，與 ROS 無關。

## 實機改走 igus ROS driver（2026-10-16，Claude；使用者要求「用 ROS2 topic 送 command，像 Isaac 一樣」）

- Position Interface 直連在 RobotControl V15 上：回 `OK`、CONFIG SetPositionInterface 不回覆、串流起步 KINSTATE 51（暖機後仍發生）。另寫了 CRI ALIVEJOG 速度點動模式（`--control jog`，仿 driver：2 %/(deg/s)、10 ms、0.4 ff + 1.0 誤差），假控制器測試通過但未上實機；直連整條路徑保留為實驗選項 `--backend real`。
- 預設實機路徑改為 `--backend real-ros`：`real_robot/dual_arm_driver.launch.py`（每臂一組 ros2_control_node / RSP(frame_prefix) / spawner，namespace /left、/right，controller.yaml 改以 `/**/` 為鍵；6 s 後 `/<side>/igus_rebel/reset_and_enable`；ROS_DOMAIN_ID 42 localhost）。teleop 讀 `/<side>/joint_states`，以 30 Hz 單點短軌跡送 `/<side>/rebel_arm_trajectory_controller/joint_trajectory`；啟動回 home、退出回 0°（`move_to()`，含兩臂 guard）。`run_quest_real_arm.sh` 預設 real-ros。GUI Real 模式：先啟 driver（等 success=True × 臂數）→ teleop → RViz；Stop all：回 0° → teleop → viz → driver。
- **實機驗證（使用者授權、確認路徑安全）**：兩臂 driver 啟動並啟用 → teleop real-ros 將兩臂移到 home（左 64.2/49.2°、右 64.1/49.3°）→ Ctrl+C 回 0°（殘差 < 1°）。未接 Quest 實際遙控；GUI 流程以 mock driver 驗證。48 項測試通過。
- 遺留：driver 連線時本身會 SetActive/Reset/Enable（dry run 也會）；JTC goal tolerance 使回位殘差 ~0.7–1°。

## 執行中切換旋轉座標系（2026-10-16，Claude）

- 使用者要求：連線後的執行畫面多一個按鈕，選擇 tool frame 或 base frame。沿用既有 `--rot-frame tool|base`（只影響旋轉；平移一律沿手臂底座軸）。
- GUI 執行頁新增「Rotation: TOOL/BASE frame」按鈕（模擬與實機皆有）：經 teleop 的 stdin 送 `rot_frame base|tool`（GUI 啟動 teleop 時開 stdin pipe，同 recorder 做法）；按鈕標籤依 teleop 的確認日誌 `rotation frame: …` 更新，確認前停用；選擇寫入 `config/launcher.json` 供下次啟動。終端機啟動時可用 `ros2 topic pub --once /quest_teleop/set_rot_frame std_msgs/msg/String "data: base"`。
- 安全：握持中的手臂保留原座標系，放開再握才套用（兩種映射對同一手勢給出不同目標，中途切換會讓工具擺動）。
- 驗證：新增整合測試（握持中切換不生效、重握後生效、錯誤指令忽略）與隱藏 Tk 測試（按鈕送指令、依確認更新、存設定）；50 項測試通過。未在實機或實際頭盔上試手感。

## 遙控卡頓 / 封包堆積排查（2026-10-19，Claude）

- 使用者回報：模擬遙控前段順、後段卡頓。檢查整條路徑 Quest → teleop → `/dual_arm/isaac_joint_commands` → Isaac → `/dual_arm/isaac_joint_states` → teleop。
- 確認的堆積點 1（Isaac 命令端）：`ROS2SubscribeJointState` 每個 tick 只取佇列中一筆，預設 queueSize 10。模擬 FPS 低於 teleop 30 Hz 時命令排隊，手臂逐筆執行舊命令、落後最多 10 筆。實測（headless、連發 9 筆舊目標 + 1 筆新目標）：queueSize 10 時手臂先往舊目標偏 3.70°，改 1 後 0.00°。`run_dual_arm_sim.py` / `dual_arm_rig_ros2_bridge.py` 改為 queueSize 1。
- 確認的堆積點 2（teleop 狀態端）：主迴圈每拍 `rclpy.spin_once` 只處理一個 callback，Isaac 每個渲染幀發布狀態，depth 10 佇列塞滿後每次取到的都是最舊的一筆，`q_meas`（握持快照、碰撞 guard、real-ros 落後檢查）落後約 10 筆。改為 depth 1（sim 與 real-ros 皆是）。回歸測試：舊設定落後 11 筆，新設定為最新。
- 使用者 2026-10-19 18:38 存的 `dual_arm_rig_v2.usda`（桌子升高 0.2 m：桌腳 scale z 0.7→0.9、桌面 z +0.2）連帶存進了腳本建立的 `/World/DualArmJointBridge` 圖（舊 queueSize），會讓啟動報 "A graph already exists"。使用者決定採用此 USD：已移除檔內的圖再 commit，headless 載入驗證正常。`run_dual_arm_sim.py` 另在載入後移除任何存進 USD 的 bridge 圖再重建（只改記憶體），以免日後再次存檔又出錯。
- 新增 `[timing]` 診斷（每 `--diag-period` 5 s 一行；異常時為 WARN，GUI 也顯示）：迴圈 Hz / overrun、Quest 讀取 / IK / 發布耗時 p95、joint state 頻率與年齡、模擬速度（sim 時間 ÷ 牆鐘）。判讀：loop < 27 Hz 或 ik 高 → teleop 本身忙；`sim < 90 % real time` 或 joint states Hz 下降 → Isaac 被 GPU/CPU 拖慢；age 高 → 狀態回授延遲。
- 已排除：isaacteleop `ControllerTracker` 取當下快照，不是 FIFO；GUI 的 `_pump` 執行緒持續讀 teleop stdout，管線不會塞住（`Proc.lines` 會整場累積，只佔記憶體）。
- 觀察：headless 模擬沒有限速（約 260 Hz、220 % 真實時間）；GUI 模式受渲染限制，實際數字未量。CPU governor 為 `powersave`（Isaac 啟動時也警告），可能讓模擬或 IK 後段變慢，尚未證實。
- 驗證：53 項測試通過（新增 3 項）。未用實際頭盔重跑長時間遙控，「卡頓已消除」尚待使用者以新版跑一次並看 `[timing]` 行確認。

## 紙箱任務場景（2026-10-19，Claude）

- 使用者需求：wrist 相機連桿會晃；箱子要四片蓋（原本只有前後兩片）；箱子在桌面範圍內隨機生成；任務 = 看到箱子 → 移到中間 → 打開四片；箱子要輕但不能一碰就飛。
- 做法：`dual_arm_sim/setup_box_task_scene.py`（`~/isaac-sim-6.1/python.sh` 執行，原地改 `dual_arm_rig_v2.usda`，可重跑、結果不變；pxr 讀寫此 USD 不會改格式）；隨機擺放在 `run_dual_arm_sim.py` 執行時處理。
- wrist 相機晃動根因：`xeg32_camera_mount_link` 質量 1e-9 kg、`camera_tilt_joint` 無剛性無阻尼（自由擺）。改為 0.05 kg、慣量 3e-5、角度 drive stiffness 20 N·m/deg、damping 1 N·m·s/deg、目標 0°（`--cam-tilt-deg` 可改），仍為 revolute（DOF 名單不變）。實測 joint1 ±45° 擺動時相機傾角峰對峰：改前 19.35°，改後 0.000°。
- 箱子：Body 由 kinematic 改為 dynamic、0.4 kg；PhysX：maxDepenetrationVelocity 0.5 m/s、maxLinearVelocity 2 m/s、線阻尼 0.1、角阻尼 0.5、solver 16/4、CCD。物理材質（原本沒有）：紙箱靜/動摩擦 0.6/0.5、桌面 0.5/0.4，回彈 0。
- 四片蓋：長蓋 LidBack/LidFront（0.157×0.45 m、0.035 kg）抬高 6 mm 疊在新增的短蓋 LidLeft/LidRight（0.30×0.12 m、0.02 kg）上，間隙 1 mm，所以要先開長蓋。鉸鏈：關閉 = 限位 0°（不會掉進箱內），全開 175°（往外翻平；PhysX revolute 限位超過 ±180° 會亂動，265° 時實測短蓋被推開），無彈簧，damping 0.0002 N·m·s/deg（原本 5，相當於 286 N·m·s/rad，蓋子幾乎推不動）。未過垂直放手會倒回關閉，推過垂直會倒向全開。
- 隨機擺放：每次啟動在桌面內取中心與 yaw（USD 方向 ±30°），四角距桌緣 ≥ 2 cm、中心距桌面中心 ≥ 5 cm；`--box-seed` / `--no-random-box` / `--box-yaw-deg` / `--box-margin` / `--box-min-offset`。執行中 `ros2 topic pub --once /dual_arm/reset_scene std_msgs/msg/Empty "{}"` 重新擺放（全部剛體一起移動、速度歸零、蓋子關閉）；`/dual_arm/box_pose` 發布 Body 位姿（frame world）。桌面中心 (0, −0.663)。
- 驗證（headless，皆為模擬）：`check_box_task_scene.py` 全部通過 —— 靜置 5 s 漂移 0.00 mm、四蓋 < 0.05°；四蓋 60° 放手回 0°、110° 放手到 175°；側推 0.3 m/s 滑 0.5 cm 不傾倒；壓入桌面 1 cm 的彈出速度 0.25 m/s（上限 0.5）；相機峰對峰 0.000°。20 次 reset 皆直立、在桌面上、靜止。home 姿態手臂最低點 1.196 m 高於箱頂 1.191 m。53 項 unittest 通過。
- 未驗證：實際用手臂/夾爪推箱子與開蓋的手感（需 Quest 實操）；遠側桌面接近 ReBeL 工作範圍邊緣。錄製尚未在每個 episode 自動 reset 箱子。既有 `ch_v4_*_hold` 固定關節每次啟動警告 "disjointed body transforms"（約 0.2 m z 差），與本次無關、未處理。

## 模擬卡頓判讀 + 模擬用 RViz（2026-10-19，Claude）

- 使用者實測的 `[timing]`（logs/teleop_20261019_193103.log）：teleop 迴圈 29.9 Hz、tick p50 約 1–2 ms、Quest 讀取 1.5 ms，皆正常；但 Isaac joint states 只有 **15 Hz**（GUI 渲染約 15 FPS），模擬速度從 65 % 降到 **35–40 % 真實時間**。卡頓主因在 Isaac 端（渲染 / CPU），不在 teleop。第二段下降約在 19:32:08，與 sim log 中使用者操作選單（開 Viewport）同時。GPU 當時未降頻；CPU governor 為 powersave；AnyDesk 在執行（遠端桌面擷取畫面也吃資源）。
- RViz 模擬版：GUI 勾選「RViz (sim)」（預設開）或 `quest_teleop/run_teleop_viz.sh --source isaac`。實心手臂 = Isaac 實際姿態，半透明 = teleop 送出的命令（`/quest_teleop/action` → `/cmd_{side}/joint_states` + 兩個 robot_state_publisher），球 = Quest 控制器（綠 = 握持）。畫面上有延遲面板並每 5 s 寫入 viz log：Quest 端（手部姿態 Hz、逐位元相同的凍結幀 %、最長間隔）與手臂端（命令 Hz、手臂狀態 Hz、模擬速度、手臂落後命令的最大角度），並給出判斷。
- 驗證：headless 模擬 + 假 teleop 發布：ghost 手臂 TF、控制器 TF、凍結偵測（凍結 2 s 時 83 %）、無狀態時提示；53 項測試通過。未在實際頭盔下看過 RViz 畫面。

## 卡頓再查：不是 GPU，是 CPU 主執行緒（2026-10-19，Claude）

- 使用者質疑顯存用量低。實測模擬執行中：GPU 35 %、P5、960 MHz、Isaac 顯存 1.5 GB → GPU 不是瓶頸；Isaac 主執行緒 CPU 87–100 % → 每秒約 98 幀，每幀只推進 1 個 physics step（1/120 s，由 `/clock` 量得 8.3 ms/幀），所以要 120 FPS 才是真實時間，量到 82 %；teleop / Viewport / RViz 同時跑時降到 35–65 %。
- 更正：先前 `[timing]` 的「joint states 15 Hz」是 teleop 量測假象 —— 每拍一次 `spin_once` 只有隔拍處理到狀態。實際 Isaac 發布約 95 Hz。teleop 改為每拍 spin 3 次（假 100 Hz 發布實測 15.0 → 29.7 Hz），q_meas 也因此每拍更新。
- teleop 雙手握持時 IK 區段 p95 達 40–60 ms 並 overrun；離線同樣的 IK 兩臂只要約 4 ms（在 Isaac 執行中量測 p95 4–10 ms）。live 尖峰原因未確定。teleop 的 2 條 OpenBLAS 執行緒在空轉（各約 62 s CPU，多為 sys）；設 `OPENBLAS_NUM_THREADS=1` 時 CPU 用量從 7.2 s 降到 1.2 s，但 IK p95 反而變差（9–10 ms 對 4–6 ms，推測與 powersave 下核心降頻有關），所以未採用。

## CPU 分核 + 計時 log（2026-10-19，Claude）

- 使用者已把 governor 改為 performance。另發現 `intel_pstate/no_turbo = 1`：i9-14900K 的 Turbo 被關，P-core（0–15）上限 3.2 GHz、E-core（16–31）2.4 GHz。Isaac 受單一主執行緒限制，頻率影響直接。是否開 Turbo 由使用者 / 管理者決定（14900K 有已知的高頻穩定性問題，可能是刻意關的；開之前先確認 BIOS / microcode）。
- GUI 啟動的程式依角色綁 CPU（`taskset`，子程序繼承；`ROBOT_REMOTE_NO_PIN=1` 關閉）：sim = 最快的核心扣掉最後兩個實體核（本機 0–11），teleop（含 CloudXR runtime）= 那兩個實體核（12–15），RViz / robot_state_publisher / 錄製 / driver = 慢核（16–31）。每個 log 第一行記錄綁定的 CPU。終端機手動執行時可用 `taskset -c 0-11 ./dual_arm_sim/run_dual_arm_sim.sh`、`taskset -c 12-15 ./quest_teleop/run_quest_dual_arm.sh ...`、`taskset -c 16-31 ./quest_teleop/run_teleop_viz.sh`。
- Isaac 新增 `[dual_arm] [timing]`（每 5 s）：frames/s、frame p50/p95/max、模擬速度、主執行緒所在 CPU。headless 綁 0–11 實測 131 frames/s、frame p50 6.8 ms。
- teleop `[timing]` 的 ik 區段拆出 `solve`（IK 解算本身，兩臂合計，含 max）與 `guard`（兩臂碰撞檢查），並記錄主執行緒所在 CPU。用來判斷先前 40–60 ms 的尖峰是 IK、guard 還是其他。
- 驗證：53 項測試通過；pinning 以子程序讀 `Cpus_allowed_list` 確認。尚未在實機操作下看到新的 log。
- 修正（2026-10-19）：teleop 啟動後握持即崩潰 `ValueError: Logger severity cannot be changed between calls.` —— `[timing]` 用同一行 `(log.warning if bad else log.info)(line)`，rclpy 不允許同一呼叫位置改變嚴重度，第一次由 WARN 轉 INFO 就丟例外。改為 `log_timing()` 兩個呼叫位置，新增回歸測試（舊寫法重現該錯誤），54 項測試通過。

## 箱子黏在手臂上：排查紀錄（2026-10-19，Claude；未修改場景）

- 使用者回報：推箱子後手臂離開，箱子跟著手臂走。Codex 調查已依使用者要求取消，未改動專案。
- headless 重現（PhysX contact report，左臂夾爪朝下、手指全開 = teleop 未按 trigger）：**側推後收回不會黏**（推 7.7 cm、收回只滑 0.7 cm、不跟隨，所有變體皆同）。
- 會拖動箱子的情境：(1) 夾爪被命令壓進已關閉的蓋子 2.5–3 cm（箱在桌上無處可退，15000 N·m/deg 的位置驅動等於無限大的力 → 穿過 5 mm 蓋子、被蓋子從下方勾住）；(2) **長蓋開到 175° 時是一片在箱頂高度、朝手臂伸出 16 cm 的硬棚板**，從棚板下推牆再離開時，夾爪／手肘勾住棚板（contact：LidBack↔夾爪、LidFront↔motor3），箱子被拖 3–4 cm。
- 無效或更糟、不要採用的變體：關相機支架碰撞（改由 xeg32_base_link 勾住）、maxDepenetrationVelocity 10、contactOffset 2 cm、手臂驅動剛性降為 1500、鉸鏈軟限位（PhysxLimitAPI k 0.05／1：蓋子被翻到 45–125°，一次把箱子抬高 7 cm）、全開限位 110°（立起的蓋子被手臂掃到）。
- 測試陷阱：IK 軌跡第一步若直接要求夾爪朝下，關節會在一幀內轉 86°，手臂甩過箱子，看起來像「沒接觸就動」。改為姿態 slerp、每幀關節 < 1.6°。手指目標未設定時停在 0（半閉），會夾住 6 mm 牆。
- 下一步：請使用者重現時錄 `ros2 bag record /quest_teleop/action /dual_arm/isaac_joint_states /dual_arm/box_pose /quest_teleop/state -o logs/sticky_box`，用實際命令在 headless 重播並看接觸對；並確認當時蓋子是否已打開。
- **使用者決定（2026-10-19）：箱子固定不動，只有四片蓋子可開合**，以避開箱子被手臂拖動等問題。`setup_box_task_scene.py` 把 Body 設為 kinematic（已套用到 USD）；`run_dual_arm_sim.py` 預設箱子在 USD 位置 (0, −0.55)，`--random-box` 才在啟動時隨機擺放；`/dual_arm/reset_scene` 改為把四片蓋子關回去。驗證：`check_box_task_scene.py` 全過（箱身 0.00 mm）；先前會拖動箱子的四種手臂軌跡（側推、壓牆緣、壓蓋子、從開蓋下方推）箱身位移全為 0；reset 實測關閉蓋子。蓋子仍可能被手臂撐住或勾起（屬開合動作）。

## 遙控器重置場景（2026-10-19，Claude；使用者要求，錄製時不用中斷）

- 按鍵：A/X = 開始／儲存 episode，B = 丟棄，**Y（左手）= 重置場景**（原本 B/Y 都是丟棄）。GUI 執行頁另有「⟲ Reset scene (= Y)」按鈕（經 teleop stdin `reset_scene`）。
- 重置（僅模擬）：送出 `reset` 事件（錄製中的 episode 由 record_dataset.py 丟棄）→ 兩臂放開、夾爪打開 → 以 0.4 rad/s 回到 HOME_DEG → 發布 `/dual_arm/reset_scene` 讓模擬關上四片蓋子（手臂離開後才關）→ 重新握持即可繼續。實機後端按 Y 只記錄警告，**不會因按鍵移動實體手臂**。（2026-10-21 已更新：實機 Y 會回 home，見「實機遙控跟不上 + 實機 Y 回 home」）
- 驗證：新增整合測試（假 Isaac 跟隨命令、假 Quest 握持中按 Y）：握持被放開、`reset` 事件與 `/dual_arm/reset_scene` 各一次、手臂回到 home ±1°；55 項測試通過。未用實際頭盔試按。

## 實機資料錄製 + 四台 USB 相機（2026-10-21，Claude）

- 使用者需求：實機接上四台相機（兩台腕部、一台中間第三人稱、一台刀子近景），把實機資料收集 pipeline 整合起來。
- 相機（`lsusb` / sysfs）：兩台 ARC「HD USB Camera」05a3:9230（無序號，只能以 USB 埠區分左右）、Orbbec Gemini 336（序號 CP96563000LY，彩色 = UVC interface 4）、Orbbec Gemini 305（CV27561000GA，彩色 interface 4；interface 6 另一彩色串流在 USB 2 上無法出圖）。Orbbec 的 `/dev/v4l/by-id` 不可用（多個 interface 同名 `video-index0`，曾指到深度節點）；`/dev/videoN` 每次插拔都會變。
- 新增 `recording/real_cameras.py`（+ `run_cameras.sh --list / --snapshot / --test S`）：以 sysfs 找裝置（Orbbec = serial + interface，ARC = usb_port），每台相機一條讀取執行緒、只留最新一幀與 monotonic 時間；不出圖的相機回報 stale / 停止，不阻塞錄製。設定 `config/real_cameras.json`（鍵 = 資料集影像名）。ARC 640×480 MJPG 只有 120 fps，解碼上限 60 fps。
- `record_dataset.py --source real`：state = `/{left,right}/joint_states`（igus driver；只開一臂時另一臂取 `/quest_teleop/joint_states` 的假設姿態），gripper 欄位 0（無實體夾爪）；action = `/quest_teleop/action`（gripper 欄位 = Quest trigger，只代表操作者意圖）；影像在錄製程序內直接讀 USB 相機（不經 ROS）；取樣時鐘 = 牆鐘；robot_type `igus_rebel_dual_arm_real`；記錄 late ticks。續錄既有資料集時若特徵 / fps / robot_type 不同則拒絕（避免 sim 與 real 混在同一資料集）。sim 路徑行為不變。
- teleop：(1) 修正：real-ros 單臂時 `publish_action()` 因另一臂 `all_q` 為 None 而**從不發布** `/quest_teleop/action`（新測試在舊程式上失敗）；改為使用另一臂的假設 / 旁觀姿態（`undriven_q()`，與 RViz 用的相同）。(2) 實機按 Y：送 `reset` 事件讓錄製丟棄當前 episode，仍不移動手臂。（2026-10-21 已更新：實機 Y 會回 home，見「實機遙控跟不上 + 實機 Y 回 home」）
- GUI：Real 模式可勾選錄製與實機相機（標籤取自 config）；啟動順序 driver → teleop（連上）→ recorder（開相機）→ RViz；Stop all 先存檔（recorder `q`）再讓手臂回 0°（沿用原順序，回位動作不會進 episode）。
- 預設對應（**待使用者以快照確認**）：wrist_left = ARC @ 1-3.1、wrist_right = ARC @ 1-3.3、head = Gemini 336、knife = Gemini 305。2026-10-21 13:27 快照：1-3.1 畫面被貼近物體擋住（無法判斷左右）、1-3.3 看到夾爪與實驗室側面、336 往下看地板與紙箱、305 看天花板且右側有刀具（手臂直立時）且影像轉了 90°（可在 config 加 `"rotate": 90/270`）。
- **硬體限制（實測）**：目前四台相機都接在一個 Genesys 05e3:0610 USB 2.0 hub。ARC 相機不論格式都要求最大頻寬（kernel：`Not enough bandwidth for altsetting 6`），兩台 ARC 無法同時在同一個 USB 2 hub 上串流（連 320×240 YUYV 也不行，v4l2-ctl 直接驗證 STREAMON ENOSPC）；Gemini 336 在此 hub 上反覆 -71 重新列舉；305 在 USB 2 上 interface 6 不出圖。（13:38 更正）使用者把同一個 USB 3 hub（Genesys 05e3:0626 SuperSpeed + 05e3:0610 USB 2 半邊，是同一顆 hub 的兩面）改接到 PC 的 USB 3 埠：Orbbec 走 5000 Mbit/s，兩台 ARC 走 hub 的 480 Mbit/s 半邊，**四台同時可用**（30 s 測試：ARC 各約 47 fps 解碼、Orbbec 30 fps、無 kernel 錯誤；改線後第一次測試左腕 0 fps，之後 3 次皆正常，推測為剛列舉完的暫態）。先前「兩台 ARC 不能共用 USB 2 hub」的結論只在 Orbbec 也走 480M 時成立，已更正 README。
- 驗證：新增 `tests/test_real_recording.py`（以 LeRobot Python 實際執行 run_record.sh --source real，私有 ROS domain 87，假 driver / teleop topic + 兩個循環播放的影片檔當相機：存 1 個 episode、Y 丟棄第 2 個；檢查 robot_type、fps、影像 shape、時間戳 1/15 s、state = 左臂 driver + 右臂假設姿態 + gripper 0、action、影片檔）；teleop 單臂 action + 實機 Y 測試；GUI Real 模式勾選測試；sim 路徑回歸測試（假 /clock 半速：2 s 模擬時間 = 20 幀，不是 4 s 牆鐘）。全部 59 項測試通過。負載測試（4 個 640×480 影片檔當相機、30 fps、20 s，REC_DEBUG）：601 幀、0 late ticks、add_frame ≤ 1 ms；存檔（AV1 編碼 4 路，隨機雜訊影像為最壞情況）28.6 s，存檔期間無法開始下一段。**實體相機四台同時錄製尚未驗證**（被 USB 頻寬擋住）；未連手臂、未移動手臂。
- 13:43 快照：1-3.1 鏡頭被擋住、1-3.3 看到夾爪與實驗室。**使用者確認（2026-10-21）**：1-3.1（被擋住那台）= 右腕 → `wrist_right`，1-3.3 = 左腕 → `wrist_left`；刀子近景相機（Gemini 305，`knife`）裝在左臂。已寫入 `config/real_cameras.json`。
- 13:52 Gemini 305 彩色串流（interface 4）不出圖（v4l2-ctl 也一樣、無 kernel 錯誤、無其他程式佔用），無 root 無法 USB reset，請使用者重插。推測：13:43 `--snapshot` 連帶探測 305 的 interface 6（三種格式皆失敗）後卡住。`--snapshot` 改為預設只拍設定中的相機，`--all` 才探測其他節點。
- 13:57 使用者重插 305 後：`--list` 兩台 Orbbec 5000 Mbit/s；四台同時 20 s：ARC 各約 47 fps 解碼、head / knife 30 fps；快照對應正確（wrist_right = 被遮住那台）。**實體相機 + 實際 recorder（假手臂 topic，未連手臂）錄 10 s**：301 幀、0 stale、add_frame ≤ 1 ms、四路影片皆寫入、robot_type igus_rebel_dual_arm_real；存檔（AV1 編碼 4 路）19.2 s ≈ 2 倍 episode 長度，存檔期間不能開始下一段（可考慮 `--vcodec h264` 加速，未測）。尚未做：GUI Real + 實際 driver 的完整錄製（需操作者在場啟動）。
- 14:00 使用者要求「跑實體相機收資料 pipeline，不要移動實體手臂」：以腳本驅動實際 GUI（隱藏視窗，launcher.json 導向暫存）Real 模式：`ROBOT_REMOTE_DRIVER_MOCK=1`（igus driver = mock_hardware，不連手臂控制器）+ Dry run（不回 home、不回 0°、不送軌跡）+ Record + 四台實體相機，不接 Quest。流程 driver → teleop（兩臂 connected）→ recorder 開四台相機 → GUI 錄製鍵錄 8.2 s → 存檔 → Stop all 全部結束。結果：246 幀、0 stale、時間戳間隔 1/30 s、四路影片各 8.2 s、teleop 迴圈 29.9 Hz；state/action 全 0（mock driver 關節皆 0）。log：logs/driver_20261021_135959.log、teleop_20261021_140001.log、record_20261021_140002.log。未驗證：真實 driver 的關節值寫入（真實 driver 連線即 Reset/Enable 馬達，需操作者在場）、Quest 按鍵錄製。
- 下一步：使用者改接線 → `recording/run_cameras.sh --list`（確認 Orbbec 為 5000 Mbit/s、ARC 各在不同 PC 埠）→ `--snapshot` 確認四個鏡頭位置並修正 `config/real_cameras.json`（usb_port / rotate）→ `--test 10`（四台同時 ≥ 30 fps）→ GUI Real + dry run + Record 錄一段確認資料集。

## 資料集預設位置 + 存放位置檢查（2026-10-21，Claude）

- 使用者要求：錄製預設存到 `dataset/`，並處理「選到空資料夾」與「如何接續」的情況。原本 `config/launcher.json` 的 root 是專案根目錄本身，所以根目錄多了 `dual_rebel_task/`（只有 meta/info.json、0 個 episode）。
- 預設 root 改為 `robot_remote/dataset`（GUI、`record_dataset.py --root`、`replay_episode.py`；`ROBOT_REMOTE_DATASETS` 仍可覆蓋）。讀設定時舊預設（`~/datasets`、`robot_remote/datasets`）與「專案根目錄本身」自動改為 `dataset/`，下次 Start 存回 launcher.json。`dataset/` 已在 .gitignore（既有追蹤的範例 `dataset/dual_rebel_task` 不受影響）。
- 新 `recording/dataset_check.py`（純標準函式庫，GUI 與 recorder 共用）判斷 `<root>/<name>`：new / empty_dir（空資料夾，含空子資料夾）→ 建立；shell（只有 meta/info.json、0 episode：之前開了 recorder 沒存就關）→ 重建（LeRobot 無法 resume，會去連 Hub 而失敗，已實測）；dataset → 只有 sim/real、fps、相機組合、影像尺寸都相同才接續；incomplete（有 info.json 但缺 meta/episodes：被強制結束）、not_dataset（非空且非資料集）、file → 拒絕，不寫入也不刪除。只會刪除空資料夾或 shell 的那一個 info.json。
- GUI：Save location 下方顯示 Start 會做什麼（新建／接續 N 個 episode 與其設定／不能接續的原因，紅字），隨目標、相機、解析度、fps 即時更新；Start 時不可行就擋下並說明。Browse 選到資料集資料夾本身 → 自動設為「上層 + 名稱」接續它。
- 現況分類：根目錄 `dual_rebel_task/` = shell（未刪，屬使用者既有內容，等使用者決定）；`dataset/dual_rebel_task` = sim 資料集 1 episode（Real 模式選同名會被擋下）。
- 驗證：新增 `tests/test_dataset_location.py`（狀態判斷、相容性、prepare_new 只刪空資料夾/shell；recorder 實際執行：新建 → 同名再錄一次接續成 2 episodes、空資料夾、shell 重建、非資料集資料夾拒絕且檔案保留、fps 不同拒絕且原資料集不變；GUI：舊 root 遷移、提示文字、Start 擋下、Browse 選資料集）。既有 GUI 測試因預設名稱在 dataset/ 已有 sim 資料集而被新檢查擋下，改用暫存位置。64 項測試通過。GUI 版面未截圖驗收。

## 安裝流程簡化 + 文件同步（2026-10-21，Claude；使用者要求「安裝重簡，Isaac Sim 與部分選項改成文字詢問再決定」）

- 新增 `setup/install.sh`：以文字詢問要裝什麼，列出計畫後才執行，再呼叫原有 `install_ros_ws.sh` / `install_quest_env.sh` / `install_record_env.sh`（三支維持可單獨執行）。問題：Isaac Sim 6.1（下載約 9 GB／用現有資料夾／略過）、錄製環境（新建 `.venv_record`／自己的 LeRobot 0.5.x python／略過）、舊版 iRC_ROS（預設否）。固定安裝：`igus_ws`、`.venv_quest`、CloudXR。選項 `--isaac download|skip|<path>`、`--record new|skip|<python>`、`--with-irc|--no-irc`、`--accept-cloudxr-eula`、`--write-bashrc`、`--yes`、`--dry-run`；答案也可由 stdin 逐行餵入。三個選項都給時不再確認；stdin 沒有答案且沒給選項則 exit 2（讓代理必須先問使用者）。
- 約束：`recording/run_record.sh` / `run_replay.sh` 一律借用 Isaac Sim 的 py3.12 rclpy → **錄製（含實機 `--source real`）需要 Isaac Sim**。選「略過 Isaac Sim」＝只能做實機遙控，不能錄製；提問文字、手冊、README 都明講。要讓錄製脫離 Isaac 是另一項工作（未做）。
- 安裝腳本本身不執行 sudo：apt 套件缺少時印出指令並 exit 3，人執行後重跑（`install_ros_ws.sh` 內的 `rosdep install -y` 仍可能對缺少的 apt 套件要求 sudo 密碼）。CloudXR 授權只在使用者明確傳 `--accept-cloudxr-eula` 時才執行，否則印出指令。非預設的 Isaac Sim / LeRobot 路徑印出 `export ISAAC_SIM=…` / `export LEROBOT_PY=…`（launcher 從環境變數讀）；`--write-bashrc` 才會寫入 `~/.bashrc`。
- 選擇存於 `config/install_profile.env`（已加入 `.gitignore`）；`setup/check_env.sh` 只依它決定要不要檢查（未選的項目顯示 `skip`），**路徑一律來自環境變數／預設位置，與 launcher 看到的一致**（自訂路徑沒 export 時會 FAIL 並提示；`install.sh` 在最後檢查前自行 export 該兩個變數）（也可 `--no-sim --no-record`）；沒有檔案＝全部檢查（行為與舊版相同）。
- 追加（同日，使用者要求「agent 也要先看電腦上已裝什麼、什麼版本」）：`install.sh --detect` 只列出已安裝項目與版本（Ubuntu、NVIDIA、磁碟、ROS、uv、各個 Isaac Sim［需 6.1.x，5.1 標為不可用］、各個 LeRobot python［需 0.5.x］、`.venv_quest`、`igus_ws`、CloudXR），不改任何東西；互動安裝開頭也印同一份清單，且只在偵測到 6.1.x 時才提示「使用現有 Isaac Sim」。代理 Step 0 跑 `--detect`，Step 1 先回報偵測結果，只問尚未決定的項目。以此機驗證（Isaac Sim 6.1.0-rc.26 可用、~/isaac-sim 5.1 標為不可用、LeRobot 0.5.1 可用）。
- `AGENT_INSTALL.md` 改寫：Step 1 要代理先用文字問使用者上述三題（含已偵測到的環境與磁碟空間），Step 2 帶旗標執行 `install.sh`（先 `--dry-run` 看計畫）。`README.md` 的 Install 縮成 clone + `./setup/install.sh` + 問題表。
- 手冊（`docs/operator_manual.html`）：Install 頁改為 6 步（需求／取得程式／執行安裝程式／網路／檢查／手動安裝），舊的逐步指令收進「By hand」；**原本以步驟序號記在瀏覽器的勾選會對不上新步驟（只是便利功能）**。補上實機錄製與四台相機（Real robot 新分頁「3 · Record real data」、相機表、接線）、Run 頁的實機選項與 Save location 規則、Data flow 的實機 topic / 資料集格式、Scripts 表（install.sh、run_cameras.sh、run_quest_real_arm.sh、launch 檔）、Troubleshooting（相機遺失／USB 頻寬／錄製缺 Isaac／資料集被拒）、更新「Validation & limits」（實機錄製、相機已實作；夾爪仍無）。README 修正：Y = 重置場景（原寫 B or Y 丟棄）、防撞距離 5 cm（原寫 12 cm）、實機錄製段落中過時的「不支援」。
- 驗證：`bash -n` 三支腳本；`install.sh --dry-run` 以 stdin 餵答案測過各分支（找到／找不到 Isaac、選 1／2／3、找到既有 LeRobot、stdin 關閉 → exit 2、`--yes`、全旗標不確認）；以假 `dpkg`（PATH 前置）模擬缺 apt 套件 → 印 sudo 指令、exit 3；`check_env.sh` 在無 profile／`--no-sim --no-record`／profile 設 sim=0 record=1（顯示 note）下皆正確。手冊在瀏覽器載入：無 console 錯誤、無重複 id、所有 Copy 目標存在、步驟與分頁連動正常（未做視覺截圖驗收）。
- **未驗證**：`install.sh` 的實際安裝路徑（Isaac Sim 下載解壓、`install_record_env.sh`、`install_ros_ws.sh` 由它呼叫）只做 dry-run；此機已裝齊，未在乾淨機器上跑過。測試時產生的 profile 已刪除，本機目前沒有 `config/install_profile.env`（check_env 視為全部檢查）。

## 實機遙控跟不上 + 實機 Y 回 home（2026-10-21，Claude）

- 使用者回報：實機遙控時命令不跟手，log 多次 `[right]/[left] let go: arm lags its command by 15-17 deg`；實機按 Y 手臂不動。負責 controller 的人表示 PID 已調好。
- 診斷（teleop log `logs/teleop_20261021_142100.log`）：teleop 本身正常（迴圈 30 Hz、tick 數 ms、joint states 60 Hz）。落差在「命令 → 手臂」：`igus_ws/src/Igus_rebel/src/igus_rebel/config/controller.yaml` 的 JTC 是 **velocity** 命令介面（p 1.0、ff_velocity_scale 0.4），檔內註解記錄的手臂辨識：增益 1.26、延遲 0.078 s、速度一階延遲 0.16 s、**手臂端加速度上限約 0.52 rad/s²（約 30°/s²）**；且註明這組增益是模擬調出、「NOT yet run on the real arm」。teleop 每 33 ms 送單點短軌跡（0.15 s），JTC 每收到新訊息就從**量測位置**重新起算參考（closed loop），P 項幾乎看不到誤差；手一反向，手臂要約 1.3 s 才能從 20°/s 反轉，命令早已回頭，差距累加到 15° → let go。PID 不是槓桿：PID 是針對連續規劃軌跡調的；瓶頸是手臂加速度上限。
- 離線模型（scratchpad，非專案檔）：同一手勢下，送速度 feedforward（cubic）或開 JTC open_loop_control 都仍超過 15°；只有「命令不超前量測 N°」能在所有情境下 < 15°。sweep：leash 6° @20°/s 無 trip，收尾過衝 ≤ 3.4°；@14°/s 過衝 ≤ 1.4°。
- 修改（teleop，僅 `--backend real-ros`）：`--max-cmd-lead-deg`（實機預設 6，sim 0）——每拍 `leash()` 把關節命令限制在量測值 ±6°（最大關節、等比例縮放保留 IK 方向），命令等手臂；原 15° let-go 在 leash 下不會觸發，改加 stall 檢查 `stall_check()`：命令頂在 leash（≥ 0.8×）且手臂 `--stall-s` 1 s 內移動 < 0.5° → let go「arm does not follow its command（馬達未啟用、急停、卡住）」。Ethernet 後端與 dry run 不套用 leash（Ethernet 自有每拍上限，拉回可能觸發其故障停止）。
- 實機 Y（使用者要求）：丟棄 episode → 兩臂放開 → 以 `--park-vel-deg` 10°/s、含兩臂碰撞 guard 回 HOME_DEG；任何 Quest 按鈕、重新握把（需先放開）或 Stop all / Ctrl+C 立即停在原處；按 A 停止時不會同時開始錄製（重同步按鍵狀態）。dry run 與 Ethernet 後端不動。GUI 實機執行頁加「⟲ Arms home (= Y)」、確認視窗與圖例說明。舊說明「實機 Y 不移動手臂」已標註更新。
- 注意：資料集的 `action` 現在是 leash 後的命令（距 state ≤ 6°），不是自由跑的 IK 目標。Y 在手臂仍移動時按下，move_to 從量測位置起算，可能有小頓挫（未測）。
- 驗證：新 `tests/test_real_ros_tracking.py`（假 igus driver = JTC velocity 介面 p 1.0/ff 0.4、每訊息從量測起算 + 上述 plant，實際跑 teleop real-ros 與假 Quest）：同一個快速來回手勢，無 leash → 落差 15.1°、let go（重現使用者現象）；leash 6° → 最大 6.3°、全程保持握持；手臂不動 → stall let go；Y 回 home ±1°；Y 後按 A 停在半路且只有一個 `reset` 事件；Y 中途 SIGINT（Stop all）→ 立即停止；dry run 按 Y 不動。全部 71 項測試通過。
- **未在實機驗證**：plant 常數來自舊增益時期的擬合，leash 設計對此不敏感，但 stall 檢查（1 s / 0.5°）與 6° 在實機的手感需確認。第一次實測：單臂、home 附近、慢慢握持，留意是否誤報「arm does not follow」。過衝在刀具附近重要，可用 `--max-joint-vel` 降速。
- 給 controller 負責人（建議查證，非定論）：(1) 串流單點訊息讓 JTC 每次從量測位置重起參考，P 項看不到誤差；(2) 約 0.52 rad/s² 的加速度上限才是 15°+ 落差的主因——可查機器人端 jog / 關節加速度設定（RobotControl / iRC 組態）能否提高；(3) 2026-09-30 的 p 1.0 / ff 0.4 是否已在實機跑過。

## 實機遙控提速（2026-10-21，Claude；使用者提供 RViz pretest log，要求提高速度）

- 使用者 log（右臂 MoveIt，RViz 加速度 scaling 0.1 = 0.52 rad/s²）：三段規劃軌跡皆 `Goal reached, success!`；home → 全 0（joint3 65°、joint5 50°）約 4.7 s，起步 0.57 s 時 joint3 只走 3.6°，與約 30°/s² 加速度上限一致；推估峰值約 18°/s。結論：參考軌跡加速度不超過手臂能力時（MoveIt 的做法）控制器追得很好。
- 改動（只有 `--backend real-ros`；Ethernet 後端維持 0.35 rad/s / 0.15 m/s / 30°/s，`ETHERNET_DEFAULTS`）：預設關節 0.35 → **0.52 rad/s（30°/s）**、TCP 0.15 → **0.25 m/s**、旋轉 30 → **45°/s**、leash 6 → **10°**，並新增 **`--max-joint-acc` 0.45 rad/s²**：`accel_step()` 讓關節命令以等比例縮放方式加減速（|Δv| ≤ a·dt、可在目標前煞停 |v| ≤ √(2a·剩餘)），之後再套 leash；`arm.v_cmd` 由實際送出的命令求得，握持時歸零。stall 檢查改為「目前命令距手臂 ≥ max(3°, 0.3×leash) 且手臂 `--stall-s` 內移動 < 0.5°」（加速度限制下凍結手臂不會一直頂在 leash，舊條件不觸發）。
- 離線模型 sweep：無加速度限制時 30–40°/s 收尾過衝 9–20°；有限制 ≤ 2.3°。來回動作的跟手程度不因提速改善（瓶頸是手臂加速度）。
- 驗證（假 driver + 實際 teleop）：0.3 m 橫掃停下，最大關節 48.7°：新預設 90% 到位 2.94 s、過衝 2.6°；舊預設 3.14 s、3.35°；新速度但無加速度限制過衝 11.7°。快速來回不 let go（落差 ≤ 10°），凍結手臂仍 let go。新增 `accel_step` 單元測試。73 項測試通過。
- 提速效益有限（此例約 6%，模型 60° 單向約 15–20%）：大角度移動時間由手臂約 30°/s² 加速度決定。要明顯變快需提高機器人端加速度（RobotControl / iRC 組態），屆時把 `--max-joint-acc` 調到略低於新上限即可。未在實機驗證；資料集 action 為加速度限制 + leash 後的命令。

## 最新一次執行的 log：`latest_run/`（2026-10-21，Claude；使用者要求「每次執行覆蓋一份最新 log，記錄移動與命令，方便 debug」）

- `latest_run/`（.gitignore）：GUI 每次按 Start 先清空（只刪 *.log / *.csv），各程序的輸出另存 `latest_run/<process>.log`（teleop / driver / record / viz / sim；`logs/` 的時間戳 log 照舊保留）。
- teleop 每次啟動覆蓋 `latest_run/teleop_trace.csv`（終端機執行也會；`--trace ''` 或 `ROBOT_REMOTE_TRACE=""` 關閉）：每拍每臂一列 —— t、mode（teleop / home / reset / park 與其 settle）、tracked、engaged、grip、trigger、hand_xyz（操作者座標）、target_xyz（手臂底座座標的法蘭命令）、ik1-6、cmd1-6（實際送出，經速度／加速度／leash／碰撞限制）、meas1-6、meas_age_ms、lag_deg、cmd_speed_dps、limited（speed / accel / leash / guard）、sent、note（該拍所有 log 訊息，相同訊息每秒最多一次）。檔頭含啟動時間、argv、各限制值與單位。約 8 KB/s。
- debug 時先看 `latest_run/teleop_trace.csv` 與 `latest_run/teleop.log` / `driver.log`。
- 測試：trace 內容測試（欄位、約 30 Hz、engaged 列數值、lag ≤ 10.5°、limited 有 accel/leash、note 含 engaged）；GUI 測試確認 `latest_run/teleop.log` 副本；teleop 相關測試模組設定 `ROBOT_REMOTE_TRACE=""`、GUI 測試把 LATEST_DIR 導向暫存，不會覆蓋操作者的 latest_run。74 項測試通過。

## 第一次 latest_run 檢查（2026-10-21 15:03 實機雙臂，Claude）

- 「錄了資料集卻沒東西」：該次 `config/launcher.json` 為 `"record": false`（GUI 第 4 區未勾 Record），recorder 沒啟動（latest_run 沒有 record.log），A/X 只送出 toggle 事件、無人接收。GUI 現在在錄製關閉時按 A/X 會顯示提示與黃色橫幅「RECORDING IS OFF」。
- 手臂表現（trace）：無 let go、無碰撞暫停；命令→手臂落差中位數約 4°、最大 9.8°（< leash 10°）；手臂實際速度 p95 約 26°/s；Y 回 home 兩次皆在 1° 內；結束回 0° 正常。98% 的握持時間命令受加速度限制，手部目標→手臂距離中位數 15–20°（最大 40°）。
- **發現並修正 bug**：`teleop_viz.py --source real`（GUI Real 的 RViz）把 `/quest_teleop/joint_states` 轉發到 `/left|right/joint_states`——正是 igus driver 發布、teleop（與 recorder 的 state）讀取量測值的 topic，並在 `/left`、`/right` 再開一組 robot_state_publisher（與 driver 的同名、同 TF 前綴）。結果 teleop 會讀回自己稍早的量測值：trace 中左臂 release 後量測值凍結 0.7 s（meas_age 仍 1–2 ms），接著跳 11.6°，手臂比最後命令多跑 7°；之後量測值每約 3 拍才更新。此 bug 從 real-ros（igus driver 有 /left /right namespace）取代 Ethernet 後就存在，也會汙染實機錄製的 state。修正：viz 改用自己的 namespace / frame `viz_left`、`viz_right`（relay `/viz_{side}/joint_states`、RSP frame_prefix `viz_{side}/`、world→viz_{side}/base_link、目標姿態轉發到 `/teleop_viz/{side}/target`），rviz 設定同步。新測試 `tests/test_viz_namespace.py`：舊版在 40 s 內往 `/left/joint_states` 灌了 162,605 筆而失敗；新版無任何 `/left|right/joint_states` 發布者、`viz_left` TF 正常。
- release 後過衝（各次 6–19°）中，88.11 s 那次明確由凍結的回授造成；其他次可能部分是物理（約 25°/s、約 30°/s² 需 10°+ 停止距離）——修正後請再跑一次看 trace 確認。
- 75 項測試通過。

## 事故：測試讓實體左臂移動（2026-10-21 15:18，Claude）——所有代理必讀

- 經過：使用者 15:18:15 以 GUI 啟動實機雙臂（driver 在 ROS_DOMAIN_ID 42、localhost）。同時 Claude 在同一台電腦跑 `tests/test_real_ros_tracking.py` 的加速度比較（假 driver + **真正的 teleop `--backend real-ros --arms left`**）。測試沿用 `ros_env.sh` 的 domain 42，測試 teleop 的軌跡命令送進了實體左臂的 `/left/rebel_arm_trajectory_controller/joint_trajectory`，假 driver 的 `/left/joint_states` 也被實機 teleop 讀到。實體左臂突然移動；使用者關閉後無人受傷。實機 teleop log 當時只有警告「1 other publisher(s) on /left/...joint_trajectory」。（同日 14:21、15:03 的實機 session 無外來 publisher，未受影響。）
- 修正（防止再發生）：
  1. `tests/ros_isolation.py`：所有用到 ROS 的測試模組**第一個 import**，強制 `ROS_DOMAIN_ID` / `ROBOT_REMOTE_DOMAIN_ID` = 97、localhost only、清掉 `ROBOT_REMOTE_TARGET`、關閉 trace；`check()` 遇到機器人 domain（42 / 91 / 11 / 0）即拒絕。新增測試確認測試 context 的 domain 是 97。
  2. teleop（real-ros）：啟動時若手臂命令 topic 上有其他 publisher → **拒絕啟動**（RobotError；dry run 僅警告）；執行中每 0.5 s 檢查，一出現就**放開兩臂、停止送命令**直到它消失；自動移動（home / Y / 回 0°）的 guard 也會因此停止。新增測試：啟動前有外來 publisher → 拒絕；握持中出現 → 放開且不再送命令。
- **規則**：實機 driver 可能在跑時，代理不得在 domain 42 執行任何會建立 ROS 節點的程式（測試、ad-hoc 腳本、ros2 topic pub）。ad-hoc 腳本也要先 `import ros_isolation` 或明確設定非機器人 domain；執行前先 `pgrep -af "ros2_control_node|quest_dual_arm_teleop"` 確認。
- 事故中的加速度比較數據無效（被實機干擾）；隔離後重測：假手臂（plant 上限 0.52）上 0.45 / 0.52 / 0.6 / 0.8 rad/s²：0.3 m 橫掃 90% 到位 2.94 / 2.81 / 2.71 / 2.54 s、過衝 2.7 / 3.6 / 5.0 / 6.1°，快速來回皆不 let go。實機預設改 **0.52**；GUI Real 區新增加速度選單（0.45 / 0.52 / 0.6 / 0.8，存 `real_accel`，傳 `--max-joint-acc`）。實機真正上限未確認（trace 的自動移動估計約 26–34°/s²，需更多資料）。78 項測試通過。

## 放手後過衝再拉回 + PID 是否調過（2026-10-21 15:31 session，Claude）

- 使用者：Record 預設打開；控制時仍「overshoot 後 damping 回來」，問 PID 是否真的調過。
- Record：GUI 讀設定時一律把 Record 設為勾選（`load_cfg`）。
- trace（15:31，加速度 0.52）：乾淨的放手事件（52.8 / 60.3 / 39.6 s）放手時 15–31°/s → 手臂越過最後命令 12–14°（約 1.2 s 後達峰），再被 JTC（P 1.0、時間常數約 1 s）慢慢拉回，4 s 後仍差 1–4°。物理上與約 30°/s² 減速上限一致。另 4 秒窗內有幾次被下一次握持汙染（26–40° 的數字不可用）。
- PID 歷史（Igus_rebel，`controller.yaml` 最後 commit 2026-10-02，本機無未提交修改；`~/rebel_controller_tuning` 不在本機）：09-09 實機調參（p 4、ff 1.0/0.9，但當時 override 約 37%）→ 改 Override 100 後實機 25° 移動過衝 15.5°、震 6 s → 09-30 改為**模擬擬合**的 p 1.0 / ff 0.4，檔內註明「NOT yet run on the real arm」；09-29 MoveIt 實機也記錄「fast transit 收尾過衝」，處理方式是把 MoveIt 降到約 9°/s，而非重調增益。結論：本機執行的增益未經實機驗證；若 controller 負責人另有調好的值，未進到此 repo。
- teleop 修改：**release brake**（real-ros，`--no-release-brake` 關閉）：放手或追蹤遺失時，以量測關節速度（joint_states 差分 EMA，`arm.v_meas`）算自然停止點 `q + v|v|/(2a) + v·BRAKE_DELAY_S`（a = --max-joint-acc，BRAKE_DELAY_S 預設 0.25 s = 延遲 0.078 + 速度延遲 0.16，可用 `ROBOT_REMOTE_BRAKE_DELAY` 調），以減速所需時間送最後一點；若停止點過近另一臂則改為原地。假 ReBeL：放手後拉回 9.5° → 2.7°，最遠點 18.2° → 12.1°（delay 0.35：拉回 1.6°、最遠 12.8°）。新增測試。79 項測試通過（全部在 domain 97）。未在實機驗證。

## 錄製 resume 變慢 + 實機 J3/J5 追隨測試（2026-10-21 15:46–16:25，Claude）

- 錄製：15:46 session 的 episode 1 只錄到約 8 fps（223/224 幀 stale + late）。原因：`LeRobotDataset.resume()` 預設 `image_writer_threads=0`，接續既有資料集時 PNG 在 add_frame 內同步寫入（3 台 640×480 ≈ 120 ms/幀）。修正：resume 也傳 `image_writer_threads`；另把 recorder 的 ROS callback 改到背景執行緒。新測試 `tests/test_record_load.py`（3×640×480 雜訊影像、兩個 driver 200 Hz、新建 + 接續）：未修正時接續只有 26 幀/3 s；修正後兩者皆 ≥ 80 幀、無 stale / late。該 episode 1 資料不可用。
- 使用者授權（16:0x）：只在 home ↔ zero 之間、只動 J3 與 J5（直線方向）測實機。新工具 `real_robot/j3j5_step_test.py`：J1/2/4/6 命令恆為 0、J5 = 50/65·J3、測試段 J3 指令 [10,55]（快速組縮短 [20,45]/[25,40]），快速移動時 J3 超出 [3,62]、慢速 approach/return 超出 [-2,67]、J1/2/4/6 偏 > 2°、狀態逾時或有其他 publisher 即保持停止；以與 teleop 相同的方式串流（30 Hz 單點、加速度限制、leash）。先在 domain 97 假 ReBeL 乾跑：初版 fast 組在模型上衝到 68°（超出 home）→ 縮短範圍、收緊中止門檻、回 home 目標 64° 後再乾跑兩次（從 0 與從 home）皆在範圍內。
- 實機（左臂，使用者在急停旁，Claude 啟動 driver arms:=left，完成後關閉）：全部組別無中止，J1/2/4/6 最大偏差 0.02°，結束於 J3 64.0 / J5 49.2。結果（`latest_run/j3j5_left.csv`）：
  normal（30°/s、a 0.52、leash 10）：落後 8.5°、手臂峰值 32°/s、**過衝 1.3–1.7°、0.84–0.97 s 內進入 1°**；fast（45°/s、a 0.8、leash 14）：峰值仍約 32–34°/s、**過衝 8–9°、2.5 s 內未進入 1°**；faster（a 1.2）：過衝 9–10°；中途停（模擬放手）有 brake：過衝 6.7°、末端 1.1°；無 brake：15.4°、2.5 s 後仍差 4.3°。手臂實際加速度約 50–75°/s²（遠高於 controller.yaml 的 0.52 rad/s² 估計）。
- 結論與修改：瓶頸是控制迴路（ff 0.4 需約 8–10° 落後才能 30°/s，命令停時累積落後 + 延遲 → 過衝），不是加速度上限；提高速度/加速度只增加過衝。GUI 移除 fast 速度與 0.6/0.8 加速度選項；release brake 延遲 0.25 → **0.45 s**（實機停點比預測遠 6.7° @ 31°/s）。80 項測試通過。
- 給 controller 負責人（依實測的建議，需其驗證）：plant 增益約 1.26，ff_velocity_scale 0.4 只補約一半的速度，其餘靠 P 項的位置誤差；可試 ff 約 0.7–0.8（≈1/1.26）以減少串流時的落後與停止過衝；改動後可直接用 `real_robot/j3j5_step_test.py --arm left --profiles normal stop` 比較。

## 實機 PID / 放手煞車測試（2026-10-21 16:3x，Claude；使用者授權在 home↔zero 只動 J3/J5 自行測試）

- 方法：左臂 driver（Claude 啟停），增益以 `ros2 param set /left/rebel_arm_trajectory_controller gains.joint{3,5}.{p,ff_velocity_scale}` 執行期修改（不改 Igus_rebel 的 controller.yaml；driver 重啟即回原值，每次測完已改回 p 1.0 / ff 0.4），`real_robot/j3j5_step_test.py`（新增 `--out`、`--no-home`）；log `latest_run/pid_*.csv`。
- 結果（30°/s 上下移動）：A p1.0/ff0.4：落後 8.6–9°、過衝 1.7°、1.0 s 進 1°；B ff0.6：落後 6°、過衝 1.2–1.9°、0.6–0.9 s；C ff0.8：落後 5°、過衝 1.4–1.7°、0.6–0.8 s，但阻尼變差——上一段還在動時開始下一段會過衝 7.6°（觸發 62° 中止），放手煞車難校準（delay 0.15 時停在預測點後 12°）。
- 中途停（模擬放手）：無煞車 A/B 皆 16–20° 過衝並慢慢拉回；**A + 煞車 0.45 s：過衝 0.2°、無拉回（最好）**；B + 煞車 0.3 s：4.5°，0.45 s：3.6°（並有小幅回拉）。
- 中止 2 次（C 的兩次），皆為安全邊界觸發後原地保持，手臂始終在 home↔zero 線上（J1/2/4/6 0°）；結束時以 5 s 單點送回 J3 64 / J5 49.2 後關閉 driver。
- 結論：維持目前增益（p 1.0 / ff 0.4）+ 放手煞車 0.45 s（已是程式預設）。ff 0.6 可讓移動中更跟手（落後 9→6°）但放手停止較差；ff 0.8 不建議。若 controller 負責人改增益，需重新校準 `ROBOT_REMOTE_BRAKE_DELAY`（可用 `j3j5_step_test.py --profiles stop`）。

## 實機 PID / lookahead 調整（2026-10-21 16:4x–17:0x，Claude；使用者授權自行調 PID 與速度測試，範圍仍為 home↔zero 只動 J3/J5）

- 量測 command→arm 延遲（交叉相關時間位移）。短程（J3 20↔45，30°/s）：p1/ff0.4/la0.15 0.28 s、過衝 1.6°；la 0.08 0.14 s 但過衝 4.7°；p2/ff0.4 0.27 s；**p2/ff0.6 0.18 s、2.1–2.3°**；p1/ff0.6/la0.08 0.09 s 但 6–10°；p3/ff0.6 0.18 s、1.7–2.0°；p3/ff0.7 0.14 s、2.6–3.0°；p4/ff0.6 0.17 s、1.6–1.9°；p3/ff0.6/la0.12 0.13 s、3.2–4.1°。
- 長程（10↔55）：p4/ff0.6 在 10→55 過衝 7°+（起動時手臂約 0.3 s 幾乎不動、累積約 8° 落後，高 P 追回後過衝）→ 觸發 62° 中止；**p2/ff0.6：延遲 0.18 s、過衝 1.0–1.6°、0.47–0.64 s 進 1°**（基準 p1/ff0.4：0.28 s、1.5–1.8°、0.9–1.0 s）。
- p2/ff0.6 放手煞車：delay 0.2 → 7.6°、0.3 → 3.8°、0.45 → 4.1°（無煞車 19°）；基準增益 + 0.45：0.2°。
- 共 3 次安全中止（C ff0.8、p4 長程、…），皆停在 home↔zero 線上；一次中止後 J5 偏離線 5°，測試工具拒絕啟動，以 3 s 單點把 J5 拉回線上。每次測完增益改回 p1/ff0.4，結束送回 J3 64 / J5 49.2 並關閉 driver。
- 實作：`config/real_controller_gains.json`（standard p1/ff0.4 brake 0.45；responsive p2/ff0.6 brake 0.3）；`dual_arm_driver.launch.py gains:=standard|responsive|upstream`（在載入 controller.yaml 後覆寫我們的暫存參數檔，不改 Igus_rebel 原檔）；GUI Real 區「Arm gains」選單，teleop 依所選設定 `ROBOT_REMOTE_BRAKE_DELAY`。**預設 standard**：responsive 只在 J3/J5 測過，launch 會套用到六軸；J1/J2/J4/J6 需使用者授權另測。`real_robot/j3j5_step_test.py` 新增 `--lookahead`、`tune` profile（J3 20↔45）。80 項測試通過。

## 右臂 J2–J5 實機測試（2026-10-21 17:1x，Claude；使用者授權：只有右臂，J2 0..20、J4 -21..0、J3/J5 home↔zero）

- 新工具 `real_robot/joint_step_test.py`（單軸，其餘軸保持；只接受 J2/J4 且範圍不得超出授權；指令範圍內縮 6°、快速移動時超出「授權範圍內縮 0.5°」或其他軸偏 > 2° 即中止；回 0 時先 5°/s 到距 0 1°，再 2 s 單點到 0，避免越過 0）。假手臂乾跑時，內縮 4° 版本 J2 回 0 越過到 -1°、J4 衝到 -20.5°（中止）→ 改為上述後乾跑通過。
- 右臂（Claude 啟停 driver，standard 啟動，執行期切換 J2–J5 增益，測完改回）：全部無中止，其他軸最大 0.03°，結束在全 0。
  延遲 standard → responsive（p2/ff0.6）：J2 0.28 → 0.21 s、J4 0.28 → 0.20 s、J3/J5 0.28 → 0.18 s；responsive 過衝 J2 1.0–1.2°、J4 1.3–1.5°、J3/J5 1.8°（standard 1.2°），無振盪。放手：J2/J4 有煞車 ≤ 1.1°（無煞車 1.7–4.8°）；右臂 J3/J5 有煞車 7.5–8°（左臂 0.2–4°）、無煞車 15–19°。
- `config/real_controller_gains.json` responsive 加 `"joints": [joint2..joint5]`，launch 只覆寫這些軸（J1/J6 未測、維持原增益）。GUI 預設仍為 standard；左臂 J2/J4 未以 responsive 測過。

## 消除「過衝再拉回」：煞車減速 + 放手滑行（2026-10-21 17:3x–18:0x，Claude；先左臂後右臂，使用者要求優先右臂）

- 左臂（授權：J2 0→+20、J4 0→+20、J3/J5 home↔zero；開始時手臂被放在 J2 20 / J4 20，先以 6 s 單點回 0）J3/J5 短程（20↔45）：反應時間（命令開始動 → 手臂動 0.5°）約 0.27–0.30 s，與設定無關（手臂本身）。加速度 1.0：過衝 13–17°；送速度（JTC 用 velocities 當 ff）：更落後（頂到 leash）且過衝 7–10°；ff 0.7 / 0.8（p2）：過衝 3.3–4.2 / 5.5–6.5°，t90 只快 0.1 s。
- 右臂 J3/J5（responsive p2/ff0.6）：W1 目前（煞車 0.52）t90 1.50 s、過衝 1.6–1.7°（再拉回）；**W2 煞車減速 0.3：t90 1.73 s、過衝 0.3–0.4°、停住即在 1° 內（無拉回）**；0.2：0.2°、t90 1.93 s；加 D 0.1 無效；standard + 0.3：0.1–0.2°、t90 1.86 s。長程 10↔55（W2）：過衝 0.3°。
- **發現危險 bug**：舊放手煞車用 joint_states 差分速度（EMA），driver 在兩個 STATUS 間重複同值後跳變 → 速度尖峰數倍 → 停止點算到 30° 外。右臂一次放手被送到 J3 0°（命令 0.000），滑到 2.8° 觸發中止（仍在範圍內）。teleop 用同一估計，實際遙控也可能發生。
- teleop 修改：(1) `--max-joint-dec`（real-ros 預設 0.3）：`accel_step(..., adec=)` 讓命令接近目標時以較小減速度煞車；(2) 放手 / 追蹤遺失改為 **glide**：以命令自身速度（無量測雜訊）計算停止距離（以 --max-joint-acc 煞車、上限 GLIDE_MAX_DEG 15°、若太接近另一臂則原地保持），每拍繼續串流加速度限制命令直到停下；重新握持立即取消。實機（右臂，腳本同邏輯）：glide 放手後滑 19°、拉回 1.5°（無處理 19.9° 過衝）；假手臂：拉回 0.8°（無處理 11.1°）、移動過衝 0.9°。`BRAKE_DELAY_S` / `real_controller_gains.json` 的 brake_delay_s 不再使用（GUI 仍設環境變數，無影響）。80 項測試通過。
- 右臂結束在全 0、左臂在 J3 64 / J5 49.2，driver 皆已關閉。代價：每次移動收尾約多 0.2 s；放手後會再滑一段（減速距離），但不會再拉回。尚未以實際 Quest 遙控驗證手感。

## 網路查找 + 右臂控制方法測試（2026-10-21 18:0x–18:4x，Claude）

- 查找：ros2_controllers JTC 文件（`open_loop_control`：插值從上次命令起算、但同時關閉回授）、RobCo 串流文件（位置串流易過衝，建議高頻或位置+速度串流）、遙控延遲補償（Smith predictor / 手部預測）、Ruckig（jerk 限制即時軌跡生成）、AIRLab ros2-igus-rebel（同樣 JTC + velocity，無調參資訊）。
- `open_loop_control` 在本 driver 為唯讀參數（只能啟動時設），未實測；送速度（位置+速度串流）先前已實測更差。
- 右臂 J3/J5（授權範圍內）：**p2 / ff0.8 + 命令收尾減速 0.3** 最佳——短程 t90 1.56 s、過衝 0.1–0.2°；長程 10↔55 過衝 0.2–0.3°、立即在 1° 內、延遲 0.21 s；ff0.9 無再改善；收尾減速 0.4 過衝回到 1.7–2.6°；lookahead 0.05 過衝 9–11°。J2：過衝 1.0 → 0.1–0.2°；J4：1.3–1.5 → 0.8–1.0°。
- 放手滑行減速（p2/ff0.8）：0.52 → 滑 21.8°、拉回 5.3°；**0.4 → 滑 20.1°、拉回 0.4°**；0.3 → 滑 24.9°、拉回 0.5°。
- 舊的量測速度放手煞車在右臂又兩次把停止點算到 0°（觸發安全中止，皆在範圍內）：已從兩個測試工具移除（teleop 先前已改為 glide）。`j3j5_step_test.py` glide 目標未限制在測試範圍內導致迴圈卡住（手臂靜止保持），已修正。
- 預設更新：`real_controller_gains.json` responsive = p2 / ff0.8（J2–J5）；teleop `--glide-dec` 0.4（新參數）、`GLIDE_MAX_DEG` 20。GUI Arm gains 預設仍為 standard。左臂 ff0.8 只在無收尾減速時測過（過衝大），左臂 J2/J4 未以 responsive 實測。80 項測試通過。右臂結束在全 0，driver 已關閉。

## 放開即停：precise 速度 + 收尾檢查（2026-10-21 18:4x–19:1x，Claude）

- 右臂放手方式比較（J3/J5、p2/ff0.8）：@~35°/s 不論命令方式手臂都要約 20° / 1.2 s 才停（hold at measured：滑 21.2° 再拉回 20.8°；glide 1.5：19.4° / 12.3°；glide 0.4：23.8° / 3.2°）→ 手臂端減速約 30°/s²（加上約 0.3 s 反應）是瓶頸，應為 RobotControl / iRC 的 jog 加減速設定（driver 未設定），需使用者/controller 負責人決定是否調整。@~15°/s：glide 滑 6.3° / 0.71 s、拉回 0.2°。
- GUI Speed 選單：`precise`（--max-joint-vel 0.26、--max-lin-vel 0.12、--max-rot-vel 22，預設）/ `normal`（30°/s）；`config/launcher.json` 已設為 precise。18:58 實機雙臂 session（precise）：無 let go / stall / 外來 publisher / 碰撞暫停；放手後 TCP 再移動中位數 0.5–1.0 cm（最大 3.2）、拉回 ≤ 0.3 cm；握持中目標→手臂中位數 1.9–2.7 cm（p95 約 6 cm）。手快速移動時落後較多（速度上限）。
- 收尾檢查：(1) recorder 結束時背景 executor 未先停 → node 銷毀後 spin 執行緒拋 Traceback（資料已 finalize，無損）：改為先 `executor.shutdown()` 再 join，測試加「輸出不得有 Traceback」。(2) 放手那拍 `v_cmd` 歸零，glide 實際是「命令先停、再從靜止緩步移向停止點」；改成延續放手速度在假手臂反而拉回 2.8°（原 0.8°），且原行為已在實機驗證 → 保留原行為並寫明為刻意設計。80 項測試通過。
- commit 範圍：除 `dataset/`（實機錄製已覆蓋原本追蹤的範例資料集）、使用者自己的 `dual_rebel_task/`、`j6_swing_test.py` 外全部，含另一個 session 完成的安裝流程（`setup/install.sh` 等，README / 手冊已引用）。

## X 鍵改為切換旋轉座標系（2026-10-21 19:2x，Claude；使用者要求）

- 按鍵：A（右）= 開始／儲存 episode，B（右）= 丟棄，**X（左）= 切換 TOOL ↔ BASE 旋轉座標系**（沿用 `set_rot_frame`，握持中的手臂於放開再握時才套用），Y（左）= 重置。原本 A/X 都是錄製。GUI 收到 teleop 的 `rotation frame:` 確認即更新按鈕並寫入 `config/launcher.json`（X 切換也會被記住）。
- GUI：圖例 X 改用獨立顏色（teal）並新增說明，文字精簡讓整個圖例可見；Real 區原本一行過長（RViz / 回 home / 回 0° 被截掉），改為多行；視窗高度 820 → min(900, 螢幕高 − 80)，否則 Start 按鈕被擠出畫面（截圖確認）。
- 手冊：A / X / Y 說明、控制器圖（X `data-k="x"`、`--frame` 顏色、亮暗主題）、提示列縮短（原本被裁切）、Y 說明去掉矛盾的「Simulator only」；README 按鍵表、資料流同步；recorder 提示文字。
- 新測試：左手 X 兩次 → rot_frame tool → base → tool，且不送 toggle 事件。81 項測試通過；手冊與 GUI 以瀏覽器 / 截圖檢查。未用實際頭盔按 X 驗證。

## 實機 mocap 錄製（2026-10-20，Claude）

- 需求：使用者以 OptiTrack mocap（另一個套件 `rebel_mocap_follower`，原始碼唯讀參考）遙控實體右臂，要用與 Isaac 資料集相同的 14 維格式錄製。決定：action = mocap 原始目標（`/dual_arm/isaac_joint_commands`），不是 follower 濾波後的命令；鏡頭在錄製時用參數選擇。實體現場為 5 台相機（head、每臂 gripper 與 knife 各一），建議 key：`head, wrist_left_gripper, wrist_left_knife, wrist_right_gripper, wrist_right_knife`；格式 LeRobot v3.0、30 fps、AV1。
- 新增 `recording/record_dataset.py --source mocap-real`（預設仍是 `isaac`，Isaac 路徑行為不變）：`--arms`（預設 right）、`--state-topic/--state-names`、`--action-topic/--action-names`、`--follower-status-topic`、`--other-arm-deg`（未驅動手臂的 state 與 action 為定值）、`--camera KEY=TOPIC`（可重複、數量不限）、`--list-cameras`、`--pick-cameras`。夾爪 state/action 一律 0.0。以牆鐘（`time.monotonic`）取樣；staleness 以接收時間判斷（state／相機 max(2/fps, 0.1 s)，action 0.2 s）。相機支援 rgb8/bgr8/rgba8/bgra8/mono8/yuv422_yuy2(yuyv)/uyvy 與 CompressedImage（jpeg/png），尺寸不符時以 PIL 縮放；不支援的編碼在 `--list-cameras` 標示 UNSUPPORTED，錄製時報錯並丟棄該 episode。額外特徵 `observation.follower_state`（2,）：0 = 未驅動或 0.5 s 內無狀態，1 DISABLED、2 HOLD、3 CATCHUP、4 ENABLED、5 REFUSED、6 其他（碼值取自 follower_core.py）。`robot_type = igus_rebel_dual_arm_real_mocap`。
- **唯讀**：節點只訂閱（QoS best-effort，可同時接 reliable 與 best-effort 發布者），`start_parameter_services=False`，不建立 publisher／service／client（只有 rclpy 自動的 `/rosout`、`/parameter_events`）。
- LeRobot 版本相容：此 PC 無 Isaac 與 `.venv_record`，mocap 模式用系統 python3（3.10）+ `/opt/ros/humble`，user-site lerobot 0.4.4（無 `LeRobotDataset.resume`）。資料夾已存在而無 resume 時，明確訊息後以 exit code 2 結束（請換 `--name`）；`create()`／`resume()` 只在簽名接受時才傳 `vcodec`。
- 新增 `recording/run_record_mocap.sh`（`ROBOT_REMOTE_TARGET=real` → 預設 domain 11、Fast DDS、localhost-only 關閉；`HF_HUB_OFFLINE=1`；`LEROBOT_PY` 預設 python3）。README 新增「Record real-arm mocap episodes」。
- 驗證（皆為假發布者，domain 187 + `ROS_LOCALHOST_ONLY=1`，未連 domain 11、未用實體相機）：新增 `tests/test_record_mocap.py` 6 項全過（約 16 s）：以子程序執行錄製器錄 ~2 s，載入 LeRobotDataset 驗證 14 維名稱、右臂 state = 假 joint_states、左臂 = other-arm-deg、action = mocap 目標、夾爪 0、follower_state = [0, 4]、影像 3×480×640（1280×720 bgr8 與 320×240 jpeg 縮放後顏色正確）、影格數、robot_type；執行中以 ROS graph 確認錄製節點無 publisher（除 /rosout、/parameter_events）／service／client；`--list-cameras` 輸出；舊資料夾無 resume 訊息；YUYV/UYVY/mono8 解碼與 pick 互動。其餘既有測試：此 PC 無 `igus_ws`，`test_teleop_integration` 13 項因 `ros2 pkg prefix igus_rebel_description` 失敗（修改前同樣失敗，與本次無關）。
- 相機 preset（2026-10-20，使用者已更新 `~/igus_rebel_ros2/src/rebel_cameras/launch/cameras.launch.py`，topic 依手臂分組）：`--camera-preset cell`（5 台）／`cell-right`／`cell-left`（head + 該臂 gripper、knife）。對應（皆 JPEG）：head = `/camera/color/image_raw/compressed`（336，1920×1080，15 fps）、`wrist_{side}_knife` = `/arm_{side}/wrist_camera/color/image_raw/compressed`（305，1280×800，15 fps）、`wrist_{side}_gripper` = `/arm_{side}/gripper_camera/image_raw/compressed`（usb_cam raw_mjpeg，1280×720，30 fps）。`--camera` 可新增或覆寫。相機在本機 domain 11 執行（由 /proc 環境變數讀得，未訂閱 domain 11）。當時左 gripper（USB 埠 1-1.2）與 C930e knife_camera 未插上。
- 解碼成本（合成資料量測）：1080p rgb8 縮放 7.8 ms、1280×800 4.1 ms、720p JPEG 5.5 ms，5 台合計約 27 ms／33 ms 一拍。改為每台相機快取：同一則訊息只解碼一次（15 fps 相機每兩拍重用），新增單元測試。7 項測試通過。
- 實機相機靜態測試（2026-10-20 21:22–21:38，使用者要求）：當時 domain 11 沒有手臂 driver（`/joint_states` 無 publisher）、也沒有 mocap 串流，所以以測試程式在**私有 topic** `/rec_static_test/{joint_states,mocap_target}` 發布固定 home 姿態 (0,0,65,0,50,0)°（沒有任何機器人節點訂閱），錄製器訂閱 5 台實體相機。結果：
  - 原始 `color/image_raw`（3–6 MB/幀）在 domain 11 只到 5–10 Hz、間隔最長 1.9 s；`.../compressed`（100–330 KiB）為 15 Hz、間隔 ≤ 73 ms → preset 全部改用 compressed。
  - 舊版在取樣迴圈內解碼，5 台 JPEG 時 20.1 s 只錄 525 幀（64 late ticks；LeRobot 以幀序號 / fps 作時間戳，會讓時間軸被壓縮）。改為每台相機一條解碼執行緒（只解最新一幀），JPEG 以 PIL draft 縮小解碼：20.1 s 錄 602 幀、0 late ticks。相機 staleness 門檻改為 max(3/fps, 0.2 s)（15 fps 相機正常即有 0.07–0.11 s）。
  - 右夾爪相機（`/arm_right/gripper_camera`）30 Hz 但畫面幾乎全黑（平均 2.8/255、最大 12），相機本身輸出如此（直接訂閱確認），可能被遮住或曝光問題。
  - head（Gemini 336）驅動在測試中劣化：收到率 10 → 1.6 → 0.3 Hz，連 `camera_info` 都延遲約 10 s → 驅動本身卡住（非網路／錄製器）；需由使用者重啟 cameras.launch（預設含 USB reset）。最後一次 4 台相機測試以右 knife 代替 head key。
  - 存檔（AV1、依序編碼）20 s × 5 台需 30–125 s。測試資料集留在 `datasets/static_cam_test_20261020_*`（213238 為 0 episodes），log 在 `logs/static_cam_test_*.log`。實際手臂關節、mocap action、follower 狀態仍未在實機錄過。
- 使用者決定（2026-10-20）：實機 ROS domain 由 11 改為 **12**。`ros_env.sh` 的 `ROBOT_REMOTE_TARGET=real` 預設改為 12（`ROBOT_REMOTE_REAL_DOMAIN_ID` 仍可覆寫），所有實機啟動腳本（錄製、teleop real-ros 等）一起改變。相機、mocap follower、mocap PC 須由使用者自行改為 12（不在本專案內）。上面記錄的 domain 11 測試結果為改變前。
- **未驗證**：從未對真實 domain 11、真實 mocap PC、Orbbec／Yahboom 實體相機執行（相機 topic 名稱為猜測，用 `--list-cameras` 確認）；Isaac 路徑在此 PC 無法執行，只做語法檢查；lerobot 0.4.4 寫出的資料集尚未用 0.5.1 載入；AV1 編碼僅在假資料短片測過；長時間（>1 min）錄製的影格延遲與丟幀未量測（log 會顯示 late ticks）。

- 實機 MoveIt 腳本錄製（2026-10-22，使用者要求）
  - 需求：另一台 PC 跑 MoveIt 路點程式（開箱子），把關節命令送給本機的單臂 igus ReBeL ros2_control 堆疊（`/joint_states` joint1..6、`rebel_arm_trajectory_controller`）；本機只跑控制器，要以與 mocap 相同的 14 維格式、相同相機錄製。
  - 決定：新增 `record_dataset.py --source moveit-real`（選項 `isaac`／`mocap-real`／`moveit-real`；其餘行為沿用 `MocapRecorder`／`run_mocap`，以 `args.source` 區分 action 來源，不複製類別）。action = `control_msgs/JointTrajectoryControllerState` 的 setpoint（`--controller-state-topic`，預設 `/rebel_arm_trajectory_controller/controller_state`，`{side}` 會代換、每個不同 topic 訂閱一次，以 joint 名稱選臂）：`reference.positions` 長度等於 `joint_names` 時用之，否則（舊版 JTC）用 `desired.positions`，兩者皆無則忽略該訊息。`--action-names` 預設依來源填入（moveit-real `joint{i}`、mocap-real `arm_{side}_joint{i}`）。`--action-topic`／`--follower-status-topic` 為 mocap 專用，與 moveit-real 併用會 error；`--controller-state-topic` 與其他來源併用也 error。
  - 只用手動 Enter／`d`／`q` 控制；沒有 `observation.follower_state`；`robot_type = igus_rebel_dual_arm_real_moveit`；夾爪 0.0、未驅動手臂 = `--other-arm-deg`、action staleness 0.2 s。節點名稱 `dataset_recorder_moveit`，仍為唯讀（best-effort 訂閱、無 publisher／service／client）。無 setpoint 時 "cannot start" 訊息會列出 topic 並說明找不到 setpoint。
  - 檔案：`recording/record_dataset.py`（`controller_topics()`、`apply_source_defaults()`、`MocapRecorder.on_controller_state()`）、新 `recording/run_record_moveit.sh`（同 run_record_mocap.sh，`ROBOT_REMOTE_TARGET=real` 預設 domain 12）、新 `tests/test_record_moveit.py`、README「Record real-arm MoveIt / scripted episodes」。
  - 驗證（僅假發布者，測試 domain 187 + localhost only）：`python3 -m unittest discover -s tests -p 'test_record_m*.py'` 共 14 項通過（原 7 項 mocap + 7 項新增）：實際子程序錄 ~2 s，action[:,7:13] = REF（假 JTC 的 `desired` 刻意不同）、state = `/joint_states`、左臂 = other-arm-deg、夾爪 0、無 follower_state 特徵、robot_type、影像 shape／顏色；錄製期間 ROS graph 唯讀檢查；reference 空時回退 `desired`、兩者皆無時忽略；雙臂 `{side}` topic 與單 topic 以 joint 名選臂；選項互斥 error。`bash -n` 兩個 script、`git diff --check` 通過。
  - **未驗證**：未對真實控制器、真實 domain（12）、另一台 PC 的 MoveIt 程式執行；真實 JTC 的 `controller_state` 發布頻率／QoS 與 `reference` 欄位內容（尤其 reference 在軌跡結束後是否持續更新）未實測；`ros-humble-joint-trajectory-controller 2.54.1` 的欄位是依訊息定義推論，未在實機 topic 上看過。

## 合併 upstream d604e38 後的狀態（2026-10-26，Claude）

- `git pull --ff-only` 把 HEAD 帶到 78a48dd（upstream 的 `--source real` + 4 台 USB 相機）；本機未提交的 mocap／moveit 錄製（上面兩節）以 stash pop 合回，conflict 在 `record_dataset.py`、README、AGENTS。
- `--source` 現為 `sim`（別名 `isaac`，預設）｜`real`（upstream，不變）｜`mocap-real`｜`moveit-real`。`make_dataset()` 共用：資料夾判斷改用 `dataset_check`（new／empty_dir／shell／dataset／incomplete／not_dataset），續錄走 `check_resume()`（features／robot_type／fps 不符就拒絕）；LeRobot 無 `resume`（0.4.x）時 `check_resumable()` 在開 ROS 前就報錯。`image_writer_threads` 預設 `max(4, 2×相機數)`（real、mocap-real、moveit-real）；mocap 路徑的 callback 本來就在背景 executor，現在結束時也 `executor.shutdown(1 s)` + join。預設 root 為 `robot_remote/dataset`（上面「datasets/」的敘述已過時）。Isaac 與 upstream real 路徑的行為不變。

## 切割過程錄製（2026-10-26，Claude）

- 需求：錄 `~/igus_rebel_ros2` 的切割執行（雙臂 `dual_cut_coordinator`；單臂 `box_h_cut_executor` T-cut 序列），一次 execute = 一個 episode。設計由主 session 決定：建在 `--source moveit-real` 上（action = JTC `reference`，state = `/joint_states`，相機走 ROS topic），新增 `--episode-trigger {manual,dual-cut,h-cut}`（預設 manual = 原行為）。
- 規則（程式在新檔 `recording/cut_trigger.py`，純標準函式庫，可單獨測）：dual-cut 在 `/dual_cut_coordinator/status` 第一次進入 WAITING_BARRIER／EXECUTING（前一則為非執行狀態）時開始，跨 A、barrier、B 全程一個 episode；STOPPED 或 `stop_latched` → 丟棄（`--keep-stopped`：存檔，task 加 ` [stopped: <stop_reason>]`）；`PLANNED` 且至少一個 `exec_report` 的 phase `completed=true` → 存檔；`PLANNED` 但沒有完成的 phase（barrier／goal 失敗）、或回到 IDLE／PLANNING → `aborted`（同樣丟棄，`--keep-stopped` 時加 ` [aborted: ...]`）。h-cut 在 `t_cut_status` 的 `{0,"sequence","running"}` 開始，`{"sequence","done"}` 存檔，任何 `"failed"` 丟棄。status 超過 2 s 沒有訊息只印 log，不結束 episode。錄製器啟動時若執行已在進行不加入，等下一次。
- 額外特徵 `observation.cut_progress`（float32, 2，僅 trigger 模式）：dual = [coordinator state 0..5, phase 0/1(A)/2(B)]；h-cut = [step 0..5, status 0 none/1 running/2 done/3 failed]。robot_type：`igus_rebel_dual_arm_real_cut`／`igus_rebel_single_arm_real_cut`。預設 task `cut the carton tape`。Enter／d／q 在所有模式都是覆寫（手動開始的 episode 只由手動結束）。`--max-episode-s` 在 trigger 模式預設 600。
- 新增 `recording/run_record_cut.sh`（預設 domain 11、Fast DDS、localhost-only 關閉，會印出；`CUT_DOMAIN_ID`／`CUT_LOCALHOST_ONLY`／`IGUS_WS` 可覆寫；雙臂模式才 source igus_rebel_ros2 的 install 以取得 `dual_cut_msgs`，且在 record_dataset.py 內延遲 import）。`--single-arm` = h-cut，`--arms` 決定單臂資料放哪一側（預設 right）。相機 preset `cut` = head + 左右 gripper（cameras.launch.py 預設開的）；新增 `knife` 鍵（`/knife_camera/image_raw/compressed`）；305 腕部相機用 `--camera` 加。錄製器仍然唯讀（測試檢查 publisher 只有 rclpy 自動的、無 service／client）。
- CRI MoveJoint 轉移（opt-in，預設關）：該臂 JTC 停用、`controller_state` 停止 → action 變舊；只計數（存檔行 "N with a stale action"），不捏造 action。
- 驗證：`tests/test_record_cut.py`（單元：trigger 規則、多 phase = 一個 episode、STOPPED、stop_latched、grace、靜默、h-cut；選項；整合：假發布者 + 真正的 CLI，domain 97：存檔／丟棄／`--keep-stopped` task 後綴／read-only 圖／單臂名稱對應／JTC 沉默計數）。Mock 端對端（domain 213，雙臂 mock stack + 假相機 + `run_record_cut.sh`）：兩次完整 A+B 執行各存成一個 episode（1990／2002 幀 @15 fps = 133 s，cut_progress 依序 WAITING_BARRIER(A) → EXECUTING(A) → WAITING_BARRIER(B) → EXECUTING(B)，action = JTC reference，|action−state| 平均 0.0003 rad），第三次中途 `~/stop` → 411 幀被丟棄；證據在 `~/igus_rebel_ros2/scan_results/cut_recorder_20261026/`。
- **未驗證**：真實機器人／真實 domain 11、真實相機、真實 `t_cut_status`（單臂 stack 沒有在 mock 跑）、CRI 轉移、`--keep-stopped` 與 `--single-arm` 的 mock 端對端（只有假發布者的整合測試）；phase B 的 barrier 失敗但 A 已完成時會被視為正常結束（coordinator 沒有失敗狀態可讀）；存檔（約 1 分鐘／130 s）期間不能開始下一個 episode（會延後開始）。LeRobot 0.4.4 以外的版本沒測 `episode_buffer["task"]` 改寫（`--keep-stopped` 的後綴；改寫失敗時警告並存原 task）。

## 切割錄製器自動啟動：原始 rosbag2（2026-10-26，Claude；使用者要求取代 LeRobot 輸出）

- 需求變更：自動錄製器要存**全解析度原始資料**，LeRobot 轉換使用者之後自己寫。所以 launch 預設後端 `raw` = 新檔 `recording/record_raw_cut.py`（+ `run_record_raw_cut.sh`）：每次執行一個 rosbag2（storage `sqlite3`，mcap 沒裝也不裝），由錄製器自己用 `rosbag2_py.SequentialWriter` 寫入序列化訊息（`create_subscription(..., raw=True)`；原始 bytes 與型別不變，bag 時間戳 = 接收時間，訊息內的 header stamp 不動）。無 LeRobot、無 numpy、不解碼影像，系統 python3 + `/opt/ros/humble` 即可；`video` 後端 = 舊 `tools/cut_recorder`（robot_remote 或 `rosbag2_py` 不存在時自動退回）。LeRobot 版（`record_dataset.py` 的 cut 模式、`--headless`、`--camera-domain`、sidecar、`fjt-status`）保留為手動工具，launch 不再使用。
- 錄的 topic：全部相機（`.../compressed` 原樣、不縮放、不重新編碼）、`/joint_states`、`/tf`、`/tf_static`（開檔時寫入全部）、兩臂與單臂的 `controller_state`、`/dual_cut_coordinator/{status,exec_report,plan_report}`（開檔時寫最新的 plan_report）、`t_cut_status`、FJT action status、`/side_cut_vision/` 底下所有 topic（overlay、measurement）、`--extra-topic`。只錄存在的 topic。`--camera-domain N`：相機 topic 由第二個 rclpy Context 讀進同一個 bag。
- 一個 episode = 與先前相同的 trigger（dual-cut／h-cut／fjt-status，fjt 另有 `--fjt-hold-s` 5 s）加 `--pre-roll-s`（預設 2，環形緩衝）與 `--post-roll-s`（預設 2）。finished 保留；stopped／aborted **也保留**並在 `clip.json` 標 outcome／stop_reason（`--discard-stopped` 才刪）；SIGINT／SIGTERM 立即關閉開啟中的 bag（outcome `shutdown`，保留），數秒內 exit 0。
- 資料夾 = side_cut_vision 的 sidecar 資料夾 `<clip-dir>/<YYYYmmdd_HHMMSS>/{clip.json, snapshot, bag/}`；`clip.json` 開啟時 `recording:true`（每 20 s 重寫），結束 `false` + outcome、reason、stop_reason、exec_reports、各 topic 訊息數／位元組／頻率／最大間隔、各相機是否有錄到、丟棄的相機訊息數。`<clip-dir>/index.jsonl` 每個 episode 一行（給轉換程式）。clip-dir 預設與 tools/cut_recorder 相同（外接碟 `igus_nvidia/cut_videos` 已掛載時，否則 `Igus_rebel/outputs/cut_videos`）；外接碟未掛載時啟動即大聲警告（`/` 已 86 % 滿；全相機約 15–20 MB/s）；可用空間低於 `--min-free-gb`（5）不開始 episode。
- 寫入不阻塞訂閱：queue + 寫入執行緒；超過 `--queue-mb`（512）先丟最舊的**相機**訊息（計入 clip.json），joint／status／tf 絕不丟。沒有出圖的相機直接不在 bag 裡（出現後才會錄），10 s 後 log 並在 clip.json 標出。
- 測試：`tests/test_record_raw_cut.py`（11 項：bag 可用 `rosbag2_py.SequentialReader` 讀、所有 topic／型別都在、訊息數與 clip.json 一致、相機 bytes 與發布的 JPEG 完全相同、兩個 domain 合併進同一個 bag、pre／post roll、sidecar 生命週期與唯讀圖、stopped 保留與 `--discard-stopped`、headless SIGINT 乾淨關閉、缺相機與慢磁碟只丟相機訊息、fjt-status 單臂、選項與無 lerobot／PIL／cv2 import）。igus_rebel_ros2 端的 launch 測試與 mock 端對端見該 repo `scan_results/cut_recorder_20261026/launch_e2e/`。

## 控制器端笛卡爾點動：`--backend cri-jog`（2026-10-30，Claude；僅以假 driver 測試，未接實體）

- 需求：Quest 經 igus_rebel_ros2 driver 的 Cartesian jog（`output:=cri` 那條路）驅動實體 ReBeL，由控制器自己做 IK，本機不跑 IK。新檔 `quest_teleop/cri_jog.py`（邏輯全在這裡）、`quest_teleop/run_quest_cri_jog.sh`（`IGUS_REBEL_INSTALL` 指向有 `igus_rebel_msgs` 的 install）、`tests/test_cri_jog_teleop.py`；`quest_dual_arm_teleop.py` 只加 backend 選項、參數註冊與 main 的早期分支（其他 backend 行為不變，`igus_rebel_msgs` 只在 cri-jog 啟動時才 import）。
- 控制律（只有平移）：握持時記下手與機器人位置（`robot_cart_pose.pos_cart_robot_raw`，**假設單位 mm**，`--cri-pose-units`）；目標 = 握持時機器人位置 + `--scale` × 手位移（同 IK 路徑的 `--yaw-deg` 映射）× `--cri-signs`（預設 1,1,1；控制器底座座標軸是否與 URDF base_link 一致**尚未在實機核對**）；v = kp·(目標 − 目前)，限 `--cri-max-lin-vel`（預設 0.03 m/s）、死區 1 mm、目標離握持點最遠 `--cri-max-offset` 0.30 m；以 TwistStamped（frame_id 空、stamp = now()、angular = 0）≥ 20 Hz 發到 `/igus_rebel/cart_jog_cmd`。**driver 的 % ↔ m/s 對應（200 %/(m/s)）是未量測的預設值；A/B/C 軸意義未量測，旋轉未實作（TODO 在 cri_jog.py），X 鍵無作用。**
- 只支援單臂（`--arms left|right`，兩臂或不指定會拒絕）。啟動：`cart_jog_enable true`（被拒就印出 driver 訊息、結束碼 2；`--cri-no-auto-enable` 略過）。結束（Ctrl+C / SIGTERM / 例外）：先送零速度再 `cart_jog_enable false`。不做 home / park；只有握把按住時機器人才會動；Y 只送 discard 事件。`--dry-run`：只計算與寫 trace，不發布、不呼叫服務。
- 立即送零並要求重新握持：放開、tracking 遺失、機器人位置超過 0.2 s、driver 離開點動模式（`hardware_state`）、KINSTATE ≠ 0、`cart_jog_cmd` 上出現其他 publisher（啟動時有就拒絕）、手臂往命令的反方向動（軸向／符號錯，**永久鎖住直到重啟**）或不跟隨命令（馬達未啟用、% 對應太低）。driver 離開模式後，放開再握會重新 `cart_jog_enable true`，之後還要再放開握一次才會動。
- `/quest_teleop/action` 在此 backend **不發布**（沒有關節命令，也不捏造）；操作者命令請看 `/igus_rebel/cart_jog_cmd` 與 `latest_run/teleop_trace.csv`（cri-jog 欄位：robot / target 位置、twist、jog 狀態、位置資料年齡）。viz 用的 `/quest_teleop/{side}/hand`、`/state`、`/events`、`/joint_states`（取自 robot_cart_pose 的量測關節）照常發布。
- 驗證：假 driver（SetBool 服務可拒絕、檢查 stamp 與 frame、0.15 s 看門狗、10 % 限幅、一階延遲 plant）測試收斂到映射目標且速度 ≤ 上限、放開／tracking 遺失一拍內歸零、enable 被拒、SIGINT/SIGTERM／例外先零再 disable、位置過期、driver 離開模式、外來 publisher（啟動／執行中）、反向／不跟隨、dry run、雙臂拒絕、按鍵；並以改壞程式的方式確認各安全測試會失敗。**未接實體 driver、未量測 % 對應、未在實機驗證方向。**
- 第一次實機測試：e-stop 在手邊、`--dry-run` 先看 trace 的 target/robot、再用 `--cri-max-lin-vel 0.01` 小幅、單軸慢慢握持；先檢查「手往前→手臂往前」與 trace 中 `robot_*` 變化方向和 `vx/vy/vz` 同向（不同向代表 `--cri-signs` 或 driver 的 `cart_jog_signs` 要調）；必要時先由使用者授權，才量測 % ↔ m/s。

## cri-jog 擴充為雙臂 + 臂間防撞（2026-10-30，Claude；僅以假 driver 測試，未接實體）

- `--backend cri-jog` 現在支援 `--arms left right`（預設就是雙臂；單臂照舊）。topic 預設：單臂 `/igus_rebel/...`，雙臂 `/igus_rebel_{side}/...`（dual stack 每臂一個 driver 實例：`cart_jog_enable`／`cart_jog_cmd`／`hardware_state`／`robot_cart_pose`，需先停用 `arm_{side}_trajectory_controller`）；`--cri-*` 選項加 `{side}` 可覆寫，兩臂解析後名稱相同會拒絕。手→臂對應同 IK 路徑（含 `--swap-hands`）；每臂各自的離合器、follow monitor、重新握持狀態；cmd topic 的外來 publisher 檢查逐臂進行。
- 進入／離開：每臂各自 `cart_jog_enable true`；任一臂被拒就把已進入的臂退出，印出每個 driver 的訊息並以結束碼 2 離開（提示停用 `arm_{side}_trajectory_controller`）。結束時對所有臂送零速度，再對本次進入過的每臂 `enable false`。
- 故障政策：一臂故障 → 兩臂都送零並要求重新握持（與 IK 路徑一致）：位置過期、driver latched／斷線／hardware_state 過期、KINSTATE ≠ 0、外來 publisher、**driver 自行離開點動模式（我選「兩臂都停」：多半是 latch／故障，另一臂單獨繼續動沒有意義也危險）**、手臂往命令反方向動（該臂永久鎖住到重啟，另一臂放開後可重新握持）、防撞不知道某臂位置。單手放開／tracking 遺失只影響那一臂；「不跟隨」只放開該臂。
- 臂間防撞（`--no-arm-guard` 可關閉並大聲警告；預設開）：用 `arm_collision.py` 的膠囊模型（`--arm-radius`／`--tool-radius`／`--min-arm-dist` 0.05 m 表面間距、`config/arm_calibration.json` 底座位置）與 URDF 的 FK（來自已 source 的 `igus_rebel_description`；沒有 URDF 就拒絕啟動，除非 `--no-arm-guard`），關節取自各 driver 的 `robot_cart_pose.joint_position_rad`。**這是反應式的**（下一刻的關節未知）：間距 ≤ `min_arm_dist + margin` 時，若某臂的命令速度沿最近點方向朝向另一臂（最近點在法蘭／夾爪／刀尖，角速度為 0 時它們與命令的 TCP 同速），或量測間距以 > 2 mm/s 縮小（前臂／手肘或軸向鏡像的後盾），該臂送零速並 log「arm guard」，需重新握持，遠離方向允許。margin = 驅動臂數 × `--cri-max-lin-vel` × (0.2 s 位置逾時 + 1 tick + 0.15 s driver 停止) ≈ 2.3 cm（雙臂 3 cm/s）。選命令方向而非前後 tick 比較作為主規則，因為重新握持後可立即拒絕靠近而不會一階一階蠕動進去；兩條規則都只移除靠近分量，不會把臂推開，所以不會來回震盪。任一臂位置過期或缺關節 → 防撞「看不見」→ 兩臂停。單臂時另一臂假設在 `--other-arm-deg`（預設全 0），或 `--watch-other` 唯讀訂閱 `/igus_rebel_{side}/robot_cart_pose`。假設控制器底座軸與 URDF `base_link` 一致（`--cri-signs` 只改手的對應，不影響防撞）。**支架、桌面、相機、工具超出膠囊的部分都未建模；實際反應時間與 margin 未在實機量測。**
- trace：每臂每 tick 一列，新增 `clearance_mm`、`guard`（單臂格式只多這兩欄）。
- 驗證：假 driver（每臂一個，位置→關節用真實 SerialChain IK，只在 fake 內）：雙臂各跟自己的手、單手放開另一臂繼續、五種故障（位置過期、KINSTATE、latched、離開模式、外來 publisher）皆兩臂停、一臂拒絕 enable 時另一臂退出且結束碼 2、Ctrl+C 兩臂先零再 disable、雙手互相靠近時停在 ≥ min_arm_dist − 6 mm（用 fake 自己的關節獨立算間距，不是 trace）且遠離可動、單臂對假設／監看的另一臂、無 URDF 拒絕、重複 topic 拒絕；關掉防撞後同樣動作會穿過（測試非空洞），並對防撞做了突變檢查。
- 雙臂第一次實測：driver 以 dual stack 啟動並停用兩個控制器、e-stop 在手；先 `--arms left right --dry-run` 看 trace 的 `clearance_mm`；再用 `--cri-max-lin-vel 0.01`、兩臂相距遠、慢慢握持；先單臂（另一臂 `--other-arm-deg` 設成實際姿態）再雙臂；量測 %↔m/s 與實際停止距離後再調 margin。

## 無 Isaac Sim 6.1 的 PC 也能實機錄製（2026-11-03，Claude；使用者要求）

- 本機（farhan-MS-7D90）只有 Isaac Sim 5.1、無 `.venv_record`、磁碟 98 %，GUI 實機錄製啟動失敗（找不到 LeRobot python）。
- `recording/run_record.sh`：找不到 `$ISAAC_SIM/exts/isaacsim.ros2.core/humble/rclpy`（或 `ROBOT_REMOTE_RECORD_PY=system`）時改用系統 python3（`LEROBOT_PY` 可覆寫，須為 3.10）+ `/opt/ros/humble`，domain 設定沿用 ros_env.sh。本機 LeRobot 0.4.4 無 `resume`：既有資料集不能接續，每次 session 換新名稱。
- GUI：`recorder_cannot_append()`（每個 GUI session 查一次 lerobot 版本）在 Save location 提示並擋下 Start（目標為既有資料集時）。
- 測試：`test_real_recording` / `test_record_load` / `test_dataset_location` 依 `CAN_RESUME` 判斷接續步驟（0.4.x 時預期被拒）；`LEROBOT_PY=python3` 跑 recorder 相關 23 項全過（domain 86–88 / 97，未碰 domain 42 上正在跑的實機）。實體相機 + 實際 driver 的 0.4.4 錄製尚待使用者以 GUI 跑一次確認。
- `dataset/dual_rebel_task`（repo 追蹤的範例）缺 `meta/episodes/`（從未進 git，`.gitignore` 排除 `dataset/`），新 clone 一律被判為 incomplete；是否從 repo 移除待使用者決定。

## 錄製改為邊錄邊編碼 + 關鍵幀間隔 30（2026-11-03，Claude；使用者要求）

- 問題：100 s × 3 台相機的 episode 存檔約 1 分鐘、暫存 PNG 2.7 GB（本機只剩 13 GB）；影片 53–77 MB／台／100 s。量測（使用者的 head 影片重新編碼）：AV1 編碼本身只要約 5 s，慢的是 PNG 寫入／讀回；檔案大是因 LeRobot 寫死 g=2（每 2 幀一個關鍵幀）——g=30 約小 9 倍（77.6 → 8.8 MB）。GPU（NVENC）可用但效益小，未採用。
- `record_dataset.py`（所有 source 共用 `make_dataset`）：LeRobot 支援時預設 `streaming_encoding=True`（`--no-streaming-encode` 關閉、`--encoder-queue` 90 幀）；`--video-gop` 預設 30（0 = LeRobot 的 2），設在 streaming encoder 上，PNG 路徑則包裝 `lerobot_dataset.encode_video_frames` 補上 g。`save_episode()`：streaming encoder 若因佇列滿而丟幀，該 episode **丟棄**（影片會與 state/action 不同步）並印 ERROR。
- 代價：每個 episode 開頭 3 個編碼器啟動約 70 ms → 約 2–3 個 late ticks（只在開頭）；`test_record_load` 改為容許 ≤ 3。
- 驗證：recorder 相關 23 項 + 新 `tests/test_record_video.py` 4 項通過（LEROBOT_PY=python3、LeRobot 0.4.4）；`test_real_recording` 檢查無 PNG、影片幀數 = 資料列數、關鍵幀 0/30。**實體相機**（head + 兩腕，假手臂 topic，domain 229）錄 20 s：598 幀、三路影片皆 598 幀、關鍵幀每 30、存檔 0.3 s、head 2.8 MB／20 s、無 PNG。實際 GUI + 實機 driver 的錄製尚待使用者跑一次。LeRobot 0.5.x（其他 PC）上的 streaming 參數未測（依簽名判斷才使用）。
- 本次錄製發現：右腕相機（USB 1-2.2）在 04:47:56 從 USB 斷線重連，recorder 不會重新開啟相機，之後的幀全部 skipped 直到 Stop all（約 78 s 未錄）；可能是線材被手臂拉扯。

## 2026-11-03 實機 log 檢查（10:48–12:23，Claude）

- **teleop 崩潰（5 次，10:53 / 11:03 / 11:56 / 12:10 / 12:20）**：`ValueError: Requested logging filters cannot be changed between calls`，觸發點皆為兩臂碰撞 guard 的 throttled 警告「arms X cm apart - holding」。原因：`TracingLog` 讓所有訊息共用 rclpy 的同一個呼叫位置，同一位置先後有／無 `throttle_duration_sec` 時 rclpy 丟例外。崩潰後 finally 會讓兩臂回 0°，錄製器則繼續錄（12:20 的 `dual_rebel_task_farhan7` 最後約 24 s action 凍結、726 幀 stale，且包含回 0° 的動作）。修正：`TracingLog._throttled()` 自行依呼叫者 (檔案, 行) 節流，不再把 filter 傳給 rclpy；新測試 `tests/test_tracing_log.py`（舊程式重現同一個 ValueError）。teleop 相關 55 項（cri-jog 34 項因本機無 igus_rebel_msgs 略過）+ real_ros_tracking 15 項通過。未在實機重跑。
- **殘留 driver**：11:20 的 GUI session（右臂 J5 在 11:22:55 過電流 3.38 A > 3.2 A → `_MNE_OC`，回 0° 途中 GUI 被關閉）留下 `ros2 launch dual_arm_driver.launch.py`（PID 2334148，子程序 2334170 /left、2334178 /right，domain 42 localhost），之後每次 Start 都有兩組同名 `/left|right/controller_manager`：spawner 隨機失敗（Controller already loaded / Failed to configure / Failed loading），命令與 joint_states 可能走舊 driver → 11 次「the arms did not reach the home pose」、多次「arm does not follow」。另有殘留 viz robot_state_publisher（11:21、12:18）。代理不得自行 kill（非本代理啟動），由使用者處理。
- 待做（建議）：driver 啟動前檢查同 domain 是否已有 `/left|right/controller_manager`，有就拒絕；teleop 啟動時 `/<side>/joint_states` publisher > 1 即拒絕；錄製器 action stale 超過數秒自動丟棄／結束 episode。

## 實機控制器更新：Igus_rebel jtc-tracking（2026-11-03，Claude；使用者要求採用另一個 thread 的新控制器）

- 來源：「Dual Arm and Static Camera Calibration」thread（igus_rebel_ros2，2026-10-30～31）在兩台實機驗證：driver 新參數 `jog_velocity_scale` 1.65（舊 driver 寫死 2.0；實測每個關節、兩臂皆 0.600 °/s per jog %，1.65 誤差 < 1 %；舊 2.0 讓手臂比命令快約 20 %，即先前擬合的 plant gain 1.26）、JTC 增益 p 3 / ff 0.6（S0–S7 ladder：無過衝、終點 0.000°、0.08–0.2 s 收斂；以 MoveIt 式規劃軌跡測試）、規劃加速度上限 0.4 rad/s²；手臂加速約 0.55 rad/s²、jog 歸零時煞車約 1.45–1.75 rad/s²。commit `287b0186`（GitHub fani8j/Igus_rebel 分支 `fani8j/jtc-tracking`）。
- 使用者同意後停止 11:20 殘留的 driver（PID 2334148，SIGINT，正常結束）與 8 個殘留 viz robot_state_publisher。
- `igus_ws/src/Igus_rebel` 由 cc0e2b6 fast-forward 到 287b0186（stash → ff → pop，保留本機兩個修改：`controller_mock.yaml` 的 open_loop_control、SRDF home/zero），`colcon build --packages-up-to igus_rebel igus_rebel_moveit_config` 成功；`setup/install_ros_ws.sh` 的 pin 改為該分支與 commit。未合併 hcut-path-consistency 的 dual-arm fault-group commits（本專案每臂獨立 driver，teleop 已在單臂故障時停兩臂）。
- `config/real_controller_gains.json` 新增 `tuned`（p3/ff0.6 全六軸 + `jog_velocity_scale` 1.65），`standard`／`responsive` 加 `jog_velocity_scale` 2.0（測量時的比例，行為不變）。`dual_arm_driver.launch.py`：`gains:=` 預設 `tuned`，依 profile 把 `jog_velocity_scale:=` 傳給 xacro（`upstream` = driver 預設 1.65 + controller.yaml），log 印出 gains 與 scale。GUI 預設 `tuned`；舊 launcher.json 的 `standard` 一次性改為 `tuned`（`gains_v2` 旗標，之後手動選 standard 會保留）。
- 驗證：以 127.0.0.1（無機器人，domain 97）啟動 launch 讀回產生的參數檔：tuned = p3/ff0.6、scale 1.65（driver log「jog_velocity_scale 1.6500」）；standard = p1/ff0.4、scale 2.0。
- teleop：`tests/test_real_ros_tracking.py` 的假手臂改為可選 `PLANTS`（預設 `tuned`：gain 0.99、加速 0.55／煞車 1.5；`old`：舊擬合），`ROBOT_REMOTE_FAKE_PLANT` 可切換。teleop 預設值在 tuned 假手臂上：0.3 m 橫掃過衝 0.9°、放手回拉 1.1°、滑行 13.7°、快速來回落差 7.5°（old 9.1°），皆無 let go → **teleop 預設不變**。比較測試（加速度限制、放手滑行的有／無）改在 `old` 上跑（tuned 控制器本身已很少過衝），另加 tuned 的絕對值測試。假手臂試算（未採用，待實機）：`--glide-dec 0.8` 滑行 13.7 → 9.7°、回拉 0.9°；`--max-joint-dec 0.5` t90 快 16 % 但過衝 1.6°。
- **未驗證**：p3/ff0.6 從未以本 teleop 的 30 Hz 單點串流命令在實機跑過（先前 scale 2.0 時 p2/ff0.6 串流正常、p4 長程過衝 7°+）。下一步（需使用者在急停旁授權）：`real_robot/j3j5_step_test.py`（home↔zero、只動 J3/J5）以 tuned 跑 normal + stop profile，再短時間 Quest 遙控看 `latest_run/teleop_trace.csv` 的 lag / 放手滑行；若放手滑行過長再試 `--glide-dec 0.8`。

## 實機 Y 需按住 1 s + 錄製時相機掉線自動重開（2026-11-03，builder ×2 實作，Claude 審查）

- 背景（今日 log）：21 次自動回 home 中 18 次為左手 Y（另 3 次為 GUI「Arms home」按鈕），共丟棄 5 個 episode（7735 幀）；右腕相機 USB 1-2.2 今日斷線重連 5 次、1-2.3 一次，recorder 不會重開，之後每拍「frame skipped」，episode 在掉線處被截斷（farhan6：錄約 85 s 只存 44 s）。另 5 次 teleop 崩潰後回 0°（logging 修正，見上）。
- teleop：`--y-hold-s`（real-ros／real／cri-jog 預設 1.0，isaac 0 = 原本的立即觸發）。`cri_jog.YHold`：Y 上升沿且無其他按鈕時開始計時，按滿才觸發（不是放開時）；提早放開只記錄提示；計時中新按其他按鈕或新握握把即取消；觸發後仍按住的 Y 須放開再按。GUI 按鈕／stdin `reset_scene` 仍立即。GUI 圖例、手冊、README 已更新。
- recorder（`--source real`）：`real_cameras.RealCamera` 掉線時執行緒不結束，每 0.5 s 以原 sysfs 身分（ARC usb_port、Orbbec serial+interface）重找並以原格式重開，log `[cam] X lost` / `reconnected at /dev/videoN`。`--camera-gap-s`（0.5）：episode 中任一相機超過此時間無新幀 → 在掉線處結束 episode（≥ 1 s 存檔，否則丟棄），印 `[rec] ERROR: camera X lost N s into episode ...`；為了切掉掉線後幾拍的凍結影像，幀先在 `FrameHold` 等 0.6 s 才寫入（正常停止時全部寫入）。相機恢復後可直接按 A 開新 episode，不用重啟。
- 驗證：teleop 測試（domain 99）real_ros_tracking 20、teleop_integration 21、cri_jog 35、launcher_frame 2、tracing_log 2 皆通過（主 session 重跑）；recorder（domain 98，假相機 = 影片檔，「拔線」= 移走檔案）新 `tests/test_real_camera_reconnect.py` 11 項 + 既有錄製測試通過。`tests/ros_isolation.py` 新增 `ROBOT_REMOTE_TEST_DOMAIN`（兩個測試同時跑時用不同 domain），robot domains 加入 12。
- **未驗證**：真實 ARC 相機的 USB 掉線重開、實際頭盔按住 Y 的手感。右腕相機一再掉線多半是硬體（線被手臂拉扯／接頭／hub），仍需現場檢查。

## 搖桿旋轉工具（2026-11-03，builder 實作，Claude 委派；僅假 driver 測試，未接實體）

- 需求：操作者覺得轉手腕旋轉工具很吃力。握把按住（arm.engaged）時，該手的搖桿慢速旋轉該臂工具。只用於 IK 後端（isaac / real-ros / real Ethernet），`cri-jog` 不受影響。
- 輸入：`QuestController.read_all()` 另填 `self.sticks = {hand: (x, y)}`（`THUMBSTICK_X/Y`，未追蹤／缺值／非有限 → (0,0)，夾在 [-1,1]）；teleop 以 `getattr(xr, "sticks", {})` 讀取（舊假 Quest 沒有也可），依 `hand_of`／`--swap-hands` 取驅動該臂那隻手。
- 對應（`quest_teleop/stick_rotation.py`）：死區 `--stick-deadzone` 0.15 後二次曲線；滿桿速率 `--stick-rot-vel` 15 deg/s（0 = 關，不超過 `--max-rot-vel`）。Y（上 = +）：傾斜工具尖端，軸 `normalize(z_tool × up)`，正轉使工具 Z 朝上；工具接近垂直（|z×up| < 0.2）改用操作者右方軸。X：`--stick-x spin`（預設，繞工具自身 Z，右 = 正轉）或 `yaw`（繞底座垂直軸，右 = 往右）。`--stick-signs SX SY` 可翻轉。積分進握持參考姿態 `arm.home_r`（在 r_des 計算前），所以 tool／base 兩種旋轉座標系都適用；Cartesian 速率限制、anti-windup、IK、加速度限制、leash、手臂 guard 照舊。
- `--rot-input hand+stick|stick|hand`（預設 hand+stick；GUI「Rotation input」，存 `config/launcher.json` 的 `rot_input`）：stick = 姿態只來自搖桿（`r_des = home_r`，手的旋轉不進入；平移仍跟手）；hand = 搖桿無效。
- 防 windup：只在「上一拍 IK 解的姿態」與指令姿態 `cmd_r` 相差 < 5°（`STICK_MAX_LEAD_DEG`）時積分，否則跳過並記錄節流警告「stick rotation held - wrist cannot follow (joint limit?)」。**刻意用 IK 解而非任務書的 FK(q_cmd)**：real-ros 的加速度／煞車限制讓 q_cmd 自然落後目標約 v²/(2·dec)（15 deg/s 時約 5°），用 q_cmd 在正常轉動時就會一直觸發。極限測試：不加 guard 放手後再轉 2.3°，加 guard 0.08°。
- trace 在 `trigger` 後新增 `stick_x, stick_y`；啟動記錄 `thumbstick rotation: <rate> deg/s full stick (X = spin|yaw, Y = tilt), rotation input <mode>`。
- 測試：`tests/test_stick_rotation.py`（單元 + real-ros 假 driver 整合：X 滿桿 2 s 繞自身 Z 轉約 30°、法蘭漂移 < 1 cm、放手後停止；Y 上／下改變工具 Z 的上分量；未握把不動；`--rot-input stick` 手轉 30° 工具姿態不變、平移仍跟；`hand` 搖桿無效；撞極限放手後不再轉），`test_launcher_frame.py` 新增 Rotation input／圖例測試。**未驗證**：實際頭盔搖桿方向與手感（OpenXR +x 右、+y 上）、sim（isaac 後端）整合只有參數與共用路徑，未另跑假 Isaac 測試、實機。
