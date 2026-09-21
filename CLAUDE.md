# drone_ws — 機載電腦上的無人機程式

這台 Jetson 是一台**實際在飛的無人機**的機載電腦。在這裡跑 Claude Code 時，以下是基準事實與規則。

使用者 PC 上的知識庫（`C:\Users\jerem\Obsidian\Drone`，飛行流程頁 `wiki/howto/offboard-flight-procedure.md`）有同一份紀錄的完整版。**任何機上程式或 PX4 參數的變更，這裡與知識庫兩邊都要記錄**（使用者 2026-09-10 要求）。

## 硬體與軟體（2026-09-10 查證）

| 項目 | 值 |
| --- | --- |
| 機載電腦 | NVIDIA Jetson Orin Nano Developer Kit Super（**不是**舊款 Jetson Nano）。L4T R36.4.7、Ubuntu 22.04、ROS 2 Humble，系統在 NVMe SSD |
| 飛控 | **Pixhawk 6X**（FMUv6X，STM32H753；USB `3185:0035`「PX4 FMU v6X.x」）。不是 6X-RT |
| 飛控韌體 | PX4 v1.17.0 正式版（ULog `ver_sw_release = 0x011100FF`） |
| Jetson ↔ 飛控 | USB `/dev/ttyACM0`，走 MAVROS（`fcu_url:=/dev/ttyACM0:57600`），不是 uXRCE-DDS |
| 遙控器 | RadioLink AT10II（Mode 2）。CH5 = SwG 飛行模式（1065 Land / 1499 Offboard / 1933 Position）、CH9 = SwD Arm、CH11 = SwF Kill（1065 關 / 1933 開） |
| 其他 | Orbbec Gemini 2 深度相機、6S 電池 |

## 在這台機器上的規則

- 以下動作**每次都要先問使用者、得到明確同意**才做：寫入 PX4 參數、任何會解鎖或讓馬達轉的動作（包括啟動 `takeoff_land` / `fm_deploy` 的 launch 檔）、修改本工作區程式碼、重新 build、關機重開、`apt` 安裝
- `~/drone_ws` 是 git repo，夥伴也會改。**不要自行 git commit / push**
- 改程式前先備份成 `<檔名>.bak-<標籤>-<時間>`（本工作區慣例）
- 跑 ROS 前：`conda deactivate` → `source /opt/ros/humble/setup.bash` → `source ~/drone_ws/install/setup.bash`
- shell 腳本裡**不要在 `source /opt/ros/humble/setup.bash` 之前 `set -u`**：setup.bash 會讀未定義的 `AMENT_TRACE_SETUP_FILES` 而中斷
- 同一時間只能有一個程式開 `/dev/ttyACM0`。開 MAVROS 前先 `pgrep -a -f '[m]avros_node'`
- 飛行日誌（ULog）在 Pixhawk 的 SD 卡：`/fs/microsd/log/<UTC 日期>/<UTC 時間>.ulg`

## 變更紀錄

### 2026-09-10：起飛穩定度參數 + Kill 開關保護

使用者提出三點：降落要更慢；Offboard 中撥 Kill 後程式必須停止、放開 Kill 螺旋槳不能再轉；自動起飛離地前搖擺。

#### PX4 參數（已寫入，並從飛控重新讀回確認）

| 參數 | 舊值 | 新值 | 原因 |
| --- | ---: | ---: | --- |
| `MPC_THR_HOVER` | 0.5 | **0.65** | 9/9 飛行（log 89）懸停實測推力 0.66 |
| `MPC_TKO_RAMP_T` | 3.0 | **1.5** | 縮短起飛時壓在地上累積積分的時間 |
| `COM_KILL_DISARM` | 5.0 | 5.0（不改） | 使用者決定保留「誤觸 Kill、5 秒內撥回可恢復」 |

相關但沒改：`MPC_LAND_SPEED` 0.6、`MPC_Z_VEL_MAX_DN` 0.5、`MPC_Z_VEL_MAX_UP` 0.5、`LNDMC_Z_VEL_MAX` 0.25、`RC_MAP_KILL_SW` 11、`COM_OBL_RC_ACT` 4、`COM_OF_LOSS_T` 1.0。

**起飛搖擺的原因**（PX4 v1.17.0 原始碼 + log 89 對照）：

1. takeoff ramp 開始時速度設定是往下 2.45 m/s（= g / `MPC_Z_VEL_P_ACC`），讓推力從 0 起步。ramp 期間 PX4 不重置速度積分，飛機在地上動不了，積分累積到約 5.7 m/s²
2. ramp 結束時往上只有 `MPC_Z_VEL_MAX_UP` 0.5 m/s，P 項抵不過積分；`MPC_THR_HOVER` 0.5 又低於實際 0.66 → log 89 解鎖 11.2 秒才離地
3. 地上這段時間，落地偵測已判定「不在地上」，姿態積分開始累積（roll 積分到 0.146）；機身停在地上 roll −3.2°、腳架頂住轉不動，離地瞬間釋放 → 約 3 秒內 roll −3° → +6° → −4°

模擬預測（模型可重現 log 89）：解鎖到離地 10.6 → 3.4 秒；地上累積積分時間 7.3 → 1.8 秒。**下次飛行後要用 ULog 驗證。**

#### 程式修改（三支飛行程式）

- 檔案：`src/takeoff_land/takeoff_land/takeoff_land_node.py`、`src/takeoff_land/takeoff_land/hold_position_node.py`、`src/fm_deploy/fm_deploy/fm_inference_real_node.py`
- 備份：同目錄 `*.bak-KILL-20260910-185232`
- 新 ROS 參數（預設值即可）：`kill_switch_enabled`（True）、`kill_channel`（11）、`kill_on_pwm`（1500）
- 程式碼中搜尋 `NEW (2026-09-10)` 可找到每一處修改

| 情況 | 程式行為 |
| --- | --- |
| 解鎖前 SwF（Kill）在開 | 等 20 秒，仍開著就拒絕解鎖 |
| 解鎖流程開始後撥 SwF（CH11 連續 2 筆 > 1500） | 立刻**永久**停止送 setpoint → 要求 AUTO.LAND → 每秒要求一般上鎖（PX4 只在落地後接受）最多 15 秒 → 程式結束。不再解鎖、不再進 OFFBOARD |
| Kill 5 秒內撥回 | 預期 PX4 在 Land 模式恢復馬達並自己降落（`COM_OBL_RC_ACT` = 4 也是 Land）。程式不接管 |
| Kill 超過 5 秒 | PX4 自動強制上鎖，撥回也不會轉 |
| 任務中發生程式沒要求的上鎖 | 立刻停止送 setpoint、不切模式、不再解鎖、程式結束 |
| `hold_position_node` | 只在空中使用，從啟動就監看 Kill |

**修掉的漏洞**：舊版 `takeoff_land_node._wait_altitude()`（起飛等待爬升）沒有檢查解鎖狀態，Kill 期間仍持續送起飛 setpoint 最多 30 秒；5 秒內撥回 Kill，馬達恢復後會在 OFFBOARD 繼續起飛（9/9 log 88 就發生過）。舊版避障程式遇到意外上鎖會走降落流程、繼續送 setpoint。

**驗證**：py_compile、pyflakes 無新問題；`colcon build --packages-select takeoff_land fm_deploy` 成功，install 內檔案與 src 一致；邏輯測試 22/22 通過（不啟動 MAVROS、不跑任務，只餵假的 RC 與解鎖訊息）。

**尚未做：拆槳實機測試。** 步驟：

1. **先拆槳。** SwF 開著啟動 `takeoff_land` → 應看到 `KILL SWITCH IS ENGAGED ... release it`，20 秒後 `REFUSING TO ARM`
2. SwF 關著重新啟動 → 進入 ARMING、解鎖後撥 SwF → 應看到 `[KILL] KILL SWITCH ENGAGED`、馬達停止、模式變 AUTO.LAND
3. 1–2 秒內撥回 SwF → 馬達可能恢復，但應在 Land 模式、隨後判定落地上鎖；程式已結束，不會再送指令
4. 再測一次，SwF 撥著超過 5 秒 → PX4 上鎖，撥回馬達不轉

**還原方式**（需使用者同意）：

```bash
cd ~/drone_ws/src
cp takeoff_land/takeoff_land/takeoff_land_node.py.bak-KILL-20260910-185232 takeoff_land/takeoff_land/takeoff_land_node.py
cp takeoff_land/takeoff_land/hold_position_node.py.bak-KILL-20260910-185232 takeoff_land/takeoff_land/hold_position_node.py
cp fm_deploy/fm_deploy/fm_inference_real_node.py.bak-KILL-20260910-185232 fm_deploy/fm_deploy/fm_inference_real_node.py
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select takeoff_land fm_deploy
```

#### 降落速度（決定：不改）

- 程式自己控制的下降段維持 0.3 m/s（`descent_speed`）
- PX4 自己的降落（SwG 撥 Land、程式最後 0.25 m 交給 AUTO.LAND）是 0.5 m/s，已是文件最小值（`MPC_LAND_SPEED` 最小 0.6、`MPC_Z_VEL_MAX_DN` 最小 0.5）
- 若強制 `MPC_LAND_SPEED` = 0.2：v1.17.0 會以 0.2 下降，但落地偵測器會自動把 `LNDMC_Z_VEL_MAX` 改成 min(`MPC_LAND_CRWL`, `MPC_LAND_SPEED`) / 1.2 ≈ 0.167 並存檔，觸地判定變嚴，下降意圖門檻（≈ 0.183）只剩很小餘裕；且這台高度基準是 GPS，降得越慢越容易被高度漂移蓋過（8/27 撞樹事故就是降落中漂移）。**不建議**
- 要更慢：加裝朝下測距儀，用 `MPC_LAND_CRWL`（最小 0.1，須大於 `LNDMC_Z_VEL_MAX`）放慢離地 `MPC_LAND_ALT3` 以下那段

#### 其他待辦

- 在水平地面起飛前先確認姿態（9/9 那個位置停在地上 roll −3.2°）；水平校正 `SENS_BOARD_X_OFF` / `Y_OFF` 在 9/9 被重做過，若手機水平儀與 QGC 角度對不上再重做
- 重心偏左前：懸停時左側兩顆馬達推力多約 10%
- 兩組對角馬達推力差約 10%（偏航需額外修正）；機體 8/27 撞過樹，檢查槳與馬達
- 懸停振動偏強：原始加速度峰對峰 9–18 m/s²（官方參考線 2–3），8/27 前就是這個等級
- 避障程式啟動時 `ONNX failed to init CUDAExecutionProvider`，規劃器改用 CPU 推論

### 2026-09-13：高度基準 GPS → 氣壓計（`EKF2_HGT_REF`）

使用者回報：飛機靜置地上、未解鎖，z 卻不穩。`takeoff_land` 因此 `EKF NEVER BECAME TRUSTWORTHY` 拒絕解鎖、`gps_guard` 回報 `NOT READY gnd_drift`。

**查證**（飛機靜置、GPS DGPS fix=4、26–30 顆、HDOP 0.52、VDOP 0.95；同時錄 90 秒原始資料）：

| 來源 | 峰對峰 |
| --- | ---: |
| GPS 原始高度（`/mavros/gpsstatus/gps1/raw` alt） | **3.60 m** |
| EKF local z | **3.63 m** |
| 氣壓計（`/mavros/altitude` monotonic） | **0.47 m** |

EKF z 完全跟著 GPS 高度漂（例：60→85 s GPS −0.05→+2.95、z −0.18→+3.03，氣壓計 ≈ 0）。原因是 `EKF2_HGT_REF` = 1（GPS）。GPS 垂直誤差受環境影響（遮蔽/多路徑、衛星幾何、SBAS 修正、USB3 干擾），水平再好也可能垂直漂好幾公尺；「沒改東西卻突然不穩」就是環境變了。

#### PX4 參數（已寫入，並從飛控重新讀回確認，飛機未解鎖）

| 參數 | 舊值 | 新值 | 原因 |
| --- | ---: | ---: | --- |
| `EKF2_HGT_REF` | 1（GPS） | **0（Baro）** | 見上表 |

寫入時其他相關值（沒改）：`EKF2_BARO_CTRL` 1、`EKF2_GPS_CTRL` 7（水平位置、速度仍用 GPS；GPS 高度改為估偏差的輔助）、`EKF2_RNG_CTRL` 1（沒裝測距儀）、`EKF2_EV_CTRL` 0、`EKF2_BARO_NOISE` 3.5、`EKF2_GPS_V_NOISE` 0.3、`EKF2_GPS_P_NOISE` 0.5。

- 程式面不受影響：`takeoff_land`、`forward_move`、`fm_deploy`、`gps_guard` 只用相對 local z（`ground_z + target_alt`），沒有用 AMSL/GPS 高度。`fm_deploy` 只有 `write_px4_params:=true` 才會寫 `EKF2_HGT_REF`（預設 false）；**別開，模擬用的父類別會寫回 1**
- 注意：氣壓計隨天氣慢漂（幾分鐘的任務影響小）；近地面有螺旋槳下洗的地效（`EKF2_GND_EFF_DZ` / `EKF2_GND_MAX_HGT`，未查）；Pixhawk 要加海綿擋風擋日曬
- 上面「降落速度」一節說「高度基準是 GPS」是 9/10 當時的狀態

**重開後重測（12:59，Jetson 12:54 重開；Pixhawk 是否一起重開未確認）：只改 `EKF2_HGT_REF` 不夠。**

| 60 秒靜置 | 峰對峰 |
| --- | ---: |
| GPS 原始高度 | 1.70 m |
| EKF z | 1.11 m |
| 氣壓計 | 0.47 m |

corr(EKF z, GPS) = **+0.99**、corr(EKF z, Baro) = −0.11；絕對 z 在地上 −8.8 → −9.9 m（`takeoff_land` 的 `max_ground_z` 1.0 會拒絕解鎖）。原因：EKF2 會融合**所有啟用的**高度來源，`EKF2_HGT_REF` 只決定哪個來源不估偏差。`EKF2_GPS_CTRL` = 7 仍含 GPS 高度（bit 1），GPS 垂直雜訊（≈ epv，~1 m）比 `EKF2_BARO_NOISE` 3.5 m 小很多，權重大，偏差估計又追不上數十秒的漂移 → z 仍跟 GPS。

#### 追加 PX4 參數（使用者 13:0x 自己用 `ros2 param set` 寫入；Claude 13:05 從 MAVROS 讀回確認）

| 參數 | 舊值 | 新值 | 原因 |
| --- | ---: | ---: | --- |
| `EKF2_GPS_CTRL` | 7 | **5** | 關掉 GPS 高度（bit 1），保留水平位置（bit 0）+ 3D 速度（bit 2） |

**驗證（13:07，90 秒靜置，飛機未解鎖）：成功。**

| 來源 | 峰對峰 |
| --- | ---: |
| GPS 原始高度 | 2.72 m |
| EKF z | **0.43 m** |
| 氣壓計 | 0.73 m |

corr(EKF z, Baro) = +0.77；GPS 往下漂 2.5 m 時 EKF z 不跟。

**但絕對 z 在地上仍是 −7.5 m**（切換前 GPS 高度時代留下的偏移；這段時間 MAVROS 沒看到 Pixhawk 斷線，推測 Pixhawk 還沒斷電重開）。`takeoff_land` 的 `max_ground_z` 1.0 會拒絕解鎖 → **要拔電池 + USB 重開 Pixhawk，再確認地上 |z| < 1 m**。

**Pixhawk 斷電重開後重測（13:15，Pixhawk 開機 13:12:48，由 `/mavros/timesync_status` remote_timestamp 推算；參數讀回仍是 HGT_REF 0 / GPS_CTRL 5）**：90 秒峰對峰 EKF z **0.42 m**、GPS 1.72 m、Baro 0.65 m；corr(z, Baro) +0.72。穩定沒問題，**但地上絕對 z 仍 −3.3 ~ −3.75 m**；home z = −3.42（x 0.17、y −0.60），代表開機後約 1 分鐘內（GPS 定位、設 home 之前）z 就偏到 −3.4，之後持續平穩。推測是開機暖機時氣壓計溫漂或 EKF 初始化，**原因未確認**（要看 ULog 開機段 `sensor_baro` / `estimator_states`）。影響：`takeoff_land` 的 `max_ground_z` 1.0 會拒絕解鎖；起降本身用相對 `ground_z`，home 也在 −3.42，常數偏移本身不影響飛行。

**13:25 使用者跑 `takeoff_land`（`max_ground_z:=5.0`）**：`EKF stable. ground_z=-2.672m (std=0.0465, spread=0.137m)` → takeoff_z −1.17 → ARMED → 進 TAKEOFF 後 1 秒內 SwF 撥下，`[KILL] KILL SWITCH ENGAGED (ch11=1933)` → AUTO.LAND → 3 秒後 DISARMED，程式不再解鎖。Kill 流程在實機解鎖狀態下照設計運作（有沒有裝槳未確認）。

地上絕對 z 隨時間：13:15 −3.5、13:25 −2.7、13:27 −3.0、13:29 −2.5（ALT Rel 相對 home ≈ 0）。z = 0 是 Pixhawk 開機 EKF 初始化那一刻的氣壓高度，不是「地面」；之後的偏移 = 氣壓高度變化（1 hPa ≈ 8 m）。開機後 2 分鐘就偏 −3.5 m，天氣變化太慢，推測主因是開機暖機：13:28 IMU 溫度 47.2 → 47.5 °C/70 s 仍在升（`SENS_EN_THERMAL` −1、`TC_B_ENABLE` 0 沒做氣壓計溫度補償）。未用 ULog 確認。地效相關：`EKF2_GND_EFF_DZ` 4.0、`EKF2_GND_MAX_HGT` 0.5。

13:35 / 13:41 GPS 原始資料 2 分鐘靜置（Orbbec 接著）：水平 CEP50 0.43 → 0.21 m、95% 0.61 → 0.33 m；垂直峰對峰 2.56 → 1.19 m、std 0.88 → 0.23 m；接收機自報 v_acc ~0.5 m（13:35 實際比自報差）。同地點 6 分鐘內差很多 → GPS 沒壞，垂直品質隨時間/環境變。13:41 地上絕對 z 平均 −0.82 m（−1.19 ~ −0.05），最後 30 秒氣壓計 +0.9 m、z 跟著升（原因未確認：風、碰到飛機？）。

#### 2026-09-13 13:4x：**已還原成 GPS 高度**（使用者決定）

使用者認為戶外用氣壓計太危險，要求全部還原。Claude 寫入並 force pull 後從飛控讀回確認（飛機未解鎖）：

| 參數 | 當時值 | 還原後 |
| --- | ---: | ---: |
| `EKF2_HGT_REF` | 0 | **1（GPS）** |
| `EKF2_GPS_CTRL` | 5 | **7** |

程式碼、launch 檔這次都沒改過，不用還原（`max_ground_z` 本來就是 launch 參數，預設 1.0）。**現況 = 回到 9/13 之前的設定**：z 會跟 GPS 高度漂（本日實測靜置 1.2–3.6 m 峰對峰），`takeoff_land` 的 EKF 地面檢查可能再度拒絕解鎖。需要 Pixhawk 斷電重開讓 EKF 乾淨初始化。

以下「尚未做」是氣壓計方案當時的待辦，已還原後不適用：

**（已不適用）尚未做**：決定 −3.4 m 偏移的處理（查 ULog 找原因，或啟動時 `max_ground_z:=5.0`，需使用者同意）；低空懸停測試觀察起降時是否因地效彈跳；知識庫同步記錄。還原：`EKF2_HGT_REF` 設回 1、`EKF2_GPS_CTRL` 設回 7。

### 2026-09-13：`fm_deploy` octomap 高度帶改成相對地面（修 OFF-MAP 誤判）

**事件（15:05 `fm_all.launch.py goal_dist:=3.0 target_alt:=2.0 max_alt_error:=0.5 dry_run:=false max_ground_z:=100.0 v_max:=0.3 descent_speed:=0.3`，GPS 高度基準）**：`ground_z=-6.16`、cruise z −4.16，起飛/懸停正常；進 FLYING 後一直 `Collision guard! drone is OFF-MAP — hover (no escape)`、`HOLDING - no trajectory`，10 秒後 `ABORT: drone off-map / outside arena ... for 10s` → 原地降落 → DONE。

**原因**：`fm_inference_real_node` 呼叫 `self._configure_octomap_band(self._alt)`，父類別用**絕對 odom z** 算 `[max(0.35, z−0.7), z+1.0]` = **[1.30, 3.00]**；odom z = EKF local z（`mavros_tf_broadcaster_node`），飛機在 −4.16，帶子在飛機上方 5–7 m → octomap 在飛行高度沒有資料 → `esdf.is_unobserved()` → OFF-MAP → blind abort。ground_z ≈ 0 時兩者剛好一樣，所以以前沒出事；`max_ground_z` 1.0 等於意外擋住了這個 bug，`max_ground_z:=100` 讓它現形。

**同一次飛行的高度漂移**：懸停 EKF alt 2.0，AUTO.LAND 觸地上鎖時 EKF alt ≈ 0.96 → 約 40 秒飛行中 GPS 高度漂 +0.9 m，實際懸停高度約 1.1 m。`max_alt_error` 抓不到（EKF 認為在目標上）。建議 `target_alt:=3.0`。

#### 程式修改

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_real_node.py`（`fm_inference_base.py` 沒動，與模擬共用；md5 `a0e9f134…` 不變）
- 備份：`fm_inference_real_node.py.bak-OCTOBAND-20260913-151602`
- 新增 override `_configure_octomap_band(alt_above_ground)`：`occ_min = ground_z + max(0.35, alt − 0.7)`、`occ_max = ground_z + alt + 1.0`，其餘（set_parameters、reset octomap）同父類別；log 改印 odom z 與離地高度。搜尋 `NEW (2026-09-13)`
- ground_z = 0 時結果與舊版完全相同（[1.30, 3.00]）

**驗證**：py_compile OK；pyflakes 無新問題（唯一一條 f-string 是舊的）；離線邏輯測試 6/6（假 service client，不開 MAVROS/octomap：ground_z 0 → [1.30, 3.00]、−6.16 → [−4.86, −3.16] 含 cruise −4.16、+1.71、低空 0.35 m 地板、alt 3.0）；`colcon build --packages-select fm_deploy` 成功，install 與 src 一致。

**尚未做**：`dry_run:=true` 實機確認 log 出現 `[REAL] ok Octomap band [...] odom z = [1.30, 3.00] m above ground` 且 PREFLIGHT 後不再 OFF-MAP；實飛。其他 z 用法已查：real node 的 alt、`max_alt_error`、降落都用 `ground_z` 相對值；ESDF 是 octomap 投影的 2D。

**還原**（需使用者同意）：

```bash
cd ~/drone_ws/src/fm_deploy/fm_deploy
cp fm_inference_real_node.py.bak-OCTOBAND-20260913-151602 fm_inference_real_node.py
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy
```

### 2026-09-13：`fm_deploy` 規劃模型改用 TensorRT 在 GPU 上推論

使用者要求：把 TensorRT 模型放進 `model/fm`，程式改成用 GPU 讀取，減輕 CPU 負擔。

**背景**：torch 與 onnxruntime 在這台 Jetson 用不了 GPU（`cublasCreate` → `CUBLAS_STATUS_ALLOC_FAILED`，8/04 起未解）。15:05 飛行中一次推論 420–510 ms（整個 replan 830–1440 ms）。

**查證**：

- 原生 TensorRT 不受 cuBLAS 問題影響。`libnvinfer_plugin` 需要 `libcudla.so.1`，檔案在 `/usr/local/cuda-12.6/targets/aarch64-linux/lib`（不在 ldconfig 路徑）；程式用 ctypes 預先載入，launch 不用設 `LD_LIBRARY_PATH`
- onnxruntime 的 TensorrtExecutionProvider 仍失敗（會初始化 CUDA EP / cuBLAS）
- TensorRT 10.3 對這個模型輸出錯誤（對 `.pth` 最大差 1.87）：`/encoder/Concat_output_0`（影像 128 + 運動 24）與後面的 `Expand` 被錯誤融合。與 opset（13/17/20）、TF32、builder 最佳化等級無關。把該 tensor 標成額外 graph output 即修正（自動二分搜尋 114 個中間 tensor 找到）

#### 新模型檔（`src/fm_deploy/model/fm/`，原 `.pth` / `.onnx` 未動）

- `fm_planner_20260724_190037_trt.onnx`：從 `.pth` 重新匯出（opset 17），標記 Concat output
- `fm_planner_20260724_190037_trt_fp32.engine`：`trtexec --onnx=..._trt.onnx --noTF32 --memPoolSize=workspace:256`。**engine 只能在建它的 TensorRT 版本 + GPU 上用**

#### 程式修改

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_node.py`、`launch/fm_all.launch.py`、`launch/fm_real.launch.py`（`fm_inference_base.py` 沒動）
- 備份：同目錄 `*.bak-TRT-20260913-215639`
- `model_path` 為 `.engine` → GPU 推論；K 固定 8，anchor 模式要 7 筆時補到 8 再丟掉多的
- 啟動時與 `_trt.onnx`（CPU）比對同一組固定輸入（新參數 `trt_parity_check` True、`trt_parity_tol` 1e-3）；任何失敗 → 自動改用該 `.onnx` 在 CPU。找不到 `.onnx` → node 不啟動
- 兩個 launch 的 `model_path` 預設改成 engine；舊預設 `...190037.onnx` 仍可用參數指定
- 搜尋 `NEW (2026-09-13)`

**驗證**：py_compile、pyflakes 無問題；離線測試 10/10（不 spin ROS、不開 MAVROS）：engine 載入、parity 3.6e-6；30 組 uint8 深度輸入（K 7/8）與兩個 `.onnx` 差 ≤ 1.4e-5；未修正的 engine 被拒（差 2.18）並退回 CPU；損壞 engine 退回 CPU；缺 `.onnx` 時 node 停止；舊 `.onnx` 路徑行為不變。推論 43.7 ms（GPU）vs 173 ms（CPU 單獨）/ 420–510 ms（CPU 飛行中）。

**Build（22:04，使用者同意）**：`colcon build --packages-select fm_deploy` 成功；install 內 `fm_inference_node.py`、兩個 launch、engine、`_trt.onnx` 與 src 一致；`ros2 launch fm_deploy fm_all.launch.py --show-args` 的 `model_path` 預設是 engine；用 install 內的模組重跑離線測試 10/10。

**使用者室內 dry run（Pixhawk 只接 USB、無電池、無 GPS）**：

- 22:06 `dry_run:=true`：`TensorRT parity OK 3.62e-06`、`Backend : FM TensorRT (GPU)`；但地上 z = 1.9 → 2.3 m（無 GPS 時 EKF 用氣壓計高度，慢漂），`max_ground_z` 1.0 → `EKF NEVER BECAME TRUSTWORTHY`，沒進規劃
- 22:11 `dry_run:=true max_ground_z:=5.0`：ground_z 2.52、octomap band `[3.32, 5.02] odom z = [0.80, 2.50] m above ground`（9/13 band 修正實機生效）；13 次 `Replan ok`

| run | 推論 median | replan 總計 median（min–max） | 其餘（optimizer + 檢查）median |
| --- | ---: | ---: | ---: |
| 15:05 飛行，ONNX CPU | 482 ms | 1000 ms（830–1443） | 518 ms |
| 15:43 dry run，ONNX CPU | 435 ms | 1082 ms（852–1324） | 648 ms |
| **22:11 室內 dry run，TensorRT GPU** | **44 ms** | 1244 ms（360–12703） | **1200 ms** |

推論快 10 倍，但 replan 總時間沒變快：其餘部分（L-BFGS optimizer、ESDF 檢查，CPU）變成約 2 倍，2/13 次第一個候選失敗要解第二次（4631、12703 ms）。場景不同（室內、戶外），**原因未確認**；要在同一地點用 `model_path:=...190037.onnx` 做 A/B，並同時錄 `tegrastats`。`bat=65.5V` 是沒接電池時的假值。

**尚未做**：同地點 GPU / CPU A/B 比較；實飛；知識庫同步記錄。

**還原**（需使用者同意；或不還原、直接 `model_path:=~/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037.onnx`）：

```bash
cd ~/drone_ws/src/fm_deploy
cp fm_deploy/fm_inference_node.py.bak-TRT-20260913-215639 fm_deploy/fm_inference_node.py
cp launch/fm_all.launch.py.bak-TRT-20260913-215639 launch/fm_all.launch.py
cp launch/fm_real.launch.py.bak-TRT-20260913-215639 launch/fm_real.launch.py
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy
```

### 2026-09-14：修 `fm_deploy` 啟動即 crash（Backend 著色）

**事件（12:07 `fm_all_barometer.launch.py goal_dist:=2.0 target_alt:=2.0 max_alt_error:=0.5 dry_run:=false`）**：沒有解鎖。`fm_inference_baro_node` 啟動約 2 秒 `process has died, exit code 1`，其他 node（MAVROS、相機、octomap）照跑。

**原因**：9/14 10:27 有人改了 `fm_inference_base.py`（未 commit，已 build），banner 的 `Backend` 行改呼叫 `self._c('blue')`。`_c()` 定義在 `FMInferenceRealNode`，讀 `self._color`，但 `_color` 在 `super().__init__()` 之後才設定 → `AttributeError: 'FMInferenceRealNode' object has no attribute '_color'`。影響所有用 real node 的 launch（`fm_all`、`fm_real`、兩個 barometer 版），`takeoff_land` / `forward_move` 不受影響。

#### 程式修改（使用者同意）

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_real_node.py` 的 `_c()`：`self._color` → `getattr(self, "_color", True)`（base 沒動）。搜尋 `NEW (2026-09-14)`
- 備份：`fm_inference_real_node.py.bak-COLORFIX-20260914-121206`；本檔 `CLAUDE.md.bak-COLORFIX-20260914-121206`

**驗證**：py_compile OK；pyflakes 只有舊的一條 f-string；`colcon build --packages-select fm_deploy` 成功，install 與 src md5 相同；離線 init 測試（不開 MAVROS、`dry_run:=true`、不 spin）`FMInferenceRealNode`、`FMInferenceBaroNode` 都 INIT OK，`Backend : FM TensorRT (GPU) K=8`。**尚未做**：實機重跑。

**還原**（需使用者同意）：

```bash
cd ~/drone_ws/src/fm_deploy/fm_deploy
cp fm_inference_real_node.py.bak-COLORFIX-20260914-121206 fm_inference_real_node.py
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy
```

### 2026-09-14：9/14 飛行紀錄分析 + `fm_deploy` replan 加速（單一候選、時間上限、向量化、單執行緒 BLAS）

**9/14 log 分析（`~/.ros/log`，18 次啟動）**：`fm_deploy` 三次實飛（12:22、12:31、12:42，都是 barometer 版）起飛/懸停正常，但進 FLYING 後都沒前進。原因是 guard 不是 planner（`Replan ok` 有 12–22 次，狀態卻一直 `HOLDING - no trajectory`）：

- `drone is OFF-MAP`：無人機所在格在 octomap 是 unknown → 每 0.5 s 作廢軌跡 → 10 s 後 abort
- 地圖是空的：Gemini 2 戶外 `only 0–4% of pixels valid`（相機本身回 0，不是 4 m 截斷）；HOVER 結束清空 octomap 後沒有資料再填進去；空曠場地沒有障礙物可看。**日光是否為主因未確認**（天空、距離、曝光設定混在一起）
- `depth_to_pointcloud` 高度 gate 用 EKF（GPS）高度且自己算 ground_z：12:42 gate 把 ground_z 3.58 改成 0.00（fm node 用 2.63）→ 飛行中 gate 一直 CLOSED，完全沒有深度進來；12:22 FLYING 大部分時間也 CLOSED
- 12:22 另有 `ESDF too close`（推測地面進入 octomap 帶，EKF 與氣壓高差 1.5 m）與一次 **55 s** 的失敗 replan
- 對照：12:38 `forward_move_baro`（不用相機）**成功前進 2 m**，終點誤差 0.05 m

使用者在空曠場地測試。若要在空地飛可關 guard（launch 參數，**不改程式**）：`use_lookahead_guard:=false blind_abort_s:=0`（`use_safety_guards` 預設已 false），GPS 版另需 `max_ground_z:=100.0`，建議 `gate_enabled:=false v_max:=0.3 target_alt:=3.0`。關掉後**沒有任何避撞**；地理圍欄、高度、電池、意外上鎖、RC、Kill 仍有效。尚未實飛。

#### 程式修改（使用者要求：只用一個候選 + 修 optimizer 其他問題）

離線量測（`bench_planner.py`，假 OccupancyGrid，不開 ROS/MAVROS）找到的原因：

| 問題 | 量測 |
| --- | --- |
| 被擋住時一次 replan = 2 目標 × 8 候選 × 5 次擾動重試 | 單一候選失敗 3.6 s → ×16 ≈ 58 s，吻合 12:22 的 55 s |
| 多執行緒 OpenBLAS 解 18×18 矩陣（每次 cost/grad 各一次，另有 `A.T` solve） | 0.418 ms/次；`OPENBLAS_NUM_THREADS=1` 0.017 ms（25×），相機/octomap/MAVROS 同時跑 CPU 時更糟 |
| cost/grad 逐點 Python 迴圈 + 逐點 ESDF 查詢（每次加鎖）；`get_full_state_cmd` 逐點 | state_cmd 13–24 ms |

- 檔案：`src/fm_deploy/fm_deploy/min_jerk_planner.py`、`esdf_ros2.py`、`fm_inference_base.py`、`src/fm_deploy/launch/fm_all.launch.py`、`fm_real.launch.py`、`src/fm_deploy_barometer/launch/fm_all_barometer.launch.py`、`fm_real_barometer.launch.py`
- 備份：同目錄 `*.bak-OPTFAST-20260914-144028`；本檔 `CLAUDE.md.bak-OPTFAST-20260914-144028`
- 搜尋 `NEW (2026-09-14)`
- `min_jerk_planner.py`：`add_sampled_cost` / `add_sampled_grad_CT` / `get_full_state_cmd` 改為整批陣列運算（公式、取樣點、端點權重不變）；新 `PlanTimeout`（RuntimeError 子類）與 `planner.deadline`（`time.monotonic`，None = 不限，expert/模擬行為不變），超時不再重試；軌跡含 NaN 時丟 ValueError（同舊版 `int(NaN)` 失敗）；地圖物件沒有 `get_edt_batch` 時退回逐點 API
- `esdf_ros2.py`：新增 `get_edt_batch(pts)`，一次加鎖，索引用 `np.trunc`（= 舊 `int()`），地圖外 / 尚無地圖回 10000 / [0,0]
- `fm_inference_base.py`：新 ROS 參數 `max_candidates`（**程式預設 0 = 全部**，模擬不變）、`replan_budget_s`（**程式預設 0 = 不限**）；整個 replan（兩個目標、所有候選、所有重試）共用一個時間上限，超過 → replan 失敗（保留舊軌跡，同一般失敗），log `Replan failed (Nx) — time budget ... exceeded`；候選 marker 仍顯示全部 K；成功 log 仍是 `cand#i/K`；啟動 banner 多一行 `Replan cost : candidates … | budget … | OPENBLAS_NUM_THREADS=…`
- 四個 launch：`max_candidates` 預設 **1**、`replan_budget_s` 預設 **1.0**；fm node 加 `additional_env={"OPENBLAS_NUM_THREADS": "1"}`（只影響 fm node 程序）
- 沒改 guard、GPS/EKF 門檻、Kill、RC、地理圍欄、PX4 參數

**驗證（離線，不開 MAVROS、不解鎖、node 不 spin）**：py_compile OK；pyflakes 無新問題（只有行號不同）；與備份版逐項比較 4 種地圖（空、unknown、障礙、完全擋住）× 20 組隨機變數：cost/grad 相對差 ≤ 3e-12，最佳化結果 waypoint 差 ≤ 5e-15，state_cmd 差 ≤ 8e-15 → **PASS**；deadline / fallback / ESDF batch（5005 點含負座標截斷、地圖外、尚無地圖）/ NaN / state_cmd（hz 7、50、300）12/12；node 層（src 版 `FMInferenceRealNode`，`.onnx`，假地圖與假候選）8/8：舊行為 16 次最佳化 3.35 s、`max_candidates=1` 2 次 0.42 s、budget 0.3 → 0.30 s 停止並 log、空地圖 1 次 12 ms 成功並安裝軌跡、deadline 每次都重置；`fm_deploy_barometer/test/offline_test_recovery.py` 用 src 版 35/35；四個 launch `--show-args` 正常。

離線計時（中位數，同一台 Jetson，當時另有 MAVROS 在跑）：

| 情境 | 舊版（BLAS 預設） | 新版 + 單執行緒 BLAS |
| --- | ---: | ---: |
| 有障礙、成功 | 1034 ms | 84 ms |
| 完全擋住、單一候選失敗（5 次重試） | 2187 ms | 255 ms |
| state_cmd | 13–25 ms | 0.5–0.7 ms |

**Build（14:5x，使用者同意）**：`colcon build --packages-select fm_deploy fm_deploy_barometer` 成功（shell 的 `python3` 是 conda，但 install 內 executable shebang 仍是 `/usr/bin/python3`、site-packages python3.10）；install 內 3 個 .py、4 個 launch 與 src 逐檔相同。**用 install 版重跑**：等價比較 PASS、deadline 等 12/12、node 層 8/8（舊行為 16 次 3.29 s、`max_candidates=1` 2 次 0.36 s、budget 0.3 → 0.30 s）、`offline_test_recovery.py` 35/35（確認 import 自 install）；4 個 launch 從已安裝套件 `--show-args` 有新參數；迷你 launch（只印環境變數，不是無人機 node）確認 `additional_env` 讓子程序 `OPENBLAS_NUM_THREADS = 1`。

**GPS 版與 barometer 版一致（使用者要求確認）**：`fm_deploy_barometer` 沒有自己的 optimizer / `_replan`（`FMInferenceBaroNode`、`FMInferenceRecoveryNode` 都繼承 `FMInferenceRealNode`），改動自動生效；node 層測試對兩個 barometer node（install 版）各 8/8，數字與 GPS 版相同；四個 launch `max_candidates` 1、`replan_budget_s` 1.0，`fm_real` / `fm_real_barometer` 都有 `OPENBLAS_NUM_THREADS=1`，`fm_all_barometer` 經 `_FORWARDED` 轉給 `fm_real_barometer`。

**實機 dry run（16:22，Claude 經使用者同意執行；無電池、Pixhawk 只接 USB、`fm_all.launch.py goal_dist:=2.0 target_alt:=3.0 max_ground_z:=100.0 use_lookahead_guard:=false blind_abort_s:=0 gate_enabled:=false v_max:=0.3 dry_run:=true`，240 s 自動 Ctrl-C）**：banner `Replan cost : candidates 1 | budget 1.0 s | OPENBLAS_NUM_THREADS=1`、`Backend : FM TensorRT (GPU) K=8`，無 crash；但 PREFLIGHT `EKF NEVER BECAME TRUSTWORTHY`（靜置 45 s 內 EKF z 0.7 → 12.3 m、std 0.25–0.71，狀態 spd 0.1–0.3 m/s，推測室內 / GPS 遮蔽），**沒有進到 replan，實機 replan 時間仍未量到**。EKF std 門檻 0.08 寫死在 `_wait_ekf_stable`，不是參數。

**尚未做（需使用者同意）**：戶外有 GPS 時 `dry_run:=true` 看實機 replan 時間與 banner `Replan cost : candidates 1 | budget 1.0 s | OPENBLAS_NUM_THREADS=1`；實飛；把 1.0 s 上限依實機數據調整；知識庫同步。飛行中 callback 被 replan 拖慢（12:22 barometer `STALE 1.2–3.2 s`）是否已改善要實機確認（optimizer 仍在同一程序）；setpoint 在 55 s replan 期間有沒有斷要看 ULog。

**還原**（需使用者同意；或不還原，只把 launch 參數設回 `max_candidates:=0 replan_budget_s:=0`）：

```bash
cd ~/drone_ws/src
T=bak-OPTFAST-20260914-144028
for f in fm_deploy/fm_deploy/min_jerk_planner.py fm_deploy/fm_deploy/esdf_ros2.py fm_deploy/fm_deploy/fm_inference_base.py \
         fm_deploy/launch/fm_all.launch.py fm_deploy/launch/fm_real.launch.py \
         fm_deploy_barometer/launch/fm_all_barometer.launch.py fm_deploy_barometer/launch/fm_real_barometer.launch.py; do
  cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy fm_deploy_barometer
```

### 2026-09-10：新增 `forward_move` 套件（直線前進）

使用者要求：新套件讓無人機往前飛，起飛與降落和 `takeoff_land` 完全相同。沒有改任何既有程式、沒有改 PX4 參數。

- 位置：`src/forward_move/`（node `forward_move/forward_move_node.py`、launch `launch/forward_move.launch.py`、`command.txt`）
- 流程：PREFLIGHT → READY → ARMING → TAKEOFF → HOVER（`hover_time`）→ **FORWARD** → HOLD → LANDING（**在終點降落**）→ DONE
- 設計：`ForwardMoveNode` 繼承 `takeoff_land.takeoff_land_node.TakeoffLandNode`。所有參數、GPS/EKF 門檻、RC 模式開關與 Kill 檢查、解鎖/OFFBOARD、`_wait_altitude`、`_controlled_descent`、AUTO.LAND 交接、所有中止處理都是直接呼叫 `takeoff_land` 的方法（改 `takeoff_land` 後重新 build 就會生效）
- **唯一複製的是 `run_sequence()` 的呼叫順序**（原版在 HOVER 與 LANDING 之間沒有掛點）。與原版不同的每一行都標 `FORWARD`。**`takeoff_land_node.run_sequence()` 以後有改，這裡要同步**
- 前進方向：鎖定 home 時機頭的 yaw（ENU），整趟 yaw 不變；XY setpoint 以 `forward_speed` 從起點滑到 起點 + `forward_distance` ×（cos yaw, sin yaw），高度維持 takeoff_z
- `max_pos_error` 的意義：TAKEOFF/HOVER 量離 home；FORWARD 量離移動中的 setpoint（追蹤誤差）；HOLD/LANDING 量離終點。超過 → AUTO.LAND（同 `takeoff_land`）
- 起飛爬升 30 秒內沒穩定 → 跳過 FORWARD，原地降落
- **沒有避障**，深度相機沒用到；機頭前方 `forward_distance` + 2 m 要淨空
- 新 ROS 參數：`forward_distance`（2.0 m，上限 10）、`forward_speed`（0.3 m/s，0.05–1.0）、`forward_hold_time`（3.0 s）。其餘參數名稱與預設值同 `takeoff_land.launch.py`
- launch 啟動時會 `pkill` `takeoff_land_node` 與 `forward_move_node`（避免兩個 setpoint 串流同時存在），`px4_sensor_reader` 用 `takeoff_land` 的

**驗證**：py_compile、pyflakes 無問題；`run_sequence` 去掉註解後與原版 diff，只有 FORWARD 行不同；離線邏輯測試 24/24 通過（不啟動 MAVROS、node 不 spin、不送 setpoint：方向、速度、終點、Kill/解鎖/RC/斷線中止、追蹤誤差超限觸發 sanity abort）；`colcon build --packages-select forward_move` 成功，install 與 src 一致。

**尚未做**：拆槳測試（先確認 Kill 在 ARMING 後仍有效）、實飛。建議第一次 `forward_distance:=1.0`。

### 2026-09-13：新增三個氣壓計高度套件 + `fm_deploy` 沿原路返航的自動處理

使用者要求（22:32）：GPS 高度漂移、數值不合理，換新 GPS 前不能依賴；新增 `fm_deploy_barometer`、`takeoff_land_barometer`、`forward_move_barometer`，**z 用氣壓計、x/y 仍用 GPS**；另外 `fm_deploy` 起飛成功後若程式出錯導致自動降落，系統要能安全處理。

**本日 log 回顧**（`~/.ros/log`，共 41 次啟動）：11:5x–12:4x `takeoff_land` 六次全被 EKF 門檻拒絕（z std 0.08–0.33、5 秒漂 0.2–0.4 m、|z| 1–6 m）；13:25、13:59 氣壓計高度基準時 10 秒內穩定（std 0.01–0.05）；14:36 `takeoff_land` 實飛成功（`max_ground_z:=100`，ground_z −3.33），降落觸地時 EKF alt −0.44 → 一分鐘漂 0.4 m；15:05 `fm_deploy` 實飛 OFF-MAP 誤判降落（已修）、40 秒漂 0.9 m；15:24–15:51 五次 `fm_deploy` 全被拒（std 0.16–0.45、5 秒漂 0.5–1.4 m）；22:07 室內 dry run 無 GPS 時 z 45 秒漂 0.33 m（氣壓計暖機）。**PX4 參數今天最後仍是 GPS 高度（`EKF2_HGT_REF` 1、`EKF2_GPS_CTRL` 7），本次沒有改任何 PX4 參數、沒有改任何既有套件。**

**實測氣壓計（22:41，MAVROS 開 100 秒錄 `/mavros/altitude`，室內、只接 USB）**：`monotonic` 10 Hz = PX4 `baro_alt_meter`（純氣壓，不是 EKF）；原始白雜訊 0.085 m RMS（5 秒峰對峰 0.4 m 是雜訊不是漂移）；濾 1 秒後約 3 cm。`/mavros/imu/static_pressure` 只有 0.01 hPa 解析度（≈ 8 cm）不用。

#### 作法（不改 PX4 參數，只在機載程式閉迴路）

- 共用模組 `src/takeoff_land_barometer/takeoff_land_barometer/baro_altitude.py`：`BaroAltitudeEstimator` + `BaroHoldMixin`。每一筆位置 setpoint 的 z 改送 **`z_sp = z_ekf + baro_gain × (目標離地高 − 氣壓高)`**，PX4 看到的位置誤差 = 氣壓高度誤差；GPS 慢漂或跳變讓 z_ekf 變，setpoint 同步變、PX4 不動。x/y 不動
- 氣壓高 = `monotonic` 濾波（`baro_tau_s` 1.0）− 解鎖前地面平均。離地 < `baro_ground_effect_alt`（1.0 m）不用氣壓計（槳下洗），改用 EKF z 相對最後可信氣壓值推算（EKF-LOCK 模式，起飛頭一公尺、降落最後一公尺行為同原程式）；模式切換用同一估計值做滯後 + 至少 1 秒停留，切換的差量 2 秒淡出；氣壓計斷流 > 1 秒退回 EKF 推算、> `baro_stale_abort_s`（5 s）中止（AUTO.LAND）
- 起飛前地面基準門檻改成：氣壓（濾波）rate ≥ 5 Hz、std < 0.08、5 秒峰對峰 ≤ 0.30；EKF z std < `ekf_gate_std` 0.15（原 0.08）、5 秒峰對峰 ≤ `max_ground_drift` 0.5（原 0.2）、`max_ground_z` 1000（原 1.0，絕對 z 對本法無意義）。**沒過門檻仍拒飛**：PX4 自己的垂直速度仍含 GPS，EKF z 劇烈抖動時本法救不了
- 三個新套件都只是 **繼承** 原節點（`TakeoffLandNode` / `ForwardMoveNode` / `FMInferenceRealNode`），任務流程、所有 abort、Kill、RC 檢查都是原碼；原套件改了、重 build 後這裡跟著生效。launch 檔是複製版（`fm_real_barometer.launch.py` 複製自 `fm_real.launch.py`，標 `BARO`），原 launch 改了要同步
- 限制：氣壓計會隨天氣慢漂（幾分鐘內小）、暖機（開機 2 分鐘 −3.5 m，**開電至少 3 分鐘再起飛**）；`fm_deploy` 的 octomap 帶在 EKF odom 座標，只往上加寬 `band_drift_margin` 0.5 m（往下加寬地面會進帶），**建議 `target_alt` ≥ 2.0**；深度 gate `gate_alt_margin` 預設改 1.5（它讀 EKF 高度，現在預期會漂）。仿真閉迴路：`baro_tau_s` ≤ 1.5 才穩，3.0 會晃 0.38 m

| 套件 | 節點 / launch | 相對原套件的差異 |
| --- | --- | --- |
| `takeoff_land_barometer` | `takeoff_land_baro_node` / `takeoff_land_barometer.launch.py` | `_wait_ekf_stable`（地面基準）、`_wait_altitude`、`_sanity_check`（高度用氣壓）、狀態列 `balt=`、setpoint publisher 包一層改 z；新參數 `baro_*`、`ekf_gate_std` |
| `forward_move_barometer` | `forward_move_baro_node` / `forward_move_barometer.launch.py` | 同上套在 `ForwardMoveNode` 上（FORWARD 追蹤誤差限制不變） |
| `fm_deploy_barometer` | `fm_inference_baro_node`、`fm_inference_recovery_node` / `fm_all_barometer.launch.py`、`fm_real_barometer.launch.py`（`use_baro:=false` 選後者 = EKF 高度 + 只加返航） | 同上 + `_wait_altitude_real`、`_watchdog` 高度改用氣壓（原 EKF z 對 cruise_z 的檢查關掉，否則預期中的漂移會誤判 ABORT）、octomap 帶上加寬、`band_drift_margin`；**返航復原**見下 |

#### `fm_deploy` 出錯後的處理（`fm_deploy_barometer/recovery.py`，`ReturnHomeRecoveryMixin`）

原本任何中止都「就地降落」（15:05 就是）。現在任務停止時先分類：

| 停止原因 | 處理 |
| --- | --- |
| 規劃器問題：`drone off-map`、`STUCK`、任務逾時（目標未達） | **RETURN**：沿 FLYING 時每 0.3 m 記的麵包屑倒退回 home（起飛點），`rth_speed` 0.3 m/s、巡航高、機頭朝行進方向讓相機持續更新地圖，到 home 懸停 2 秒後在 home 降落 |
| 機體問題：`GEOFENCE`、`ALTITUDE`、`BATTERY`、意外上鎖、氣壓計斷流 | 就地降落（同原本） |
| 駕駛：RC 接管、斷線、Kill | 立刻停 setpoint（同原本） |
| 已到目標 / 離 home < `rth_min_dist` 1 m / 沒有軌跡 / `rth_enabled:=false` | 就地降落 |

返航途中每 20 Hz 檢查，任一失敗 → 就地降落：ESDF 已知障礙（含虛擬圍欄）離機身或前導點 < `rth_min_clearance` 0.5 m → 停住，連續 `rth_block_s` 8 s 仍擋 → 降落；機身落後前導點 > 1.5 m 達 3 s（風、GPS 跳）→ 降落；時間預算 2 × 路長 / 速度 + 30 s；地理圍欄半徑、電池、上鎖照舊；Kill / RC / 斷線 → `_hard_stop`。yaw 以 45°/s 滑到航向、轉到 30° 內才前進；前導點最多領先 0.6 m。搜尋 `[RTH]`。

**驗證（離線，不開 MAVROS、不解鎖）**：py_compile、pyflakes 全乾淨；三個 launch 檔可解析；`BaroAltitudeEstimator` 單元 + 閉迴路仿真（PX4 位置環 P=1、vz ±0.5、氣壓雜訊 0.085 m）32/32：GPS 60 秒漂 1.5 m 時實體高度誤差 < 0.06 m、±1.5 m 跳變不動、增益 1.0 仍穩；返航 mixin 35/35（分類表、L 形軌跡返家 22 s、途中障礙 → 就地降、追不上 → 就地降、返航中 Kill → hard stop、`rth_enabled:=false`、`FMInferenceBaroNode` 用 `.onnx` 實例化 + watchdog/status 可跑）；假 PX4 整段任務六個情境見下。測試檔在各套件 `test/offline_*.py`（`/usr/bin/python3` 跑，不用 conda）。

**假 PX4 整段任務（同程序內跑真正的 `TakeoffLandBaroNode` / `ForwardMoveBaroNode` + 假 PX4：EKF z = 真高 + 腳本化 GPS 偏差、氣壓 = 真高 + 0.085 m 雜訊 + 地效、PX4 位置環 P=1、vz ±0.5；不開 MAVROS、不解鎖真機）**：

| 情境 | 結果 | 懸停段真實高度（目標 2.0） | 懸停段 EKF z 變化 |
| --- | --- | ---: | ---: |
| EKF 每秒漂 −0.025 m + 0.15 m 擺動（老門檻會拒飛） | DONE、AUTO.LAND 上鎖 | 1.95–2.05 m | 0.29 m |
| 懸停中 EKF 跳 +1.5 m、再跳 −2.5 m | DONE | 1.90–2.06 m | 2.58 m |
| forward_move 1.5 m、EKF 每秒漂 +0.02 m | DONE、終點誤差 0.07 m | 1.94–2.03 m | 0.16 m |
| EKF 地上 −45 m（目標 1.5） | DONE（老門檻 `max_ground_z` 會拒） | 1.45–1.55 m | 0.10 m |
| 懸停中氣壓計斷流 | 5 秒後 `SANITY: BAROMETER STREAM LOST` → AUTO.LAND | — | — |
| 懸停中撥 Kill | `[KILL]` → AUTO.LAND → 上鎖，程式結束（繼承路徑正常） | — | — |

注意：起飛完成那行 `[TAKEOFF] reached target altitude, alt=…` 是繼承的 `run_sequence` 印的 **EKF** 高度（會跟 GPS 偏），看氣壓高請看狀態列 `balt=`。

**Build（23:1x，使用者同意）**：`colcon build --packages-select takeoff_land_barometer forward_move_barometer fm_deploy_barometer` 成功（用 `/usr/bin/python3`，非 conda）；install 內 12 個 .py / launch 檔與 src 逐檔相同，4 個 executable 註冊正常，4 個 launch 檔 `--show-args` 正常。使用者要求後，三個 `command.txt` 都加了「PX4 也改用氣壓計」一節：`EKF2_HGT_REF` 0 + `EKF2_GPS_CTRL` 5 的寫法（`ros2 param set /mavros/param …` 或 QGC）、前提（未解鎖、拆槳）、寫完要斷電重開 Pixhawk 並暖機 3 分鐘、還原成 1 / 7 的指令。**這些參數本次沒有寫入，仍是 GPS 高度。**

**尚未做（都需要使用者同意）**：拆槳測試（Kill 流程繼承自原碼但要再確認一次）；低空懸停實飛看 `balt=` 對 `ekf_alt=`；`fm_all_barometer.launch.py dry_run:=true` 看 `Ground reference OK`、`Octomap band ... +0.5 m drift margin`；返航實測建議 `goal_dist:=4.0 mission_timeout_s:=40`（逾時是可復原停止，應沿路飛回 home 降落）；知識庫同步。`~/drone_ws_jeremy` 個人副本沒有這三個套件，要的話 `rsync -a ~/drone_ws/src/{takeoff_land_barometer,forward_move_barometer,fm_deploy_barometer} ~/drone_ws_jeremy/src/` 再 build。

**還原**：三個套件都是新目錄，刪掉即可（`rm -rf src/takeoff_land_barometer src/forward_move_barometer src/fm_deploy_barometer` + 刪 `install/` `build/` 對應目錄）；本檔備份 `CLAUDE.md.bak-BARO-20260913-230718`。

### 2026-09-14 18:4x：GPS 版起飛前 `max_ground_z` 預設 1.0 → 1000（使用者要求）

**背景（同日 log 分析）**：z = 0 是 Pixhawk 開機時 EKF 初始化那一刻的高度，不是地面；GPS 高度靜置漂 1–6 m，`|z| < 1 m` 常拒絕解鎖。7/16 三次 `takeoff_land` 飛行也一樣漂（地上 EKF z 峰對峰 0.8–5.8 m，= GPS 高度逐 cm 相同），當時舊版只看 1 秒 std 所以放行；「地上 0.4–0.5」是 `[ALT] Rel=`（home 在上鎖時會被 PX4 更新）。9/14 12:24/12:33 barometer 版 fm 飛行上下晃：切到 BARO 後 `balt` 2 秒跳 +1.5 m（推測螺旋槳氣流、Pixhawk 氣壓計沒海綿，未用 ULog 確認），節點照氣壓計壓低 setpoint。使用者決定暫用 GPS 版 `target_alt:=3.0`。9/14 12:41 `forward_move_baro` 使用者看到往左右而非往前；EKF 自認前進 1.5 m、側偏 ≤ 0.14 m、yaw 110° ENU（羅盤 ~340°）→ 懷疑羅盤航向錯（磁干擾 / `CAL_MAG0_ROT`），**未查證**。9/13 23:xx、9/14 00:17、16:00 的 `[FWD]` log 是離線假 PX4 測試，不是實飛。

- 檔案：`src/takeoff_land/launch/takeoff_land.launch.py`、`src/forward_move/launch/forward_move.launch.py`、`src/fm_deploy/launch/fm_all.launch.py`、`fm_real.launch.py`（只改 launch 預設；node 內預設仍 1.0，直接 `ros2 run` 仍是 1.0）。搜尋 `NEW (2026-09-14): default 1.0 -> 1000.0`
- 備份：同目錄 `*.bak-GNDZ-20260914-184048`；本檔 `CLAUDE.md.bak-GNDZ-20260914-184048`
- 其他地面檢查不變：EKF z std、`max_ground_drift` 5 秒峰對峰、GPS 門檻（這些才是擋 8/27 那種情況的）。barometer 版本來就是 1000，沒動
- **驗證**：`colcon build --packages-select takeoff_land forward_move fm_deploy` 成功；install 內 4 個 launch 與 src 相同；四個 launch `--show-args` 的 `max_ground_z` 預設 `1000.0`。未實飛
- **還原**（需使用者同意；或不還原、啟動時加 `max_ground_z:=1.0`）：

```bash
cd ~/drone_ws/src && T=bak-GNDZ-20260914-184048
for f in takeoff_land/launch/takeoff_land.launch.py forward_move/launch/forward_move.launch.py fm_deploy/launch/fm_all.launch.py fm_deploy/launch/fm_real.launch.py; do cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select takeoff_land forward_move fm_deploy
```

### 2026-09-14 23:05：PX4 高度基準再改為氣壓計（使用者要求「整個 z 系統改用氣壓計」）

背景：9/14 `_barometer` 套件的飛行中，PX4 EKF 仍是 GPS 高度（本次讀到的舊值 `EKF2_HGT_REF` 1、`EKF2_GPS_CTRL` 7 證實），只有 Jetson 程式用氣壓計修正 setpoint。使用者決定 PX4 本身也改用氣壓計；x/y 仍用 GPS。

#### PX4 參數（Claude 經使用者要求寫入；飛機未解鎖 `armed: false`，MAVROS 另開於 23:0x、寫完已關閉；是否拆槳未確認）

| 參數 | 舊值 | 新值 | 原因 |
| --- | ---: | ---: | --- |
| `EKF2_HGT_REF` | 1（GPS） | **0（Baro）** | z 基準改氣壓計 |
| `EKF2_GPS_CTRL` | 7 | **5** | 關 GPS 高度（bit 1），保留水平位置 + 3D 速度；兩個都要改（見 9/13 corr +0.99） |

其他讀到的值（沒改）：`EKF2_BARO_CTRL` 1、`EKF2_RNG_CTRL` 1。

- 驗證：`ros2 param set` 兩個都回 `Set parameter successful`（MAVROS 收到 FCU 回傳的 PARAM_VALUE）；之後 `force_pull` 5 次都 `success=False`（USB 收到 968–1045 筆，不完整），`param get` 讀到 0 / 5。**完整讀回尚未確認 → Pixhawk 斷電重開後再讀一次**
- 程式碼沒改。一般套件與 `_barometer` 套件都可用（`_barometer` 相容，EKF z ≈ 氣壓計，修正量小）。**`fm_deploy*` 不要 `write_px4_params:=true`**（會寫回 1）
- 已知風險：9/14 螺旋槳轉動時 `balt` 跳 +1.5 m（推測氣流，未用 ULog 確認）→ 現在 PX4 本身也會跟著；Pixhawk 氣壓計要加海綿。開電暖機 ≥ 3 分鐘。這個組合**從未實飛過**（9/13 13:25 只解鎖 1 秒就 Kill）

**尚未做**：拔電池 + USB 重開 Pixhawk、暖機 3 分鐘後讀回參數；90 秒靜置測試（EKF z 峰對峰 < ~0.5 m、不跟 GPS、corr(z, Baro) > 0）；拆槳 `takeoff_land` 確認 `EKF stable` 與 Kill；低空懸停實飛看起降地效彈跳並取 ULog（`sensor_baro`、`estimator_states`）；知識庫同步。

**還原**（需使用者同意）：`ros2 param set /mavros/param EKF2_HGT_REF 1`、`ros2 param set /mavros/param EKF2_GPS_CTRL 7`，再斷電重開 Pixhawk。本檔備份 `CLAUDE.md.bak-BAROPX4-20260914-230839`。

### 2026-09-15：`fm_deploy` FLYING 時 yaw 鎖定 home 方向（修「原地打轉 / scanning」）

**事件（9/15 00:53 `fm_all_barometer`，goal 8 m、alt 2.5、guard 關）**：起飛懸停正常，FLYING 後使用者看到飛機像在原地掃描。`px4_sensor_reader` 每秒 yaw/位置：T+56–77 yaw 102→75→170→89→46→0→116°（home→goal 方向 102° ENU），邊轉邊來回移動（每秒 0.1–0.39 m，約一半步伐方向與機頭差 > 45°，淨前進 ≈ 0.9 m）；T+67–85 replan 連續超過 1.0 s 上限、`clear` 降到 0.97 m，8 次失敗後清地圖；T+82–89 沿機頭 178° **確實照軌跡**往左側飛 1.7 m（方向與 yaw 差 ≤ 5°）；T+90–122 機頭停在 −175°（離 goal 方向 83°，相機 FOV ±45° 看不到前方路徑），`Replan ok` 每秒一次但速度 0.02–0.06 m/s，30 s 只前進 0.7 m（軌跡太慢或 GPS 漂移，**未確認**，要錄 `/mavros/setpoint_raw/local`）；T+01:55 使用者切 Position。同晚 00:49 goal 2 m 成功（yaw 穩定 106–118°）；00:44 是 `~/drone_ws_jeremy` 版本 + guard 開，一直 `HOLDING - no trajectory`。**另：該次電池起飛 28%、飛行中 19% LOW BATTERY，banner `Battery : disabled`。**

**原因**：`fm_inference_base._publish_cmd` FLYING 時 `yaw = atan2(軌跡速度)`、無轉速限制；FM 模型雙峰（`GATE two-sided 33–35%`），每次 replan 左右換邊 → yaw 目標跳動 → 相機看到新區域 → 地圖/規劃再變。

#### 程式修改（使用者同意：選項 A）

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_real_node.py`（`fm_inference_base.py` 沒動，md5 `edae0738…` 不變）
- 備份：`fm_inference_real_node.py.bak-YAWLOCK-20260915-084929`；本檔 `CLAUDE.md.bak-YAWLOCK-20260915-084929`
- 新類別 `_FlyingYawLockPublisher` 包住 `_pub_sp`：`_mission_state == FLYING`、非 escape、`_home_locked` 時把 `msg.yaw` 換成 `_home_yaw`；位置、速度、z 不動。其他階段（TAKEOFF/HOVER/LANDING 本來就是 home yaw）、escape、`[RTH]` RETURN 的 yaw 不變。barometer 版鏈為 `BaroZPublisher → _FlyingYawLockPublisher → publisher`
- 新 ROS 參數 `flying_yaw_mode`：`home`（預設）/ `velocity`（舊行為）。launch 檔沒加這個參數（要切回舊行為需加 launch 參數或還原檔案）；banner 多一行 `Flying yaw : ...`。搜尋 `NEW (2026-09-15)`
- 限制：機頭固定朝 goal，側向繞行時相機看不到側邊；goal 在正前方的任務適用

**驗證（離線，不開 MAVROS、node 不 spin、不解鎖）**：py_compile OK；pyflakes 只有舊的一條 f-string；src 版 13/13（`FMInferenceRealNode` FLYING 速度朝東仍送 102°、位置/速度不變、type_mask 沒 IGNORE_YAW、escape 不鎖、TAKEOFF 照舊、dry_run 不送；`velocity` 模式 = 舊行為；`FMInferenceBaroNode` 包裝順序正確、FLYING 鎖定、RETURN 保留 RTH yaw）。測試腳本在 Claude scratchpad（未放進 repo）。

**Build（9/15 08:5x，使用者同意）**：`colcon build --packages-select fm_deploy fm_deploy_barometer` 成功（排除 conda PATH，shebang `/usr/bin/python3`）；install 內 `fm_inference_real_node.py` 與 src 相同、`fm_inference_base.py` md5 不變；用 install 版重跑離線測試 13/13。

**尚未做（需使用者同意）**：`dry_run:=true` 看 banner `Flying yaw : HOME yaw locked`；實飛（先換滿電、建議錄 bag）；知識庫同步。

**還原**（需使用者同意）：

```bash
cd ~/drone_ws/src/fm_deploy/fm_deploy
cp fm_inference_real_node.py.bak-YAWLOCK-20260915-084929 fm_inference_real_node.py
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy fm_deploy_barometer
```

### 2026-09-15：build `vfh_avoidance_barometer`（使用者要求）

- 套件 `src/vfh_avoidance_barometer/`（9/14 22:15–22:52 建立，**不是本 Claude 對話寫的**，內容未審查；程式碼這次沒改）
- `colcon build --packages-select vfh_avoidance_barometer` 成功（排除 conda PATH，shebang `/usr/bin/python3`）；之前 install 內沒有這個套件
- 驗證：py_compile OK；install 內 5 個 .py、2 個 launch（`vfh_all_barometer`、`vfh_perception`）與 src 相同；executables `vfh_avoidance_node`、`vfh_flight_baro_node`；兩個 launch `--show-args` 正常。沒有啟動 node、沒有跑離線測試（`test/offline_*.py`）
- **尚未做**：審查程式、離線測試、dry run、實飛；知識庫同步
- 還原：`rm -rf install/vfh_avoidance_barometer build/vfh_avoidance_barometer`

### 2026-09-15 20:2x：`fm_deploy` FLYING yaw 新增 `smooth` 模式（使用者同意；已 build）

**背景（9/15 上午 5 次 `fm_all_barometer` 實飛 log）**：`home` yaw 鎖定有效（09:52 yaw 82–85°、10:07 66–68°），但飛機側移可達 4 m（09:54:31–39 x −3.5 → +0.55），相機 FOV ±45° 看不到移動方向。舊 `velocity` 模式會原地打轉（9/15 00:53）。同批 log 其他發現（未處理）：10:07、10:11 兩次 OFFBOARD → AUTO.LOITER → 5 s → AUTO.LAND 時電池 4% / 0%（推測 PX4 電池 failsafe，**未查證**；node banner `Battery : disabled`，10:11 起飛時已 14% LOW）；goal 12/20 m 超出 geofence（fwd 10 / radius 12，09:54 GEOFENCE abort）；氣壓計飛行中與 EKF 差 0.6–0.9 m（09:42 ALTITUDE abort），之後 `max_alt_error` 被加到 1.5 m（目標 1.2–1.5 m 時等於不檢查過低）；v_max 0.8 時 replan 500–1000 ms、多次超過 1.0 s 上限；戶外深度有效像素 3–20%。

#### 程式修改

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_real_node.py`、`src/fm_deploy/launch/fm_all.launch.py`、`fm_real.launch.py`、`src/fm_deploy_barometer/launch/fm_all_barometer.launch.py`、`fm_real_barometer.launch.py`（`fm_inference_base.py` 沒動，md5 `edae0738…`）
- 備份：同目錄 `*.bak-YAWSMOOTH-20260915-202550`；本檔 `CLAUDE.md.bak-YAWSMOOTH-20260915-202550`
- 搜尋 `YAWSMOOTH`
- `_FlyingYawLockPublisher` 新增 `flying_yaw_mode:=smooth`（**預設仍 `home`**）。只在 FLYING、非 escape、home 已鎖、未設 IGNORE_YAW 時改 `msg.yaw`；位置/速度/z 不動
- smooth 演算法（相對 home yaw 的偏角）：軌跡速度方向 → 限制在 ±`yaw_max_offset_deg`（60）→ 低通 `yaw_smooth_tau_s`（1.0）→ 死區 `yaw_deadband_deg`（15，遲滯：小擺動不轉；超過後跟到濾波穩定 < 2° 為止）→ 轉速上限 `yaw_rate_max_dps`（30）。速度 < `yaw_min_speed`（0.15 m/s）或往後飛（> 120°）保持目前朝向。進入 FLYING（或中斷 > 0.5 s）從目前機頭開始。參數夾限：tau 0–5、rate 5–90、min speed 0.05–1、offset 0–90、deadband 0–45；未知 mode → home
- 四個 launch 都加 `flying_yaw_mode` + 5 個 `yaw_*` 參數（`fm_all_barometer` 經 `_FORWARDED`）；banner `Flying yaw : SMOOTH follow path | ...`
- 沒做提案中「偏角 > 45° 時減速」（可選項）

**驗證（離線，不開 MAVROS、不 spin、不解鎖；腳本在 Claude scratchpad `test_yawsmooth.py`）**：py_compile OK；pyflakes 只有舊的一條 f-string；src 版 27/28：home/velocity 行為不變、轉速 ≤ 30°/s、+40° 路徑 2.1 s 內到 5° 內並收斂到 123.0°、側向 90° 夾到 60°、往後飛保持、±30° 每秒雙峰切換擺幅 27.7° 峰對峰（velocity 模式 60°）、±10° 擺動機頭不動、低速保持、escape/LANDING 不改、重新進 FLYING 從機頭開始、±180° 繞回無跳動、IGNORE_YAW 不改、例外時仍送 setpoint、`FMInferenceRealNode` 參數讀取/夾限/banner/`_publish_cmd`；唯一 FAIL 是 `FMInferenceBaroNode` 讀到 home — 它 import 的是 **install 內舊版** `fm_inference_real_node.py`（已確認路徑），build 後應通過。四個 launch（src 路徑）`--show-args` 有新參數。

**Build（20:3x，使用者同意）**：`colcon build --packages-select fm_deploy fm_deploy_barometer` 成功（排除 conda PATH，shebang `/usr/bin/python3`）；install 內 real node、base（md5 不變）、4 個 launch 與 src 相同；用 install 版重跑 `test_yawsmooth.py` **28/28**（`FMInferenceBaroNode` smooth 讀取 OK，鏈 `BaroZPublisher → _FlyingYawLockPublisher → Publisher`）；`fm_deploy_barometer/test/offline_test_recovery.py` 35/35；4 個已安裝 launch `--show-args` 各有 6 個新參數。

**尚未做（需使用者同意）**：`dry_run:=true flying_yaw_mode:=smooth` 看 banner；實飛（滿電、`goal_dist` ≤ 8 m、v_max 0.3–0.5）；知識庫同步。

**還原**（需使用者同意；或不還原，`flying_yaw_mode` 保持預設 `home` 即舊行為）：

```bash
cd ~/drone_ws/src && T=bak-YAWSMOOTH-20260915-202550
for f in fm_deploy/fm_deploy/fm_inference_real_node.py fm_deploy/launch/fm_all.launch.py fm_deploy/launch/fm_real.launch.py \
         fm_deploy_barometer/launch/fm_all_barometer.launch.py fm_deploy_barometer/launch/fm_real_barometer.launch.py; do cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy fm_deploy_barometer
```

### 2026-09-15 21:2x：所有 `_barometer` 套件 `baro_gain` 預設 0.7 → 0（使用者要求「整個系統 gain 0」；已 build）

**背景（9/15 log 分析）**：PX4 高度基準已是氣壓計（9/15 00:30 `drone_ws_jeremy` 版 `takeoff_land_baro` 從飛控讀到 `EKF2_HGT_REF=0 EKF2_GPS_CTRL=5 EKF2_BARO_CTRL=1`；5 次 fm 地面基準 offset ekf−baro 30 分鐘內恆為 −140.1 ± 0.1 m，氣壓計隨天氣升 1.45 m、EKF 同步）。`~/drone_ws` 的 9/13 氣壓計迴路（`z_sp = z_ekf + 0.7 × (target − 原始氣壓高)`）等於同一顆氣壓計用兩次：EKF z std（HOVER/前進）gain 0（jeremy 版 00:26–00:40）0.01–0.03 m；gain 0.7（drone_ws）凌晨 2.0–2.5 m 高 0.03–0.12，早上 fm 1.5 m 高 0.10–0.30（範圍到 1.1–1.3 m）、09:41 ALTITUDE abort。使用者目視懸停與移動時上下晃。凌晨/早上差異另含目標高度（1.5 m 靠近 1.0 m 地效切換）、v_max 0.8、日照風、電量（推測，未用 ULog 確認）。

**注意：舊程式直接 `baro_gain:=0` 很危險** — `z_ekf + 0 × …` = 永遠命令「停在原地」，起飛不會爬升到目標。所以改程式碼而不只是改參數。

#### 程式修改

- 檔案：`src/takeoff_land_barometer/takeoff_land_barometer/baro_altitude.py`、`launch_common.py`（`takeoff_land_barometer`、`forward_move_barometer`、`vfh_avoidance_barometer` 三個 launch 共用）、`src/fm_deploy_barometer/launch/fm_all_barometer.launch.py`、`fm_real_barometer.launch.py`
- 備份：同目錄 `*.bak-GAIN0-20260915-212730`；本檔 `CLAUDE.md.bak-GAIN0-20260915-212730`
- 搜尋 `GAIN0`
- `BaroAltitudeEstimator`：gain ≤ 0 → `altitude()` 回傳 EKF z − 地面 EKF z（新模式 `EKF`），所以起飛到達判定、`_sanity_check` / fm `_watchdog` 高度限制、狀態列 `balt=` 全部用 EKF 高度；`z_setpoint()` 回傳 `z_ground + target`（名目值）；`BaroZPublisher` gain ≤ 0 時不改 setpoint（`rewritten` 0）
- 仍保留：起飛前氣壓計地面閘門（std/spread/rate）、氣壓計斷流 5 s abort、EKF 地面閘門、fm octomap 帶上加寬 0.5 m、RTH
- 節點參數 `baro_gain` 預設與 4 處 launch 預設 0.7 → 0.0（`BaroAltitudeEstimator` 建構子預設仍 0.7，只有測試直接用）。要回舊行為：`baro_gain:=0.7`
- banner 多一行 `[BARO] baro_gain=0: NO barometric loop ...`
- `~/drone_ws_jeremy` 沒動

**驗證（離線，不開 MAVROS、不解鎖；腳本 Claude scratchpad `run_gain0.py`，用 repo 的 `offline_fake_px4.py` + 真的 `TakeoffLandBaroNode` / `ForwardMoveBaroNode`）**：py_compile、pyflakes 乾淨。gain 0（EKF = 真實高度，模擬 PX4 用氣壓計）：`tl_nominal` DONE 懸停真實高 2.00、`rewritten` 0；**氣壓計跳 +1.2 m / −0.8 m + 慢漂** DONE 懸停 1.998（同情境 gain 0.7 → `SANITY: Altitude deviation baro alt=2.54` AUTO.LAND）；`forward_move` DONE；不給 `baro_gain`（預設）DONE 2.00；EKF −45 m DONE 1.50；氣壓計斷流 → `BAROMETER STREAM LOST` AUTO.LAND；Kill → `[KILL]` AUTO.LAND 上鎖。回歸（repo 6 個情境 + `baro_gain=0.7`）結果同 9/13：nominal / jump / fm / bigoffset DONE（懸停 1.92–2.06），baro_lost、kill 照舊中止。`offline_test_estimator.py` 32/32；`fm_deploy_barometer/test/offline_test_recovery.py` 35/35（出現 gain 0 banner）。`fm_all_barometer` / `fm_real_barometer`（src）`--show-args` `baro_gain` default `0.0`。

**Build（21:3x，使用者同意）**：`colcon build --packages-select takeoff_land_barometer forward_move_barometer fm_deploy_barometer vfh_avoidance_barometer` 成功（排除 conda PATH，shebang `/usr/bin/python3`）；install 內 `baro_altitude.py`、`launch_common.py`、兩個 fm launch 與 src 相同；已安裝 5 個 launch（`takeoff_land_barometer`、`forward_move_barometer`、`fm_all_barometer`、`fm_real_barometer`、`vfh_all_barometer`）`--show-args` `baro_gain` default `0.0`；import 自 install 確認含 GAIN0；build 後重跑：estimator 32/32、recovery 35/35、假 PX4 任務 default / 氣壓計跳動 / forward_move DONE（真實高 1.998、`rewritten` 0）、Kill → 上鎖。

**尚未做（需使用者同意）**：實機 dry run 看 banner；低空懸停實飛（`target_alt` ≥ 2.0）；知識庫同步。

**還原**（需使用者同意；或不還原，啟動時加 `baro_gain:=0.7`）：

```bash
cd ~/drone_ws/src && T=bak-GAIN0-20260915-212730
for f in takeoff_land_barometer/takeoff_land_barometer/baro_altitude.py takeoff_land_barometer/takeoff_land_barometer/launch_common.py \
         fm_deploy_barometer/launch/fm_all_barometer.launch.py fm_deploy_barometer/launch/fm_real_barometer.launch.py; do cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select takeoff_land_barometer forward_move_barometer fm_deploy_barometer vfh_avoidance_barometer
```

### 2026-09-15 22:1x：`fm_deploy` MINCO 速度可行性權重 `w_feasibility`（修「FLYING 中一直 replan 但幾乎不動」；已 build）

**原因（9/15 早上 log + 離線）**：MINCO 權重 `[energy 1, time 1, feasibility 1, collision 1e4]`，超速懲罰太弱，optimizer 規劃速度約 v_max 的 2–3 倍；`_install_trajectory` 以 k = peak / v_max 把整條軌跡時間拉長，**起點速度也被除以 k**；下一次 replan 的 head 取自這條已放慢的軌跡，再被新 k 除一次 → 速度被棘輪壓到接近 0。證據：09:44（v_max 0.3）replan cost 中位 9.6 = 同條件離線 optimizer 9.4（T 8.2 s、peak 0.84 m/s、k 2.8、1 s 後只走 3 cm）；10:07（v_max 0.8）6.4 ≈ 離線 6.6（k 1.3）；09:44 FLYING spd 中位 0.07 m/s。純 optimizer 的 receding-horizon 模擬（`sim_replan.py`）v_max 0.3 / replan 1 s → 0.02–0.04 m/s，與實飛相符。另：replan 時間 > `planning_time_ahead` 時 setpoint 往回跳（模擬 38/40 次）。

#### 程式修改

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_base.py`（與模擬共用，**程式預設 1.0 = 行為不變**）、`src/fm_deploy/launch/fm_all.launch.py`、`fm_real.launch.py`、`src/fm_deploy_barometer/launch/fm_all_barometer.launch.py`、`fm_real_barometer.launch.py`
- 備份：同目錄 `*.bak-WFEAS-20260915-220759`；本檔 `CLAUDE.md.bak-WFEAS-20260915-220759`
- 搜尋 `WFEAS`
- 新 ROS 參數 `w_feasibility`（夾在 0.01–1e5），寫入 `cfg.weights[2]`；四個實機 launch 預設 **1000.0**（要回舊行為 `w_feasibility:=1.0`）
- 每次安裝軌跡後 log（每秒最多一行）`[TRAJ] T=…s peak=…m/s scale k=… -> flown over …s | start v=… (head …) | w_feas …`；base 的 DONE 摘要有 k 中位數（real node 自己的 DONE 路徑沒有加）；banner `Replan cost` 行多 `| w_feasibility …`
- 沒改：路徑/避障成本、guard、geofence、abort、RTH、yaw

**驗證（離線，不開 MAVROS、node 不 spin、不解鎖）**：py_compile OK；pyflakes 無新問題（舊 5 條）。**閉迴路 node 測試**（`node_loop_wfeas.py`：src 版 `FMInferenceRealNode` + TensorRT 模型 + 真 ESDF（合成 30×30 m 地圖，空地 / 三根 0.35 m 柱子）+ 真 `_replan()` + 深度全遠 10 m + 類 PX4 位置跟隨執行緒；v_max 0.5、max_candidates 1、budget 1.0 s、guard 關；每組 30 s、goal 12 m）：

| 地圖 | w | replan / ahead | 平均前進 | 速度中位 | 靜止比例 | replan 中位 / p90 / max | 失敗 | k 中位 | 最小中心距 |
|---|---:|---|---:|---:|---:|---|---:|---:|---:|
| 空地 | 1 | 1.0 / 0.3 | 0.136 m/s（4.1 m） | 0.15 | 0 | 104 / 117 / 146 ms | 0 | 1.82 | – |
| 空地 | 100 | 1.0 / 0.3 | 0.363 | 0.43 | 0 | 118 / 129 / 151 | 1 | 1.20 | – |
| 空地 | **1000** | 1.0 / 0.3 | **0.398（29 s 到 12 m）** | 0.50 | 0.05 | 122 / 136 / 149 | 1 | 1.09 | – |
| 柱子 | 1 | 1.0 / 0.3 | 0.063（1.9 m） | 0.08 | **0.31** | 163 / 263 / 341 | 3 | 1.78 | 1.26 m |
| 柱子 | 100 | 1.0 / 0.3 | 0.291 | 0.38 | 0 | 125 / 218 / 640 | 1 | 1.21 | 0.75 |
| 柱子 | **1000** | 1.0 / 0.3 | **0.334（10 m）** | 0.49 | 0.06 | 135 / 303 / 533 | 1 | 1.09 | 0.90 |
| 柱子 | 1 | 2.0 / 0.6 | 0.226 | 0.28 | 0.13 | 136 / 178 / 625 | 1 | 1.78 | 0.75 |
| 柱子 | 1000 | **2.0** / 0.6 | 0.263 | 0.18 | **0.45** | 132 / 213 / 308 | 5 | 1.10 | 0.75 |
| 柱子 | 1000 | 1.0 / **0.6** | 0.522（23 s 到） | **0.64（> v_max 0.5）** | 0 | 129 / 149 / 196 | 0 | 1.08 | 0.84 |
| 空地 | 1000 | 1.0 / 0.6 | 0.519（23 s 到） | **0.65** | 0.07 | 122 / 138 / 179 | 1 | 1.09 | – |

結論：w 1000 + replan 1.0 s 最好；**replan 2.0 s 與 w 1000 搭配反而變差**（軌跡跑完停住）；`planning_time_ahead` 0.6 更快但實際速度超過 v_max 約 30%（setpoint 超前 + PX4 P 項，模型簡化）。replan 時間只多約 15–20 ms。`fm_deploy_barometer/test/offline_test_recovery.py` 35/35；四個 launch（src）`--show-args` `w_feasibility` default `1000.0`。`FMInferenceBaroNode` 參數讀取測試 FAIL — import 的是 install 內舊 base（build 後重測）。

**Build（22:2x，使用者同意）**：`colcon build --packages-select fm_deploy fm_deploy_barometer` 成功（排除 conda PATH，shebang `/usr/bin/python3`）；install 內 base 與 4 個 launch 與 src 相同；已安裝 4 個 launch `--show-args` `w_feasibility` default `1000.0`。install 版重跑：`FMInferenceBaroNode` 參數讀取 3/3（預設 1.0、1000、−5 → 夾到 0.01）、recovery 35/35、yaw smooth 28/28；閉迴路（install 版 real node）w 1000 / replan 1.0 / ahead 0.3：柱子 0.376 m/s（11.3 m，速度中位 0.49，靜止 0，replan 139 / 230 / 556 ms）、空地 0.393 m/s（11.8 m）；ahead 0.6：柱子 0.464（26 s 到）、空地 0.502（24 s 到），速度中位 0.62–0.64 > v_max；`[TRAJ]` log 正常（例：`T=6.0s peak=0.52m/s scale k=1.03 | start v=0.10 (head 0.11)`）。

**尚未做（需使用者同意）**：dry run 看 `[TRAJ] ... k≈1.0–1.2`；實飛（v_max 0.5、goal ≤ 8 m、replan 1.0 s）；知識庫同步。

**還原**（需使用者同意；或不還原，`w_feasibility:=1.0`）：

```bash
cd ~/drone_ws/src && T=bak-WFEAS-20260915-220759
for f in fm_deploy/fm_deploy/fm_inference_base.py fm_deploy/launch/fm_all.launch.py fm_deploy/launch/fm_real.launch.py \
         fm_deploy_barometer/launch/fm_all_barometer.launch.py fm_deploy_barometer/launch/fm_real_barometer.launch.py; do cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy fm_deploy_barometer
```

### 2026-09-16 12:0x：`fm_deploy_barometer` 新增 `rth_after_goal`（任務成功後也沿原路返家降落；已 build）

使用者要求：跑 `fm_all_barometer`（goal 9 m）到達目標後，飛機要回到起飛點再降落。原本 `ReturnHomeRecoveryMixin` 只在**可復原的規劃器中止**（off-map / STUCK / 任務逾時）才 RETURN，`_rth_eligible()` 對「已到目標」明確回 `False`（"goal reached — landing here is the mission"）→ 在目標點降落。

#### 程式修改（使用者同意）

- 檔案：`src/fm_deploy_barometer/fm_deploy_barometer/recovery.py`、`src/fm_deploy_barometer/launch/fm_all_barometer.launch.py`、`fm_real_barometer.launch.py`（`fm_deploy` GPS 版沒動）
- 備份：同目錄 `*.bak-RTHGOAL-20260916-120140`；本檔 `CLAUDE.md.bak-RTHGOAL-20260916-120140`。搜尋 `RTHGOAL`
- 新 ROS 參數 `rth_after_goal`（**預設 False = 舊行為**，三處 launch 預設也是 false）。設 true 時 `_rth_eligible()` 在 `reason is None` 且 `_reached_target`（或離 goal < 1 m）時回 `(True, "goal reached (rth_after_goal)")` → 沿麵包屑倒退回 home、hover `rth_hover_s` 2 秒、在 home 降落
- 其餘 RETURN 行為完全沒動：`rth_speed` 0.3、`rth_min_dist` 1.0（離 home < 1 m 仍就地降）、途中障礙 / 追不上 / 地理圍欄 / 電池 / 意外上鎖 → 就地降落；Kill / RC / 斷線 → `_hard_stop`；時間預算 2 × 路長 / 速度 + 30 s（與 `mission_timeout_s` 無關）
- 啟動時 `rth_after_goal:=true` 會多印一行提醒會增加飛行時間

**驗證（離線，不開 MAVROS、不解鎖）**：py_compile、pyflakes 乾淨；`test/offline_test_recovery.py` 原本 35/35 仍過（預設關閉 → 行為不變）；擴充版（Claude scratchpad `test_rthgoal.py`，= 原檔 + R8）**46/46**：預設仍在目標降落、開啟後 eligible 且分類字串含 goal reached、開啟時 GEOFENCE / Kill 仍就地降、goal 離 home < `rth_min_dist` 不返航、9 m 假飛行 35 s 回到 (0.04, 0.00) 並印 `landing at home`、返航中 Kill → `_hard_stop` 且不再送 setpoint。

**Build（12:0x，使用者同意）**：`colcon build --packages-select fm_deploy_barometer` 成功；install 內 `recovery.py` 與兩個 launch 與 src md5 相同；用 install 版重跑 46/46；兩個已安裝 launch `--show-args` 有 `rth_after_goal`（default `false`）。

**尚未做**：實飛（第一次建議 `goal_dist` 小一點、電池充飽；回程約多 goal_dist / 0.3 秒）；知識庫同步。

**還原**（需使用者同意；或不還原，不加 `rth_after_goal:=true` 即舊行為）：

```bash
cd ~/drone_ws/src && T=bak-RTHGOAL-20260916-120140
for f in fm_deploy_barometer/fm_deploy_barometer/recovery.py fm_deploy_barometer/launch/fm_all_barometer.launch.py \
         fm_deploy_barometer/launch/fm_real_barometer.launch.py; do cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy_barometer
```

### 2026-09-16 12:2x：`rth_mode:=replan`（返家改用 FM planner 重新規劃，不只是倒退走麵包屑）；已 build

**事件（12:14 `fm_all_barometer ... goal_dist:=9.0 rth_after_goal:=true`）**：`[RTH] rth_after_goal:=true` banner 有出現，但到達目標時 `[RTH] no return: vehicle problem (BATTERY) -> land in place`、`(ABORT: BATTERY 19% < 20%)` → 在目標點降落。**不是 bug**：電池被歸類為機體問題。去程 T+00:49 → T+02:22（93 s，前段一直 `HOLDING - no trajectory`，replan 500–2846 ms、`slow x45`），起飛時電池只有 22.5 V。往返要 3–4 分鐘，**必須滿電起飛**。（另注意：狀態列 LANDING 段的 `home=` 變小是因為 `_controlled_descent` 會把 `_home_xy` 改成目前位置當下降定點，不是 home 位置變了。）

使用者要求：返家時也要邊飛邊重新規劃。

#### 程式修改（使用者同意）

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_real_node.py`（**純重構**）、`src/fm_deploy_barometer/fm_deploy_barometer/recovery.py`、`fm_all_barometer.launch.py`、`fm_real_barometer.launch.py`（`fm_inference_base.py` 沒動）
- 備份：同目錄 `*.bak-RTHREPLAN-20260916-122251`；本檔 `CLAUDE.md.bak-RTHREPLAN-20260916-122251`。搜尋 `RTHREPLAN`
- `run_sequence()` 裡「14. FLYING」整段迴圈原封不動搬進新方法 `_fly_to_target(target, timeout_s, tag="FLYING", banner_extra="", timeout_label="mission timeout")`，回傳 `"reached"` / `"abort"` / `"stopped"`（`stopped` = 已經 `_hard_stop`，呼叫端要直接 return）。`run_sequence` 行為不變 → **GPS 版 `fm_deploy` 行為不變**
- `recovery.py` 新參數 `rth_mode`：`trail`（**預設 = 舊行為**，倒退走麵包屑）/ `replan`（新）。`replan` 時走 `_return_home_replan()`：清掉 `_abort_reason`（planner 停止的原因由「飛回家」處理）→ 作廢舊軌跡 → 用 `_fly_to_target(home, timeout, tag="RETURN")` 跑**同一套 replan 迴圈**（FM 模型、MINCO、guard、geofence、電池、Kill 全部照舊）→ 到家後把 `_rth_sp` 設在 home、`_mission_state = RETURN` 懸停 `rth_hover_s` → 交給繼承的降落（在 home 降）
- 新參數 `rth_replan_timeout_s`（0 = 自動 = 直線距離 / `v_max` × 2 + `rth_timeout_extra_s` 30 s）。逾時 / 途中 abort → 就地降落；RC / 斷線 / Kill → `_hard_stop`（不再送 setpoint）
- 兩個 barometer launch 加 `rth_mode`（預設 `trail`）與 `rth_replan_timeout_s`（0.0）
- 限制：`rth_mode:=replan` 時返家路徑由 planner 決定，可能不是原路；去程關掉的 guard（`use_lookahead_guard:=false`、`blind_abort_s:=0`）返家時同樣是關的

**驗證（離線，不開 MAVROS、不解鎖）**：py_compile、pyflakes 乾淨。重構回歸：`test/offline_test_recovery.py` 35/35、`test_rthgoal.py`（scratchpad）46/46，與重構前相同。新測試 `test_rthreplan.py`（scratchpad）**17/17**：mixin 層 P1–P5（到目標 → 以 home 為 goal 呼叫 `_fly_to_target` 一次、tag RETURN、預算 66 s、`landing at home`、繼承降落跑一次；返家中 abort → 就地降；RC/Kill → hard stop 且不降落流程；`rth_mode:=trail` 完全走舊路徑不呼叫 `_fly_to_target`；STUCK 時 abort reason 先清掉）；真 node 層 P6–P10（`FMInferenceBaroNode` 讀到 `rth_mode=replan`、1 m 內 → `reached`、abort reason → `abort`、0.5 s 預算 → `abort`、RC override → `stopped` 且 `_stream_on=False`）。

**Build（12:2x，使用者同意）**：`colcon build --packages-select fm_deploy fm_deploy_barometer` 成功；install 內 `fm_inference_real_node.py`、`recovery.py`、兩個 launch 與 src md5 相同；用 install 版重跑 17/17 與 35/35；兩個已安裝 launch `--show-args` 有 `rth_mode`（default `trail`）。

**尚未做**：實飛（滿電、`rth_after_goal:=true rth_mode:=replan`）；知識庫同步。

**還原**（需使用者同意；或不還原，不加 `rth_mode:=replan` 即舊行為）：

```bash
cd ~/drone_ws/src && T=bak-RTHREPLAN-20260916-122251
for f in fm_deploy/fm_deploy/fm_inference_real_node.py fm_deploy_barometer/fm_deploy_barometer/recovery.py \
         fm_deploy_barometer/launch/fm_all_barometer.launch.py fm_deploy_barometer/launch/fm_real_barometer.launch.py; do cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy fm_deploy_barometer
```

### 2026-09-16 13:0x：返家時機頭轉向 home（修 RTHYAW）+ 可錄 octomap 供事後除錯（已 build）

使用者回報：`rth_mode:=replan` 返家時機頭方向沒變；另要求能存 octomap 事後除錯。

**機頭不轉的原因**：`_FlyingYawLockPublisher` 在 `_mission_state == FLYING` 時把 `msg.yaw` 換成 **`_home_yaw`**（鎖定 home 時的機頭方向 = 朝去程 goal）。返家用的 `_fly_to_target` 也把狀態設成 FLYING → 整段返家仍朝 goal，等於**倒退飛回家**，相機看的是背後。（`trail` 模式不受影響：它走 `STATE_RETURN`、yaw 由 `_rth_sp` 給。）

#### 程式修改（使用者同意）

- 檔案：`src/fm_deploy/fm_deploy/fm_inference_real_node.py`、`src/fm_deploy_barometer/fm_deploy_barometer/recovery.py`、`src/fm_deploy_barometer/launch/fm_all_barometer.launch.py`、`src/fm_deploy_barometer/command.txt`（`fm_inference_base.py`、`fm_real_barometer.launch.py` 沒動）
- 備份：同目錄 `*.bak-RTHYAWMAP-20260916-130908`；本檔 `CLAUDE.md.bak-RTHYAWMAP-20260916-130908`。搜尋 `RTHYAW`、`RTHYAWMAP`
- **RTHYAW**：新屬性 `_yaw_lock_ref`（每段航程的 yaw 基準）。`_fly_to_target(..., yaw_ref=None)`：`None` → `_home_yaw`（去程行為完全不變）；返家時 `recovery._return_home_replan` 傳入 `atan2(home − 目前位置)`。yaw lock 的 `home` 與 `smooth` 兩種模式都改用 `_ref(n)`（= `_yaw_lock_ref`，沒設就退回 `_home_yaw`）；基準一變就重置 smooth 的時間狀態，讓轉向從目前機頭平順開始。位置、速度、z 都沒動
- **RTHYAWMAP**（`fm_all_barometer` only）：`_FORWARDED` 加 `publish_3d_map`（預設 false，轉給 octomap_server → `/octomap_binary`、`/octomap_full`）；新 launch 參數 `record_map`（預設 false）與 `map_record_dir`（預設 `~/flight_maps`）。`record_map:=true` 會多跑一個 `ros2 bag record`（`IfCondition`）錄 `/projected_map`、`/octomap_binary`、`/octomap_full`、`/mavros/local_position/pose`、`/odom`、`/tf`、`/tf_static` 到 `<map_record_dir>/map_<時間>`。**只訂閱、不發佈，不碰 `/dev/ttyACM0`**；topic 還沒出現也會等（已實測）
- `command.txt` 加了錄製與 `octomap_saver_node` / `ros2 bag play` 的用法

**驗證（離線，不開 MAVROS、不解鎖）**：py_compile OK。新測試 `test_rthyaw.py`（scratchpad，真 `FMInferenceBaroNode`、publisher 鏈最內層換成 sink）5/5：去程仍送 home yaw 90°、返家送 −90°（朝 home）、`yaw_ref` 有傳就生效、沒傳維持 home yaw、smooth 模式也以返家方位為基準。回歸：`test_rthreplan.py`（重建版）9/9（含「`yaw_ref` = 由 goal 指回 home 180°」）、`offline_test_recovery.py` 35/35（`trail` 模式不變）。launch 可解析，`--show-args` 有 `record_map`、`map_record_dir`、`publish_3d_map`。

**Build（13:1x，使用者同意）**：`colcon build --packages-select fm_deploy fm_deploy_barometer` 成功；install 內 real node、`recovery.py`、`fm_all_barometer.launch.py` 與 src md5 相同；用 install 版重跑 5/5、9/9、35/35。

**尚未做**：實飛（滿電；返家時看機頭是否轉向 home）；用錄下的 bag 實際除錯一次；知識庫同步。

**還原**（需使用者同意）：

```bash
cd ~/drone_ws/src && T=bak-RTHYAWMAP-20260916-130908
for f in fm_deploy/fm_deploy/fm_inference_real_node.py fm_deploy_barometer/fm_deploy_barometer/recovery.py \
         fm_deploy_barometer/launch/fm_all_barometer.launch.py fm_deploy_barometer/command.txt; do cp $f.$T $f; done
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy fm_deploy_barometer
```

### 2026-09-21 20:3x：`fm_deploy_barometer` 新增 `octomap_view_node`（SSH 終端機看環境地圖）；已 build

使用者要求：在 SSH 時能看到環境的視覺化。選擇（使用者決定）：**終端機 ASCII 地圖**，並**掛進 `fm_all_barometer.launch.py`**（`view_map:=true`，預設 false）。

畫的是 `/projected_map` — octomap_server 投影出來、`esdf_ros2.py` 真正拿來規劃的那張 2D OccupancyGrid（`odom` frame），不是 3D octree。所以看到的就是規劃器看到的（OFF-MAP 誤判、地圖空白都能直接看出來）。

**這個 node 只訂閱，不發佈任何 topic、不呼叫任何 service、不開 `/dev/ttyACM0`**（離線測試有驗）。訂閱：`/projected_map`、`/px4/sensors`（local_x/y/z、yaw、battery_pct）、`/px4/state`（armed/mode/connected）。

#### 程式修改（使用者同意）

- 新檔：`src/fm_deploy_barometer/fm_deploy_barometer/octomap_view_node.py`、`test/offline_test_mapview.py`
- 改檔：`src/fm_deploy_barometer/setup.py`（註冊 executable）、`launch/fm_all_barometer.launch.py`、`command.txt`
- 備份：同目錄 `*.bak-MAPVIEW-20260921-203348`；本檔 `CLAUDE.md.bak-MAPVIEW-20260921-203348`。搜尋 `MAPVIEW`
- 沒動：`fm_inference_*`、`recovery.py`、`fm_real_barometer.launch.py`、`fm_deploy`（GPS 版）、PX4 參數

| 項目 | 內容 |
| --- | --- |
| 輸出模式 `view_mode` | `scroll`（預設，每次印一塊新的，不用游標控制 → 與 launch 內其他 node 的 log 共用 stdout 不會互相破壞）／ `fullscreen`（清畫面原地重畫，給第二個 SSH 終端機單獨跑） |
| 座標 | ENU，上 = 北（+y）、右 = 東（+x），與 ESDF、RViz marker 同一套 |
| 符號 | `#` 佔據、`.` 已觀測空白、空格 unknown、箭頭 = 無人機（8 方位機頭）、`H` home、`G` goal、`o` 走過的麵包屑 |
| home / goal | home 在 **未解鎖→解鎖那一刻**鎖定（`fm_inference_real_node._lock_home()` 同一時機）；goal = home + `goal_dist` 沿機頭 + `goal_lat` 向左，launch 直接把任務的 `goal_dist` / `goal_lat` 轉給它，所以 `G` 與實際目標一致 |
| 比例 | 字元格約 2:1，所以每列公尺數 = 每欄的 2 倍，圖不會被壓扁。解析度受 `view_rows` 限制 → 想更細就加 `view_rows:=30 view_cols:=120` |

- launch 參數（四個 `--show-args` 可見）：`view_map`(false)、`view_hz`(0.5)、`view_rows`(18)、`view_cols`(90)、`view_range`(0.0 = 整張圖；>0 = 跟著無人機的 ±N m 視窗)、`view_mode`(scroll)。node 自己還有 `view_color`(auto/true/false)、`view_ascii`、`trail_step`(0.3)、`trail_max`(400)、三個 topic 名稱
- 單獨跑：`ros2 run fm_deploy_barometer octomap_view_node --ros-args -p view_mode:=fullscreen -p view_hz:=2.0 -p goal_dist:=5.0`（`view_rows:=0 view_cols:=0` = 跟著終端機大小）
- 用法寫在 `src/fm_deploy_barometer/command.txt` 最後一節

**開發中修掉的三個 bug**（都是測試抓到的）：

1. `-p view_color:=false` 在命令列是 **bool**、`-p view_hz:=2` 是 **int**，固定型別的 `declare_parameter` 會讓 node 一啟動就 crash（實機 smoke test 抓到）→ 所有 `view_*` / `goal_*` / `trail_*` 改用 `ParameterDescriptor(dynamic_typing=True)` 再自己轉型
2. 上框線用 `bar[:-1]` 接指北標籤，開顏色時會砍掉 ANSI reset 的最後一個字元 → 輸出亂碼。改成先畫完整框線再接標籤
3. `trail_max` 調小時每次只刪一個元素，list 永遠縮不回上限 → `if` 改 `while`

**驗證（離線，不開 MAVROS、不解鎖、不碰飛控）**：py_compile、pyflakes 乾淨。`test/offline_test_mapview.py` **50/50**（src 與 install 版都跑）：無地圖時的等待訊息、佔據/空白/unknown 三種格子、home/goal/箭頭/麵包屑、**方位正確性**（北邊的牆印在無人機上方、南邊的印在下方、東邊的柱子印在右邊）、8 個機頭方位、`trail_step` 間距與 `trail_max` 上限、`view_range` 視窗、無人機在視野外的警告、pose STALE 警告、scroll/fullscreen、開關顏色後**去掉 ANSI 必須與無色版逐字相同**、ASCII fallback、空 grid 與壞 JSON 不 crash、400×400 大圖仍塞進終端機、**node 除了 rosout 沒有任何 publisher、沒有 arming/param service client**、5 種命令列參數型別。回歸：`offline_test_recovery.py` 35/35 不變。

**實機 smoke test（無 MAVROS、無電池、沒有任何東西解鎖）**：install 版 `ros2 run` 正常啟動並每 0.5 s 印等待訊息；另用假發佈器（scratchpad `fake_map_pub.py`，純 ROS topic）餵 100×80 合成地圖 + 移動中的 pose，地圖、箭頭、麵包屑、`H`、`G` 都正確畫出來。launch 整合用**不執行**的方式檢查（`check_launch.py`，避免開 MAVROS/相機）10/10：node 有掛上、走 TimerAction、`view_map` 條件開關正確、`goal_dist` / `goal_lat` 有轉過去、數值參數型別正確、其他 node 沒被動到。

**Build（20:4x，使用者同意）**：`colcon build --packages-select fm_deploy_barometer` 成功（排除 conda PATH，shebang `/usr/bin/python3`）；install 內 5 個 .py 與 2 個 launch 與 src 逐檔相同；executable `octomap_view_node` 已註冊；已安裝 launch `--show-args` 有 6 個新參數。

**尚未做**：實飛時實際使用（戶外 Gemini 2 深度常常只有 0–4% 有效像素，地圖可能大半是空白 — 這正好是這個工具要讓你看見的）；知識庫同步記錄。

**還原**（需使用者同意；或不還原，不加 `view_map:=true` 即完全沒有影響）：

```bash
cd ~/drone_ws/src/fm_deploy_barometer && T=bak-MAPVIEW-20260921-203348
cp setup.py.$T setup.py
cp launch/fm_all_barometer.launch.py.$T launch/fm_all_barometer.launch.py
cp command.txt.$T command.txt
rm -f fm_deploy_barometer/octomap_view_node.py test/offline_test_mapview.py
cd ~/drone_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fm_deploy_barometer
```
