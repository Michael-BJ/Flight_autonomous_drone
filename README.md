# drone_ws — Autonomous flight: take-off → flight → landing

Summary of how the drone works, from power-up to disarm: which node does what,
which data goes where, and which safety checks can stop the mission.
Everything below was read from the code in `src/` (default values are those of
`fm_deploy_barometer/launch/fm_all_barometer.launch.py`, the launch file used
for real flights).

---

## 1. Big picture

```
 Orbbec Gemini 2 (depth)                         RC transmitter (pilot)
        │ USB                                          │ radio
        ▼                                              ▼
┌────────────────────────── Jetson Orin Nano (ROS 2 Humble) ─────────────────────────┐
│                                                                                    │
│  orbbec_camera ─► gemini2_depth_bridge ─► depth_to_pointcloud ─► octomap_server    │
│  /camera/depth/image_raw   /realsense/depth/float32   /realsense/depth/points      │
│                                  │                                   │ /projected_map
│                                  ▼                                   ▼             │
│                        ┌──────────────────── fm_inference_baro_node ──────────┐    │
│                        │  FM model (TensorRT) ─► MINCO refinement ─► ESDF checks│   │
│                        │  mission FSM · safety layer · altitude hold · RTH     │   │
│                        └───────────────▲──────────────────────┬────────────────┘   │
│                         /px4/state     │                      │ /mavros/setpoint_raw/local (50 Hz)
│                         /px4/sensors   │                      │ /mavros/cmd/arming, /mavros/set_mode
│                        ┌───────────────┴──────┐               ▼                    │
│                        │  px4_sensor_reader   │◄────────── MAVROS ◄──── TF odom→base_link
│                        └──────────────────────┘     (mavros_tf_broadcaster)        │
└──────────────────────────────────────────────────────────┬─────────────────────────┘
                                          serial /dev/ttyACM0:57600 (USB) or /dev/ttyTHS1:921600 (UART)
                                                           ▼
                                             PX4 flight controller (EKF2, position control)
                                 sensors: IMU, magnetometer, barometer, GPS, Holybro H-Flow (range + flow)
```

Principle: **the Jetson never drives the motors directly.** It only streams
*position setpoints* to PX4 in OFFBOARD mode; PX4 does the attitude/rate control.
The pilot's RC always has priority (see §7).

## 2. Packages

| Package | Role |
|---|---|
| `fm_deploy` | Real-drone version of the FM planner: camera bridge, point cloud, TF, `px4_sensor_reader`, `fm_inference_real_node` (mission FSM + hardware safety layer), TensorRT/ONNX model in `model/fm/`. Planner files are identical to simulation. |
| `fm_deploy_barometer` | **What is flown.** `fm_inference_baro_node` = barometer/H-Flow altitude hold + return-home recovery on top of `fm_inference_real_node`; launch files `fm_all_barometer` / `fm_real_barometer`; `octomap_view_node` (ASCII map viewer). |
| `takeoff_land_barometer` | Altitude estimators (`baro_altitude.py`, `hflow_altitude.py`), shared launch helpers, and a plain take-off → hover → land node with barometric/H-Flow altitude. |
| `takeoff_land` | Original plain take-off → hover → land node (`takeoff_land_node`), `px4_sensor_reader`, hold-position and mode-monitor nodes. The pre-flight gates and the landing code are inherited by the packages above. |

## 3. Start-up (one launch file)

```bash
ros2 launch fm_deploy_barometer fm_all_barometer.launch.py goal_dist:=6.0 target_alt:=1.0 ... dry_run:=false
```
(full command and parameter reference: `src/fm_deploy_barometer/command.txt`; run it in `tmux`.)

| Time | Started |
|---|---|
| t = 0 s | Kill of stale nodes/shared memory, **MAVROS** (`fcu_url`, plus `hflow_mavros.yaml` which enables `/mavros/distance_sensor/hflow`) |
| +5 s | **Orbbec Gemini 2** driver (depth only: colour, IR, IMU, point cloud, TF disabled) |
| +9 s | `fm_real_barometer.launch.py`: `mavros_tf_broadcaster`, static TF base_link→camera_depth_frame, `gemini2_depth_bridge`, `depth_to_pointcloud`, `octomap_server_unknown`, `fm_inference_baro_node` |
| +10 s | `px4_sensor_reader` (from `mavros_only.launch.py`) |
| optional | `record_map:=true` → rosbag in `~/flight_maps/`; `view_map:=true` → ASCII octomap in the terminal |

`dry_run:=true` runs the whole perception + planner stack **without arming and without sending setpoints** (safe on the ground).

## 4. Data that is sent and received

### 4.1 PX4 → Jetson (telemetry)

`px4_sensor_reader` is the **only** MAVROS subscriber for telemetry. It merges everything into two JSON strings (`std_msgs/String`) that the flight nodes read:

| Topic | Content |
|---|---|
| `/px4/state` | `connected`, `armed`, `mode`, `guided`, `manual` |
| `/px4/sensors` | `local_x/y/z` and `yaw` (EKF pose, used for flight), `vel_x/y/z`, `imu` (roll/pitch/yaw), `gps` (lat/lon/alt/fix_status), `gps_quality` (`fix_type`, `satellites`, `hdop`, `vdop`, `h_acc_m`), `altitude` (amsl/relative/terrain), `battery` (voltage, %) |

Raw MAVROS sources: `/mavros/local_position/pose`, `…/velocity_local`, `/mavros/imu/data`, `/mavros/imu/mag`, `/mavros/imu/static_pressure`, `/mavros/imu/temperature_imu`, `/mavros/global_position/global`, `/mavros/gpsstatus/gps1/raw`, `/mavros/battery`, `/mavros/state`, `/mavros/altitude`, `/mavros/home_position/home`, `/mavros/statustext/recv`.

Read **directly** by the flight node (not through the JSON):

| Topic | Use |
|---|---|
| `/mavros/rc/in` | Pilot switches: mode switch (ch 5, must sit at the OFFBOARD slot ≈1499 µs ± 150) and kill switch (ch 11, engaged > 1500 µs) |
| `/mavros/altitude` (`.monotonic`) | Pure barometric altitude, ~10 Hz, for `baro_altitude.py` |
| `/mavros/distance_sensor/hflow` + `/mavros/imu/data` | H-Flow range and tilt → height above ground (`hflow_altitude.py`) |
| PX4 parameter `COM_RC_OVERRIDE` | Read through MAVROS param services before flying |

### 4.2 Camera → map → planner

| Stage | Topic | Data |
|---|---|---|
| Gemini 2 driver | `/camera/depth/image_raw` | 16UC1, millimetres |
| `gemini2_depth_bridge` | `/realsense/depth/float32`, `…/camera_info` | 32FC1 metres, 640×480 (nearest-neighbour resize, small holes median-filled, invalid pixels → 10 m "far"), clipped to ≤ 10 m |
| `depth_to_pointcloud` | `/realsense/depth/points` | PointCloud2, stamped with the node's clock (matches MAVROS TF); optional altitude gate and voxel confirmation (`confirm_hits`, default 1 = off) |
| `octomap_server_unknown` | `/projected_map` (2D occupancy), optional 3D map | Resolution `resolution`, height band `occupancy_min_z/max_z` set at runtime to follow the cruise altitude (widened upward by `band_drift_margin`) |
| `fm_inference_baro_node` | ESDF built from `/projected_map` | Distance-to-obstacle field used for ranking, MINCO cost and all guards |

TF tree: `odom → base_link` (from `/mavros/local_position/pose`) and `base_link → camera_depth_frame` (static: x 0.10, y 0.00, z −0.05 m, yaw −1.5708 rad by default).

### 4.3 Jetson → PX4 (commands)

| Command | Interface | Content |
|---|---|---|
| Position setpoint | `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`), **50 Hz** (`cmd_hz`) | `FRAME_LOCAL_NED`; position x/y/z (ENU values, MAVROS converts) + yaw, plus velocity feed-forward in the FLYING phase. One single publisher gate closes immediately on RC override, link loss, kill switch or unexpected disarm. |
| ARM / DISARM | service `/mavros/cmd/arming` | |
| Mode | service `/mavros/set_mode` | `OFFBOARD`, `AUTO.LAND` |
| PX4 parameters | `/mavros/param/set` | **Not used** by default (`write_px4_params:=false`) |

### 4.4 Altitude — which number is trusted

PX4 always receives an ordinary EKF-frame setpoint, but z is re-computed every tick:

`z_setpoint = z_ekf + gain · (alt_target − alt_measured)`

| Source | Parameter | Status |
|---|---|---|
| EKF z | `baro_gain:=0`, `hflow_mode:=shadow` (defaults) | Used for flight. H-Flow altitude is only measured, logged to `hflow_log_dir` and compared with the EKF (warning above 0.25 m difference). |
| Barometer | `baro_gain>0` | Low-passed (τ = 1 s), **not used below 1 m** (propeller ground effect); abort if no sample for 5 s. |
| H-Flow | `hflow_mode:=hold` (needs `baro_gain 0`) | Replaces the above for take-off, hover, watchdog and setpoint; abort to AUTO.LAND if no valid sample for 3 s. |

Reference: PX4 uses the H-Flow range as height reference since 2026-10-19, GPS still fused for x/y; optical-flow fusion is off.

## 5. Mission flow, phase by phase

Phase names appear on the terminal as `[T+mm:ss] PHASE | …` status lines (every 2 s, coloured).

```
PREFLIGHT ─► READY/ARMING ─► TAKEOFF ─► HOVER ─► FLYING (→ goal) ─► [RETURN] ─► LANDING ─► DONE
                                                       any phase: ABORT paths (§7)
```

### 5.1 PREFLIGHT — nothing spins yet
In order; failing any gate ends the program without arming:

1. **FCU connection** via `/px4/state` (timeout 60 s).
2. **`COM_RC_OVERRIDE`** read from PX4 — must allow RC sticks to leave OFFBOARD (`verify_rc_override_param`).
3. **Local pose** available (30 s).
4. **GPS gate** (`require_gps`): fix ≥ 3D, ≥ 8 satellites, HDOP ≤ 2.0, held 5 s continuously, timeout 120 s. Born from the 2026-08-27 crash (take-off with 0 satellites).
5. **Perception ready**: depth frame + ESDF from the octomap (timeout 90 s).
6. **Ground reference** (`ground_z`) — EKF (and barometer/H-Flow when active) must be quiet over `ekf_window_s` = 5 s: peak-to-peak ≤ `max_ground_drift` (0.5 m), std ≤ `ekf_gate_std` (0.15 m); barometer std ≤ 0.08 m, drift ≤ 0.30 m, ≥ 5 Hz. A wandering estimate is refused rather than forced to 0.
7. **Home lock**: home x/y/yaw = current pose. The **goal** is `goal_dist` metres straight ahead of the nose at that moment (+ `goal_lat` to the left). The **geofence box** (`fence_fwd` 10 / `fence_back` 3 / `fence_lat` ±4 m) and the abort radius `max_home_dist` (12 m) are defined in this home frame.
8. **Octomap band** set around cruise altitude, **model warm-up** (TensorRT engine; ONNX fallback on out-of-memory).

### 5.2 READY → ARMING
1. Setpoint stream starts, holding **home x/y at ground z** (PX4 refuses OFFBOARD without a stream); ~2 s warm-up.
2. Pilot checks: **RC mode switch already in the OFFBOARD slot** (so every other switch position stays an escape route) and **kill switch released**. From here the kill switch is watched; engaging it is terminal.
3. **ARM** (retry every 1.5 s up to `arm_timeout` 30 s) → **OFFBOARD** (retry up to 5 s). Failure → disarm and exit.
4. From this moment *RC override detection* is active: any mode change away from OFFBOARD is read as the pilot taking over.

### 5.3 TAKEOFF
Setpoint = home x/y, z = `ground_z + target_alt`, yaw = home yaw. Done when altitude is within ±0.15 m and |vz| < 0.2 m/s for 2 s (30 s timeout; on timeout the mission continues with a warning).

### 5.4 HOVER (settle)
`hover_settle_s` = 5 s over home. Then the octomap is **wiped** so planning starts from a clean map — unless `keep_octomap:=true` (the map built since take-off is kept; used in the reference flight).

### 5.5 FLYING — the planning loop
`replan_period` = 1.0 s (earlier if a guard invalidates the trajectory, min. 0.3 s gap), 50 Hz command stream in between.

1. **Input**: depth image → 8-bit (0–10 m) + 24-value motion vector (velocity, attitude, head/target state in body frame).
2. **FM model** (flow matching, 2 Euler steps) samples `K` = 8 candidate paths (2 waypoints + 3 segment times), run once through the encoder.
3. **Ranking** by ESDF clearance; best `max_candidates` kept.
4. **MINCO refinement** (`min_jerk_planner`) with `safe_dis` 0.8 m, feasibility weight 1000, wall-clock budget `replan_budget_s` 1.0 s. Retries once with a shorter local target.
5. **Validation**: quality gate (`max_plan_cost`), hard clearance post-check (0.55 m from centre), time scaling so peak speed ≤ `v_max` (0.5 m/s).
6. **Install**: the setpoint is sampled from the trajectory `planning_time_ahead` = 0.3 s ahead; yaw follows the motion direction with smoothing (`flying_yaw_mode`, rate-limited to 30°/s, deadband 15°, ±60° from home yaw).
7. Optional guards (look-ahead, speed limit, blind/stuck abort, escape mode) are switched per launch argument; the `command.txt` flight turns most of them off — **open fields only**.
8. **Goal reached** when within 1 m (or the planner reports it) → LANDING, or RETURN if `rth_after_goal:=true`.
9. Stops: mission timeout `mission_timeout_s`, or any abort reason from §7.

### 5.6 RETURN (recovery / optional)
Only entered for a *recoverable planner* stop (drone off-map, STUCK, mission timeout) or after the goal when `rth_after_goal:=true`. Skipped if already within `rth_min_dist` (1 m) of home, if disabled, or if the trail is empty.
- `rth_mode:=trail` — retrace the recorded breadcrumbs (one every 0.3 m) backwards at `rth_speed` 0.3 m/s, nose in the direction of motion.
- `rth_mode:=replan` — run the FM planner again with home as goal.
- Guards: known obstacle within `rth_min_clearance` for `rth_block_s` (8 s), tracking error > 1.5 m for 3 s, time budget, geofence, battery → land in place. Hover `rth_hover_s` = 2 s over home, then LANDING.

### 5.7 LANDING
1. **Controlled descent**: z setpoint lowered at `descent_speed` = 0.3 m/s down to `ground_z + land_handoff_alt` (0.25 m), holding the current x/y (home for the plain takeoff_land node, current position after a planner stop).
2. **Hand-off**: setpoint stream stopped, mode → **AUTO.LAND**; PX4's land detector disarms.
3. Wait up to 20 s for auto-disarm; if still armed → forced DISARM.
4. **DONE**: summary logged (successful replans, cost-gate and post-check vetoes, safety-distance report, bimodality gate statistics); node shuts down.

## 6. Plain take-off / land variant (no planner)

`takeoff_land` / `takeoff_land_barometer` run the same PREFLIGHT → ARM → OFFBOARD → TAKEOFF sequence, then **HOVER** at home for `hover_time` (8 s) and LANDING exactly as above. Setpoint is always home x/y at the take-off altitude (`target_alt` 1.2 m). Its pre-flight gates are the code reused by the FM node.

## 7. Safety and abort matrix

| Trigger | Detection | Reaction |
|---|---|---|
| **Pilot takes over** (mode leaves OFFBOARD) | `rc_override_enabled` | Setpoints stop at once; pilot has full control. Needs `COM_RC_OVERRIDE` ≥ 2 on PX4 (verified before arming). |
| **Kill switch** (ch 11) | `/mavros/rc/in` | Latched: setpoints stop, AUTO.LAND requested, normal disarm retried, never re-arms. |
| **MAVROS/FCU link lost** | `/px4/state` | Setpoints stop (mission aborted). |
| **Unexpected disarm** | `armed` flag | Setpoints stop, no mode command, exit. |
| **Geofence** — farther than 12 m from home | watchdog, 5 Hz | Land in place. |
| **Altitude deviation** > `max_alt_error` (0.5 m in the example command) from cruise | watchdog (EKF, barometer or H-Flow altitude) | Land in place. |
| **Battery** below `min_battery_pct` / `min_battery_v` | `/px4/sensors` | Land in place. |
| **Barometer / H-Flow stream lost** (5 s / 3 s) | estimators | Land in place. |
| **Planner stuck / off-map / timeout** | planner guards | RETURN (if enabled) then land; otherwise land in place. |
| **Planner unable to find a plan** | replan failures | Escape mode (if guards on) or hover until the next replan. |

"Land in place" = controlled descent + AUTO.LAND. Terminal abort paths (pilot, kill, link, disarm) deliberately send *nothing* further.

## 8. Logs and diagnostics

- Terminal: coloured `[T+mm:ss] PHASE | alt, ekf_alt, drift, sp_z, ARMED/mode, gps sats/hdop, baro Hz` lines every 2 s.
- ROS logs: `~/.ros/log/<timestamp>/` (flight node, sensor reader).
- Map rosbag (`record_map:=true`): `~/flight_maps/map_<timestamp>/` with `/projected_map`, `/octomap_*`, pose, TF.
- RViz: `/planner/candidates` (model candidate fan + flown path).
- H-Flow CSV at 10 Hz in `hflow_log_dir` when `hflow_mode` ≠ `off`.

## 9. Operating notes

- Run flights in **tmux**: a stalled SSH/VS Code terminal froze the pose stream mid-flight on 2026-10-02 (there is no pose-stale watchdog yet).
- `ROS_DOMAIN_ID=0`, `FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_jetson.xml` must exist before launching (see `command.txt`).
- Indoors without GPS: `require_gps:=false` — then you are responsible for confirming the other position source (VIO/flow) is healthy.
- Only one flight node may run at a time; the launch helpers kill any stale `*_node` of this workspace on start.
