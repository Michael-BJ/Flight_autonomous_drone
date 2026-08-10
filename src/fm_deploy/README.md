# `fm_deploy` — FM-Planner inference on the real drone

This package is the hardware version of the inference pipeline that already
runs in simulation (`RL_FM/src/fm_planner`). The planner is **identical** —
`fm_model.py`, `fm_inference_base.py`, `fm_inference_node.py`,
`min_jerk_planner.py`, and `esdf_ros2.py` are copied over as-is, without a
single line changed. If the planning logic were modified here, real-flight
results could no longer be compared against simulation results, and that
weakens the "same model" claim in the paper.

Only three files are **new**:

| File | Role |
|---|---|
| `gemini2_depth_bridge_node.py` | Orbbec Gemini 2 → model input contract (replaces `gz_depth_bridge_node.py`) |
| `fm_inference_real_node.py` | `FMInferenceNode` + the hardware safety layer from `takeoff_land_node.py` |
| `launch/fm_real.launch.py` | Full stack with no Gazebo dependency |

`px4_sensor_reader.py` and `mavros_only.launch.py` are taken from
`takeoff_land` (the serial/Jetson version already proven in flight), not
from the sim workspace.

---

## 1. What changes going from simulation to the real world

### 1.1 Camera — the most fragile part

In Gazebo, a "no obstacle" pixel is `+inf`, and `_form_model_input` maps it
to 10 m (**far/safe**). Gemini 2 returns **0** for "no return" — and 0 under
the same convention means **"obstacle touching the lens"**. In the field, a
0 reading is actually often something safe: a shiny surface, a window, an
object beyond 10 m, a stereo dropout hole.

If raw depth were passed straight through, the model would see a solid wall
right in front of its nose for the whole flight. That's why the bridge:

1. converts 16UC1 mm → 32FC1 meters,
2. resizes to 640×480 with `INTER_NEAREST` (linear interpolation would
   invent depth "between" an obstacle's edge and the background → ghost
   obstacles),
3. patches **small** holes with the neighborhood median,
4. fills the remaining invalid pixels with `invalid_fill_m` (**default
   10.0 m = far**),
5. clips to `[0, 10]` m per `DEPTH_NORM_MAX_M`,
6. scales the `camera_info` intrinsics to match the resize (otherwise the
   point cloud is metrically wrong and obstacles in the octomap end up
   wider/narrower than they are).

Verify before flying — point the drone at a wall ±1.5 m away:

```bash
ros2 topic hz   /realsense/depth/float32     # a stable ≥10 Hz
ros2 topic echo /realsense/depth/stats       # p50_m ≈ 1.5, high valid_pct
```

If `valid_pct` < 20%, the node will warn. Don't fly with a "blind" camera:
the output still looks safe (everything reads as 10 m).

### 1.2 Goal frame

In simulation the drone always spawns at (0, 0) facing +X, so
`goal_x:=20` is immediately correct. In the field the EKF origin is at
whatever position & heading it happens to have at boot. This node locks
**home** (x, y, yaw) right before ARM and then computes:

```
goal = home + R(yaw_home) · [goal_dist, goal_lat]
```

The geofence is also defined in the home frame, then wrapped into an
axis-aligned box for the virtual ESDF wall (the ESDF only understands
axis-aligned boxes; the AABB is always more permissive, so the hard
safeguard is `max_home_dist` in the watchdog).

Consequence: **point the drone's nose in the direction you want before
running the mission.**

### 1.3 Safety layers that don't exist in Gazebo

| Mechanism | Action |
|---|---|
| RC override (mode leaves OFFBOARD) | setpoints stop **instantly**, pilot has full control |
| MAVROS link drops mid-mission | setpoints stop, PX4 failsafe takes over |
| Geofence `max_home_dist` | abort → controlled descent → AUTO.LAND |
| Altitude deviation > `max_alt_error` | abort → land |
| Battery below threshold | abort → land |
| Unexpected disarm | abort |
| `mission_timeout_s` | land |
| TAKEOFF/LANDING | holds **XY home**, not the instantaneous position (sim uses the instantaneous position, meaning EKF drift becomes part of the command) |

`COM_RC_OVERRIDE ∈ {2, 3}` is verified before ARM. Without it the RC stick
can't physically take over OFFBOARD, and every detection above is useless.

### 1.4 Planner guards: defaults deliberately DIFFER from simulation

In sim, `use_safety_guards` defaults to `false` because the goal is to
measure the model's raw behavior ("fair" mode for ablation). In the field,
"letting the model fail" means hitting a real wall — so in
`fm_real.launch.py` the default is `true` (guard + escape active), `v_max`
is 0.5 (not 1.0), and `cmd_hz` is 50 (not 100, to avoid flooding the serial
link).

For real-world ablation measurements later, turn these off again
explicitly — but only do that in a large open space with a net/pilot on
standby.

### 1.5 PX4 parameters are not touched

The sim node writes `EKF2_HGT_REF=1` (height reference = GPS). For
**indoor** flight with VIO/optical flow, that actually breaks the
estimate. The default here is `write_px4_params:=false`. Set
`MPC_XY_VEL_MAX` and similar via QGroundControl to match your vehicle.

---

## 2. Installation

```bash
cd ~/drone_ws/src
cp -r fm_deploy .

# FM checkpoint (do NOT commit this, it's large)
mkdir -p fm_deploy/model/fm
cp ~/saved_net/fm/run_20260724_190037/fm_planner_20260724_190037.onnx      fm_deploy/model/fm/
cp ~/saved_net/fm/run_20260724_190037/fm_planner_20260724_190037.onnx.data fm_deploy/model/fm/

cd ~/drone_ws
colcon build --packages-select fm_deploy
source install/setup.bash
```

Python dependencies on the Jetson: `torch` (if using `.pth`),
`onnxruntime-gpu` (if using `.onnx`), `scipy`, `opencv-python`,
`pyquaternion`, `cv_bridge`.

**ONNX backend note:** the existing `.onnx` file locks the `noise` input to
a static `[8, 9]` shape. That means with the ONNX backend, `K` **must be
8**, and `use_anchor_sampling` (which requests K−1 fresh samples) can't be
used. If you need a different K, re-export with dynamic dimensions or use
`.pth`. Its opset is 20, so it needs onnxruntime ≥ 1.17.

---

## 3. Running it

```bash
# T1 — communication
ros2 launch fm_deploy mavros_only.launch.py fcu_url:=/dev/ttyTHS1:921600

# T2 — camera driver (Orbbec package, outside this repo)
ros2 launch orbbec_camera gemini2.launch.py

# T3 — perception + planner
ros2 launch fm_deploy fm_real.launch.py \
    model_path:=$HOME/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037.onnx \
    dry_run:=true

# T4 — optional
rviz2
```

### Testing ladder — don't skip steps

| Stage | Command | Propellers | Passes when |
|---|---|---|---|
| 1 | `dry_run:=true` | **REMOVED** | `[INF] Replan ok` shows up; `/planner/candidates` looks reasonable; `[FM] GATE two-sided` is populated |
| 2 | `dry_run:=false goal_dist:=0.0` | on | clean takeoff → hover → land in an empty room |
| 3 | `dry_run:=false goal_dist:=3.0` | on | straight 3 m flight with no obstacle |
| 4 | `goal_dist:=5.0` + 1 obstacle | on | dodges correctly |

Stage 1 needs no motor battery and will never arm — this is where you find
TF issues, depth-unit issues, and camera-mount issues.

---

## 4. What you MUST measure yourself

**Camera transform** (`cam_x`, `cam_y`, `cam_z`) from the drone's center of
mass to the depth lens, in meters, FLU convention (forward+, left+, up+).
The default `0.10 / 0.00 / -0.05` is copied from the simulation SDF and
almost certainly does **not** match your vehicle. Being off by 5 cm shifts
the entire octomap by 5 cm; a wrong sign shifts obstacles to the wrong side.

Quick check: hover in front of a wall, open RViz, display `/projected_map`
and TF. The wall on the map must sit exactly at the distance you measure
with a tape measure.

`cam_roll/pitch/yaw` (−1.5708 / 0 / −1.5708) maps `base_link` FLU to the
camera's **optical** frame (z forward, x right, y down) — only change this
if the camera is mounted at an angle.

---

## 5. Quick diagnostics

| Symptom | Check |
|---|---|
| `Timeout depth/ESDF` | `ros2 topic hz /realsense/depth/points`, `ros2 run tf2_ros tf2_echo odom camera_depth_frame`, whether `/projected_map` is publishing |
| `Message Filter dropping message` in octomap | TF `odom→base_link` isn't flowing → MAVROS `local_position/pose` is empty (needs GPS/VIO) |
| Every replan fails | `ros2 topic echo /realsense/depth/stats` — low `valid_pct`, or `occ_min_z/occ_max_z` doesn't bracket `target_alt` |
| Drone dodges to the wrong side | camera transform (section 4) |
| `[FM] GATE two-sided 0%` | normal for a narrow corridor; compare against the simulation numbers for a similar scene |
| Setpoints not being sent | `dry_run` is still `true`, or `_rc_override`/`_link_lost` is already set |

Log lines worth recording for the paper:
`[REAL] DONE` (replan count, cost-gate vetoes, post-check vetoes) and
`[REAL] Bimodality gate` at the end of every mission.
