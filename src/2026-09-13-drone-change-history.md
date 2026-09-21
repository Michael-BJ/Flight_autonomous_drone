---
title: Drone Change History (up to 2026-09-13)
date: 2026-09-13
tags:
  - discussion
  - uav/safety
  - uav/flight-control
  - uav/state-estimation
  - stack/px4
  - stack/ros2
---

# Drone Change History (up to 2026-09-13)

This document lists every recorded change made to our drone:

- flight-controller (PX4) parameters and calibration
- the RC transmitter setup
- the onboard code in `~/drone_ws` on the Jetson
- the Jetson itself and the airframe hardware

It also lists what was tried and then reverted, and what still has to be tested.

- **Snapshot:** 2026-09-13, 17:13 (Jetson clock) for the onboard code up to 15:17, extended with the later entries in the timeline. PX4 parameters updated at 23:45 (section 3.6). The three `*_barometer` packages added at 22:47–23:14 are documented in `~/drone_ws/CLAUDE.md`, not here.
- **Time zone:** all times are local time (UTC+8).
- **Evidence:** flight logs, git history, the change log on the Jetson, and the records of the sessions in which the changes were made. See section 10 for sources and gaps.

## The drone

| Item | Value |
| --- | --- |
| Flight controller | Pixhawk 6X (FMUv6X, STM32H753). **Not** 6X-RT — the firmware is not interchangeable |
| Firmware | PX4 v1.17.0 release (`ver_sw_release = 0x011100FF`). Same in every log from 2026-08-27 to 2026-09-09 |
| Companion computer | NVIDIA Jetson Orin Nano Developer Kit Super, JetPack 6 (L4T R36.4.7), Ubuntu 22.04, ROS 2 Humble |
| Jetson ↔ Pixhawk | USB `/dev/ttyACM0:57600`, through MAVROS (not uXRCE-DDS) |
| RC transmitter | RadioLink AT10II, Mode 2 |
| Depth camera | Orbbec Gemini 2 (USB to the Jetson) |
| Propulsion | 6S battery, KV390 motors, 13-inch propellers, ES-40A (40 A) ESCs |
| Onboard packages | `takeoff_land`, `fm_deploy`, `gps_guard`, `forward_move` |

---

## 1. Current state at a glance

| Area | Current state | Last changed |
| --- | --- | --- |
| Mode switch (CH5, SwG) | PWM 1065 = **Land**, 1499 = **Offboard**, 1933 = **Position** | 2026-09-09 |
| Arm switch | CH9, SwD (`RC_MAP_ARM_SW = 9`) | unchanged |
| Kill switch | CH11, SwF (`RC_MAP_KILL_SW = 11`). Was CH10 | 2026-09-09 |
| Dedicated Offboard switch | Removed (`RC_MAP_OFFB_SW = 0`). Was CH11 | 2026-09-09 |
| Offboard setpoints lost for 1 s | PX4 lands (`COM_OBL_RC_ACT = 4`). Was Position mode | 2026-09-09 |
| Critical battery | PX4 lands (`COM_LOW_BAT_ACT = 2`). Was warning only | 2026-09-09 |
| RC signal lost | Return (`NAV_RCL_ACT = 2`, PX4 default) | unchanged |
| Vertical speed limits | 0.5 m/s up, 0.5 m/s down. Were 3.0 / 1.5 | 2026-09-09 |
| Horizontal speed limit | 1.5 m/s (`MPC_XY_VEL_ALL`) | unchanged (1.0 tried for one minute) |
| Hover thrust / takeoff ramp | `MPC_THR_HOVER = 0.65`, `MPC_TKO_RAMP_T = 1.5` s. Were 0.5 / 3.0 | 2026-09-10 |
| Kill → auto-disarm delay | 5 s (`COM_KILL_DISARM = 5`, kept on purpose) | unchanged |
| Height reference | **GPS** (`EKF2_HGT_REF = 1`, `EKF2_GPS_CTRL = 7`). Switched to barometer and back on 2026-09-13. Read back unchanged at 23:32 | 2026-09-13 (reverted) |
| Flight logging / GPS satellite info | `SDLOG_MODE = 1` (from boot), `GPS_SAT_INFO = 1`. Were 0 / 0. **Written and read back, but active only after the next Pixhawk reboot** | 2026-09-13 23:40 |
| Onboard code | Kill-switch handling, "mode switch must be in Offboard" pre-arm check, GPS and ground-height pre-arm gates, octomap height-band fix, new `forward_move` package | 2026-09-13 |
| Git | Everything after the V5 commit (2026-09-08 22:37) is **not committed** | — |
| Jetson storage | System on a 1 TB NVMe SSD; old SD card kept as a fallback | 2026-09-08 |

---

## 2. Timeline

| Date / time | Area | Change | Status |
| --- | --- | --- | --- |
| before 2026-08-27 | PX4 | Baseline setup: airframe, motor outputs, RC channel map, 1.5 m/s speed limit, telemetry ports (section 3.1) | Partly changed later |
| 2026-07-08 | Code | V1: first commit of `takeoff_land` | Built on by later versions |
| 2026-07-27 | Code | V2: pilot-takeover latch, link-loss abort, `COM_RC_OVERRIDE` check | In effect |
| 2026-07-28 | Code | V3: new `fm_deploy` package (Flow Matching planner on the real drone) | In effect |
| 2026-08-10 | Code | V3 update: GPU-failure handling, depth-only camera, geofence and watchdog | In effect |
| 2026-08-27 14:34–16:27 | Calibration | Accelerometers, magnetometers, level horizon and RC ranges re-calibrated | Replaced on 2026-09-09 |
| **2026-08-27 16:38** | **Incident** | Crash into a tree during auto-landing, caused by GPS height drift. Not a change, but the reason for most changes below | — |
| 2026-08-28 | Code | V4: GPS-quality gate and ground-height trust check before arming | In effect |
| 2026-08-31 | Hardware | Post-crash motor/ESC inspection plan: replace M2, M4 and all propellers | **Not recorded whether done** |
| 2026-09-08 18:16 | Jetson | Operating system moved from SD card to NVMe SSD | Done |
| 2026-09-08 19:57 | Code | Parameter-name typo fixed: `COM_OBL_ACT` → `COM_OBL_RC_ACT` | In effect, uncommitted |
| 2026-09-08 22:37 | Code | V5: terminal status display, launch-argument forwarding fix, new `gps_guard` package | In effect |
| 2026-09-09 00:24–03:36 | PX4 + transmitter | Mode switch rebuilt (Land / Offboard / Position), Offboard switch removed, kill moved to CH11, failsafes set to Land, vertical speeds 0.5 m/s | In effect |
| 2026-09-09 03:49–04:14 | Code | Switching to Land now aborts the mission; arming refused unless the mode switch is in Offboard | In effect, uncommitted |
| 2026-09-09 02:32–13:25 | Calibration | Accelerometers, magnetometers, level horizon re-done; CH5/9/10/11 RC ranges back to defaults | In effect |
| 2026-09-09 13:25–13:27 | Test flights | Logs 88 and 89. Not a change | — |
| 2026-09-10 ~19:00 | PX4 | Hover thrust 0.5 → 0.65, takeoff ramp 3.0 → 1.5 s | In effect, **needs a flight-log check** |
| 2026-09-10 18:52–19:13 | Code | Kill-switch and unexpected-disarm handling in all three flight nodes | In effect, uncommitted, **prop-off test pending** |
| 2026-09-10 22:46 | Code | New `forward_move` package (straight-line flight) | Built, **not flown** |
| 2026-09-13 ~12:50–13:45 | PX4 | Height reference GPS → barometer, GPS height fusion off, then **both reverted** | Reverted |
| 2026-09-13 15:16 | Code | `fm_deploy` octomap height band made relative to the ground (fixes a false OFF-MAP abort) | Built, **not tested on the drone** |
| 2026-09-13 21:56–22:02 | Code + model | `fm_deploy` planner model runs on the GPU with TensorRT; launch default switched to the `.engine`, automatic fallback to `.onnx` on CPU (4.2 f) | Built 22:04, **not tested on the drone** |
| 2026-09-13 23:40 | PX4 | `GPS_SAT_INFO` 0 → 1 (per-satellite info), `SDLOG_MODE` 0 → 1 (log from boot) (section 3.6) | Written and read back; **takes effect only after a Pixhawk reboot** |

---

## 3. PX4 parameters and calibration

### 3.1 Baseline before 2026-08-27 (who and when: unknown)

These values already differed from the PX4 v1.17.0 defaults in the earliest log we have (log 84, 2026-08-27 14:34). The defaults were read from the default-value table stored inside that log. Calibration values, RC ranges and sensor IDs are left out.

**Airframe and motors**

| Parameter | Value | Default | Meaning |
| --- | --- | --- | --- |
| `SYS_AUTOSTART` | 4001 | 0 | Airframe: Generic Quadcopter |
| `CA_ROTOR_COUNT` + `CA_ROTOR0..3_PX/PY/KM` | 4, ±1 positions, ±0.05 | 0 | Quad-X rotor layout and spin directions |
| `PWM_MAIN_FUNC1..4` | 101–104 | 0 | Outputs 1–4 drive motors 1–4 |
| `PWM_MAIN_MIN1..4` / `PWM_MAIN_MAX1..4` | 1100 / 1900 | 1000 / 2000 | ESC signal range |

**RC and flight modes**

| Parameter | Value | Default | Meaning |
| --- | --- | --- | --- |
| `RC_MAP_ROLL/PITCH/THROTTLE/YAW` | 1 / 2 / 3 / 4 | 0 | Stick channels |
| `RC_MAP_FLTMODE` | 5 | 0 | Mode switch on CH5 |
| `RC_MAP_ARM_SW` | 9 | 0 | Arm switch on CH9 |
| `RC_MAP_KILL_SW` | 10 | 0 | Kill on CH10 → **changed 2026-09-09** |
| `RC_MAP_OFFB_SW` | 11 | 0 | Offboard switch on CH11 → **removed 2026-09-09** |
| `COM_FLTMODE1/2/4/5/6` | 2 / 5 / 7 / 0 / 8 | −1 | Position / Return / Offboard / Manual / Stabilized → **changed 2026-09-09** |
| `COM_RC_OVERRIDE` | 3 | 1 | Moving the sticks takes control back in Auto **and** Offboard modes. The check is on stick speed (with `COM_RC_STICK_OV = 30`, about 130 PWM per second) and includes the throttle stick |
| `MAN_ARM_GESTURE` | 0 | 1 | Arming by stick gesture disabled (switch only) |

**Speed and navigation**

| Parameter | Value | Default | Meaning |
| --- | --- | --- | --- |
| `MPC_XY_VEL_ALL` | 1.5 | −10 (off) | Sets all horizontal speed limits at once. `MPC_XY_VEL_MAX`, `MPC_XY_CRUISE`, `MPC_VEL_MANUAL` follow it (defaults 12 / 5 / 10) |
| `NAV_ACC_RAD` | 2.0 | 10.0 | Waypoint acceptance radius (m) |
| `RTL_RETURN_ALT` / `RTL_DESCEND_ALT` | 30 / 10 | 60 / 30 | Return-mode heights (m) |

**Telemetry, sensors, other**

| Parameter | Value | Default | Meaning |
| --- | --- | --- | --- |
| `MAV_1_CONFIG` / `MAV_1_MODE` | 102 / 0 | 0 / 2 | MAVLink on TELEM 2, Normal mode |
| `SER_TEL1_BAUD` / `SER_TEL2_BAUD` | 115200 / 57600 | 57600 / 921600 | Telemetry port speeds |
| `MAV_2_CONFIG` | 1000 | 0 | Second MAVLink instance over Ethernet (UDP 14550, broadcast on) |
| `UAVCAN_ENABLE` | 2 | 0 | DroneCAN sensors, automatic configuration |
| `BAT1_N_CELLS` | 6 | 0 | 6S battery |
| `EKF2_MULTI_IMU` / `SENS_IMU_MODE` | 3 / 0 | 0 / 1 | Multi-EKF running on 3 IMUs |
| `IMU_GYRO_RATEMAX` | 800 | 400 | Gyro rate limit (Hz) |
| `EKF2_RNG_FOG` | 1.0 | 3.0 | Range-finder fog check (no range finder is fitted) |
| `MC_AT_EN` | 1 | 0 | Multicopter autotune module enabled |

Worth knowing — these are still **at their defaults** and were never changed: `EKF2_HGT_REF = 1` (GPS height), `EKF2_GPS_CTRL = 7`, `NAV_RCL_ACT = 2`, `COM_OF_LOSS_T = 1.0`, `COM_DISARM_PRFLT = 10`, `COM_KILL_DISARM = 5`, `CBRK_FLIGHTTERM = 121212` (flight termination disabled), `GF_ACTION = 2` with no fence distance set, `BAT1_CAPACITY = −1`.

### 3.2 2026-08-27 — calibration before the crash flight

Between log 84 (14:34) and log 85 (16:27). Who did it is not recorded (most likely QGroundControl). No other parameter changed that day, so the crash flight (log 86, 16:38) flew with exactly these values.

| What | Change |
| --- | --- |
| Accelerometers (3) | Offsets **and scale factors** changed → a full calibration |
| Magnetometers (2) | Offsets, scales and off-diagonal terms changed → a full calibration |
| Level horizon | `SENS_BOARD_X_OFF` 0.14° → 1.08°, `SENS_BOARD_Y_OFF` 1.52° → 4.27° |
| RC calibration | CH5, CH9, CH10, CH11 ranges 1000–2000 → 1065–1933; small stick-trim changes |

### 3.3 2026-09-09 — mode switch and failsafe setup

**Why.** A live test of every switch (flight controller connected, drone not armed) found three problems:

1. The mode switch has only 3 positions, so only slots 1, 4 and 6 can ever be selected. `COM_FLTMODE2 = Return` and `COM_FLTMODE5 = Manual` never had any effect.
2. The Offboard switch on CH11 forced OFFBOARD mode and made the mode switch ignored. In an emergency, flipping the mode switch would have done nothing. This was reproduced twice.
3. There was no "land now" position on the mode switch.

**Parameter changes** (all read back from the flight controller afterwards)

| Parameter | Before | After | Done by | Reason |
| --- | --- | --- | --- | --- |
| `COM_FLTMODE1` | 2 Position | **11 Land** | User, in QGC (02:55–03:27) | "Land" at one end of SwG (PWM 1065) |
| `COM_FLTMODE2` | 5 Return | −1 | User, in QGC (~02:45) | Slot cannot be reached |
| `COM_FLTMODE4` | 7 Offboard | 7 Offboard | Cleared in QGC, re-written by Claude (02:52) | Offboard in the middle (PWM 1499); net unchanged |
| `COM_FLTMODE5` | 0 Manual | −1 | User, in QGC (~02:45) | Slot cannot be reached |
| `COM_FLTMODE6` | 8 Stabilized | **2 Position** | Claude wrote 11 (Land) at 02:52; the user then swapped Land and Position in QGC | Position at the other end (PWM 1933). Removing Stabilized also means an accidental arm-switch flip in the air is now rejected by PX4 in all three modes (PX4 only disarms after landing in height-controlled modes) |
| `RC_MAP_OFFB_SW` | 11 | **0** | Claude (02:52) | Problem 2 above. Entering Offboard is now the job of the Jetson program; the transmitter only keeps the ability to leave it |
| `RC_MAP_KILL_SW` | 10 | **11** | User (before 02:54), confirmed | Kill on SwF |
| `COM_OBL_RC_ACT` | 0 Position | **4 Land** | Claude (02:52) | If the Jetson stops sending setpoints for 1 s (for example Ctrl-C), PX4 lands |
| `COM_LOW_BAT_ACT` | 0 Warning | **2 Land** | Claude (02:52) | Land at critical battery |
| `MPC_Z_VEL_MAX_UP` | 3.0 | **0.5** | Claude (02:52) | Gentle climbs |
| `MPC_Z_VEL_MAX_DN` | 1.5 | **0.5** | Claude (02:52) | Gentle descents (documented minimum) |
| `MPC_Z_V_AUTO_UP` / `MPC_Z_V_AUTO_DN` | 3.0 / 1.5 | 0.5 / 0.5 | Changed together with the two above | Seen in the log; not written separately |
| `MPC_LAND_SPEED` | 0.7 | **0.6** | Claude (02:52) | Documented minimum. The real landing speed is capped at 0.5 m/s by `MPC_Z_VEL_MAX_DN` |
| `MPC_XY_VEL_ALL` | 1.5 | 1.5 | Claude: 1.0 at 03:35, back to 1.5 at 03:36 | 1.5 kept as a margin against wind; the planner itself flies at ≤ 0.5 m/s |

**Transmitter (AT10II) changes the same night**

| Channel | Setting |
| --- | --- |
| CH5 | ATTITUDE function, SW3 = SwG (3 positions → 1065 / 1499 / 1933), SW2 = NULL |
| CH9 | SwD — arm |
| CH11 | SwF — kill (1065 off, 1933 on) |
| CH6, CH7, CH8, CH10, CH12 | NULL |
| CH3 throttle | 920 – 1787, centre ≈ 1353. No centre detent |

Tip: adding a 2-position switch as SW2 in the ATTITUDE function would give 6 mode positions without a program mix.

**Verification.** A 7-minute switch recording at about 03:30 confirmed 1065 → `AUTO.LAND`, 1499 → `OFFBOARD`, 1933 → `POSCTL`. The arm and kill switches responded, and the mode switch did not disturb the throttle or the other sticks. Logs 88 and 89 (13:25, 13:27) contain all of the values above.

**Calibration later that day** (between log 87 at 02:32 and log 88 at 13:25; who did it is not recorded)

| What | Change |
| --- | --- |
| Accelerometers and magnetometers | Re-calibrated (scale factors changed) |
| Level horizon | `SENS_BOARD_X_OFF` 1.08° → 2.63°, `SENS_BOARD_Y_OFF` 4.27° → −0.56° |
| RC ranges | CH5/9/10/11 `MIN/MAX/TRIM` back to 1000 / 2000 / 1500. Cause not recorded. Mode switching still worked in flight (log 89) |

### 3.4 2026-09-10 — takeoff stability

| Parameter | Before | After | Reason |
| --- | --- | --- | --- |
| `MPC_THR_HOVER` | 0.5 | **0.65** | Measured hover thrust in log 89 was 0.66 |
| `MPC_TKO_RAMP_T` | 3.0 s | **1.5 s** | Shorter time pushing against the ground |

Written by Claude through MAVROS and read back. The drone was not armed.

**The problem.** In log 89 the drone sat armed on the ground for 11.2 s before lifting off, then rocked (roll −3° → +6° → −4° within about 3 s).

**The cause** (checked against the PX4 v1.17.0 source and log 89):

1. During the takeoff ramp, PX4 does not reset the vertical-velocity integrator. While the drone cannot move, the integrator builds up a downward term.
2. At the end of the ramp, the upward command (limited by `MPC_Z_VEL_MAX_UP = 0.5`) is too weak to cancel it, and the hover thrust parameter (0.5) was below the real value (0.66). Thrust could only rise about 0.05 per second.
3. Meanwhile the attitude integrator also built up while the legs held the frame at −3.2° roll. At lift-off that stored error was released as rocking.

A model of this reproduced log 89 (lift-off at 10.6 s vs. 11.2 s measured). With the new values it predicts lift-off after **3.4 s** and 1.8 s of integrator build-up instead of 7.3 s. **This still has to be confirmed with a flight log.**

**Considered and not changed**

- `COM_KILL_DISARM` stays at 5 s. This was the user's choice: an accidental kill can still be undone within 5 s. (PX4 v1.18 removes this parameter and fixes the delay at 5 s.)
- A slower landing (`MPC_LAND_SPEED = 0.2`) was rejected. It is below the documented minimum of 0.6. PX4 would then automatically lower `LNDMC_Z_VEL_MAX` to about 0.167 and make touchdown detection fragile. And with GPS height, a slow descent is more exposed to drift. The program's own descent stays at 0.3 m/s. The request came up again on 2026-09-13; still no change.

### 3.5 2026-09-13 — height-reference experiment (reverted)

**The problem.** The drone sat still and unarmed, but its estimated height (local z) kept wandering. `takeoff_land` refused to arm (`EKF NEVER BECAME TRUSTWORTHY`), and `gps_guard` reported `NOT READY gnd_drift`.

At rest, over 90 s, with good GPS (DGPS fix, 26–30 satellites, HDOP 0.52):

| Source | Peak-to-peak |
| --- | --- |
| Raw GPS height | 3.60 m |
| EKF z (what the controller uses) | 3.63 m |
| Barometer | 0.47 m |

The EKF height followed the GPS height almost exactly.

**What was tried**

| Time | Change | Result |
| --- | --- | --- |
| before 12:54 | `EKF2_HGT_REF` 1 → **0** (barometer) | Not enough. After a restart, z still followed GPS (correlation +0.99), 1.11 m peak-to-peak. The EKF still fused GPS height, which it trusted more than the barometer |
| ~13:00 | `EKF2_GPS_CTRL` 7 → **5** (GPS height fusion off; GPS position and velocity kept). Written by the user | Stable: 0.43 m peak-to-peak over 90 s, and z no longer followed a 2.5 m GPS drift |
| 13:12 | Pixhawk power-cycled | Still stable (0.42 m). But the height on the ground sat at −3.3 to −3.75 m. The cause was not confirmed (likely sensor warm-up after boot) |
| 13:25 | `takeoff_land` run with `max_ground_z:=5.0` | See section 7 (kill switch test) |
| ~13:45 | **Both reverted**: `EKF2_HGT_REF = 1`, `EKF2_GPS_CTRL = 7` | User decision: the barometer was judged too risky outdoors. Read back, drone not armed |

**Net effect: none.** The height estimate again follows GPS height drift. `takeoff_land` may refuse to arm again. A clean Pixhawk power cycle is needed after the revert. No code or launch file was changed for this experiment.

### 3.6 2026-09-13 23:40 — logging from boot, per-satellite GPS info

| Parameter | Before | After | Reason |
| --- | --- | --- | --- |
| `SDLOG_MODE` | 0 (log from arming until disarm) | **1** (log from boot until disarm) | Many attempts on 2026-09-13 never reached arming, so no flight log exists for them. All 13 logs up to 15:07 had mode 0 |
| `GPS_SAT_INFO` | 0 | **1** (publish `satellite_info`) | Check the signal strength of each satellite, to find out whether the GPS antenna was damaged in the 2026-08-27 crash |

**Done by:** Claude, through a minimal MAVROS (`sys_status` + `param` plugins only), at the user's (Jeremy's) explicit request. Written at 23:40:30 and read back after a full parameter pull (1153 parameters). The drone was not armed. The Pixhawk was **not** rebooted.

**Both need a Pixhawk reboot to take effect** (PX4 v1.17.0 source: reboot required for both). Until then the old values stay in effect.

What changes after the reboot:

- `SDLOG_MODE = 1` (PX4 v1.17.0 `logger.cpp`): a log file starts at boot and stops at the first disarm. Every later arming starts a new file, as before. If the drone is never armed, the file runs until power-off. Logs get longer and larger, because the time on the ground is now included.
- `GPS_SAT_INFO = 1`: in the QGC MAVLink Console, `listener satellite_info` shows each satellite and its signal strength. Compare with another receiver at the same place and time; a single reading cannot say "broken".

---

## 4. Onboard software (`~/drone_ws` on the Jetson)

### 4.1 Committed versions (git, author Michael-BJ)

| Version | Commit | Date | What changed |
| --- | --- | --- | --- |
| V1 | `cbf105d` | 2026-07-08 | `takeoff_land` package: `takeoff_land_node` (arm → Offboard → take off → hover → controlled descent → AUTO.LAND), `hold_position_node`, `mode_monitor`, `px4_sensor_reader`, launch files |
| V2 | `e1ef513` | 2026-07-27 | Safety: a mode change away from Offboard is treated as pilot takeover and **latched** — setpoints stop at once and never resume (both nodes). Link loss to the flight controller aborts the mission. `COM_RC_OVERRIDE` is checked before flying. New launch arguments: `land_handoff_alt` (0.25), `rc_override_enabled`, `verify_rc_override_param`, `max_pos_error`, `max_vz`, `max_alt_error` |
| V3 | `e4b8c66` | 2026-07-28 | New `fm_deploy` package: the Flow Matching planner from simulation (planner code unchanged), plus a hardware safety layer (RC override, link loss, geofence, battery, home frame, dry run). Gemini 2 depth bridge, ESDF, trajectory planner, model `fm_planner_20260724_190037`. Launch notes: escape manoeuvres disabled (operator request, 2026-07-27); planner clearance preference raised 0.65 → 0.8 m for the 2026-07-28 test |
| V3 update | `cae5278` | 2026-08-10 | Jetson GPU failures (`CUBLAS_STATUS_ALLOC_FAILED`): a test inference runs at load time and ONNX falls back to CPU. Camera stream reduced to depth only (colour and IR used up the USB 2.0 bandwidth and silently stopped depth). Virtual arena wall in the ESDF. Home-frame goal and geofence, 5 Hz safety watchdog |
| V4 | `a50d0a7` | 2026-08-28 | After the crash. **GPS-quality gate before arming** (`require_gps` true, 3D fix, ≥ 8 satellites, HDOP ≤ 2.0, held 5 s, 120 s timeout). **Ground-height trust check rewritten**: standard deviation, peak-to-peak ≤ `max_ground_drift` (0.20 m) and \|z\| ≤ `max_ground_z` (1.0 m) over a 5 s window; a timeout now aborts. Removed code in `fm_deploy` that replaced a bad ground height with 0 and flew anyway. Incident report `src/INCIDENT_2026-08-27_gps_ekf.md` |
| V5 | `5e1b1e4` | 2026-09-08 22:37 | Terminal monitoring for both flight programs (phase banners, status line; `status_period_s`, `color_output`, `warn_replan_overrun`). `fm_all.launch.py` now forwards safety arguments that were silently ignored before. `hover_settle_s`. New `gps_guard` package: an independent GPS readiness check that publishes `/px4/gps_ready` and never commands the drone |

### 4.2 Changes after V5 (not committed)

All were made in Claude Code sessions with the user's approval. Each edited file has a backup next to it on the Jetson (section 9), and each change was rebuilt with `colcon build`. For (c), (d) and (e) the records also show `py_compile` and `pyflakes` checks, offline logic tests, and a comparison of the installed files with the source.

**(a) 2026-09-08 19:57 — parameter-name typo**

- Files: `takeoff_land_node.py`, `fm_inference_real_node.py`
- `COM_OBL_ACT` does not exist in PX4. It was replaced with `COM_OBL_RC_ACT`. Until then the pre-flight read of this parameter had failed on every run, so its reminder was never printed.
- Made before V5 but not included in that commit.

**(b) 2026-09-09 03:49–04:14 — mode-switch handling**

- **Switching to Land now aborts the mission.** The mode-change detection used to ignore `AUTO.LAND`. So switching to Land and back to Offboard let the old mission continue in the middle of a landing. Files: `takeoff_land_node.py`, `fm_inference_real_node.py`, `hold_position_node.py`.
- **Arming requires the mode switch in Offboard.** New check `_check_rc_offboard_position()`: CH5 must read 1349–1649. The program shows a reminder, then refuses to take off after 20 s. It can be switched off with the launch argument `require_rc_offboard:=false` (added to `takeoff_land.launch.py`, `fm_all.launch.py`, `fm_real.launch.py`).

**(c) 2026-09-10 18:52–19:13 — kill switch and unexpected disarm**

Files: `takeoff_land_node.py`, `hold_position_node.py`, `fm_inference_real_node.py`. Search for `NEW (2026-09-10)` in the code.

| Situation | Program behaviour |
| --- | --- |
| Kill switch (SwF) on before arming | Waits 20 s, then refuses to arm |
| Kill switch turned on after arming starts (CH11 > 1500 for 2 samples) | Stops sending setpoints **for good** → asks PX4 for `AUTO.LAND` → requests a normal disarm every second for up to 15 s (PX4 accepts it only after landing) → exits. Never arms or enters Offboard again |
| Kill undone within 5 s | PX4 restores the motors; the drone should be in Land mode and land by itself. The program does not take over |
| Kill on for more than 5 s | PX4 disarms; undoing the switch does not restart the motors |
| Disarm during the mission that the program did not ask for | Stops setpoints, no mode change, exits |
| `hold_position_node` | Used only in the air; watches the kill switch from start-up |

New ROS parameters (defaults are fine): `kill_switch_enabled` (True), `kill_channel` (11), `kill_on_pwm` (1500).

**Bug fixed:** `_wait_altitude()` did not check whether the drone was still armed. After a kill undone within 5 s, the motors came back and the takeoff continued in Offboard. Log 88 shows exactly this. The avoidance program also used to run its landing routine (and keep sending setpoints) after an unexpected disarm.

Offline logic tests: 22 / 22 passed (no MAVROS, fake RC and arm messages only).

**(d) 2026-09-10 22:46 — new package `forward_move`**

- Flight: take off → hover → fly straight ahead → hold → **land at the end point**.
- Parameters: `forward_distance` (2.0 m, max 10), `forward_speed` (0.3 m/s, range 0.05–1.0), `forward_hold_time` (3.0 s). "Ahead" is the heading when home is locked; yaw stays fixed.
- It inherits `TakeoffLandNode`, so every check, abort path and the kill-switch handling come from `takeoff_land`. Only `run_sequence()` is copied. **If `takeoff_land_node.run_sequence()` changes, update `forward_move` too.**
- **No obstacle avoidance.** Keep `forward_distance` + 2 m clear in front of the nose.
- No existing code or PX4 parameter was changed. Offline tests 24 / 24 passed. Not flown yet.

**(e) 2026-09-13 15:16 — `fm_deploy` octomap height band**

- **Problem** (15:05 flight, see section 7): the ground height was −6.16 m, but the obstacle-map height band was computed from absolute z as [1.30, 3.00] m — 5 to 7 m above the drone. The map therefore had no data at flight height, the program reported `drone is OFF-MAP`, and after 10 s it aborted and landed in place.
- Why it never showed before: with a ground height near 0 both formulas give the same result, and `max_ground_z = 1.0` normally blocks takeoff when the ground height is far from 0.
- **Fix:** `fm_inference_real_node.py` now computes the band relative to the ground: `ground_z + [max(0.35, alt − 0.7), alt + 1.0]`. Search for `NEW (2026-09-13)`. `fm_inference_base.py` was not touched (it is shared with simulation).
- Offline tests 6 / 6 passed. **Still to do:** a `dry_run:=true` check (look for `[REAL] ok Octomap band [...]`), then a flight.

**(f) 2026-09-13 21:56–22:02 — `fm_deploy` planner model on the GPU (TensorRT)**

- **Problem:** the planner model ran on the CPU. torch and onnxruntime cannot use this Jetson's GPU (`cublasCreate` → `CUBLAS_STATUS_ALLOC_FAILED`, open since 2026-08-04). In the 15:05 flight one inference took 420–510 ms of a 830–1440 ms replan.
- **Findings:** native TensorRT is not affected by the cuBLAS bug. Its plugin library needs `libcudla.so.1`, which is in `/usr/local/cuda-12.6/targets/aarch64-linux/lib` but not on the loader path; the node now preloads it, so no launch-file environment change is needed. TensorRT 10.3 returned wrong candidates for this model (max difference 1.87 against `.pth`) because it mis-fuses the `Concat` that joins image and motion features with the following `Expand`. Marking `/encoder/Concat_output_0` as an extra graph output fixes it (found by automatic bisection over all 114 intermediate tensors).
- **New model files** in `src/fm_deploy/model/fm/`: `fm_planner_20260724_190037_trt.onnx` (opset 17, re-exported from the `.pth`, Concat marked) and `fm_planner_20260724_190037_trt_fp32.engine` (FP32, `--noTF32`). The original `.pth` and `.onnx` are untouched. An engine only works with the TensorRT version and GPU it was built on; the build command is in the `fm_inference_node.py` docstring.
- **Code:** `fm_inference_node.py` loads a `.engine` on the GPU, pads the noise to 8 rows when anchor sampling asks for 7, and at start-up compares the engine with the `_trt.onnx` on CPU (`trt_parity_check`, tolerance `trt_parity_tol` 1e-3). On any failure it falls back to that `.onnx` on CPU. `fm_all.launch.py` and `fm_real.launch.py` now default `model_path` to the engine. Search for `NEW (2026-09-13)`. `fm_inference_base.py` was not touched.
- **Checks:** `py_compile`, `pyflakes` clean. Offline tests 10 / 10 (no ROS spin, no MAVROS): engine loads, parity 3.6e-6; 30 uint8 depth inputs with K 7 and 8 match both `.onnx` files within 1.4e-5; an engine built without the fix is refused (difference 2.18) and falls back to CPU; a corrupt engine falls back; an engine without its `.onnx` stops the node; the old `.onnx` path behaves as before. Inference 43.7 ms (GPU) vs 173 ms (CPU, isolated) or 420–510 ms (CPU, in flight).
- **Built** 22:04 with the user's approval (`colcon build --packages-select fm_deploy`). The installed node, both launch files and both model files match the source; the launch default `model_path` is the engine; the offline tests re-run against the installed module passed 10 / 10.
- **Indoor dry runs by the user** (Pixhawk on USB power only, no battery, no GPS):
  - 22:06 `dry_run:=true`: engine loaded on the GPU (`parity OK 3.62e-06`). Stopped at the ground-height check: without GPS the EKF uses barometer height, which read 1.9 → 2.3 m on the floor, above `max_ground_z` 1.0.
  - 22:11 `dry_run:=true max_ground_z:=5.0`: 13 successful replans. The octomap band fix (4.2 e) worked on the real stack: `[3.32, 5.02] odom z = [0.80, 2.50] m above ground`.

  | Run | Inference (median) | Whole replan (median, min–max) | Rest: optimizer + checks (median) |
  | --- | ---: | ---: | ---: |
  | 15:05 flight, ONNX on CPU | 482 ms | 1000 ms (830–1443) | 518 ms |
  | 15:43 dry run, ONNX on CPU | 435 ms | 1082 ms (852–1324) | 648 ms |
  | **22:11 indoor dry run, TensorRT on GPU** | **44 ms** | 1244 ms (360–12703) | **1200 ms** |

  Inference is 10× faster, but the whole replan did not get faster: the CPU part (optimizer and clearance checks) took about twice as long, and in 2 of 13 replans the first candidate failed and a second one had to be solved (4631 and 12703 ms). The scene was different (indoors vs outdoors), so the cause is **not established**. The `bat=65.5V` value is a false reading with no battery connected.
- **Still to do:** an A/B test in the same spot (engine vs `model_path:=...190037.onnx`) while recording `tegrastats`; then a flight.

Also edited on 2026-09-13 14:31: `src/takeoff_land/command.txt` (a notes file with example commands; not reviewed here).

---

## 5. Jetson (companion computer)

| Date | Change |
| --- | --- |
| 2026-09-08 18:16 | Operating system moved from the SD card to a 1 TB NVMe SSD (`/dev/nvme0n1p2`): `fstab` and the boot root partition were updated on the SSD copy. The SD card (`mmcblk0`) stays inserted and unmounted as a fallback with the old system |
| 2026-09-10 | `~/drone_ws/CLAUDE.md` created: hardware facts, safety rules for work on the Jetson, the on-board change log, backup names and restore commands |

Working rules now in place on the Jetson:

- Writing PX4 parameters, anything that can arm or spin the motors, editing code, rebuilding, rebooting and `apt` installs need explicit approval **every time**.
- Back up a file as `<file>.bak-<label>-<time>` before editing it.
- Do not commit or push `~/drone_ws` on the partner's behalf.
- Record every change both on the Jetson (`~/drone_ws/CLAUDE.md`) and in the knowledge base.

---

## 6. Airframe hardware

**After the crash** (analysis on 2026-08-31, from log 86). The drone hung in the tree with motors running.

| Motor | Position | Load while stuck | Recommendation |
| --- | --- | --- | --- |
| M2 | rear left | average 35.3 A, full throttle for 13.5 s | Replace without testing |
| M4 | rear right | average 23.0 A, high throttle for 11.4 s | Replace (strongly advised) |
| M3 | front left | average 2.7 A, one short spike | Use if inspection passes |
| M1 | front right | average 1.1 A, stopped the whole time | Electrically fine; check mechanically |

Also recommended: check all four ESCs (ES-40A, 40 A rating), re-calibrate the ESCs after any replacement, and **replace all four propellers**.

**Whether any of this was done is not recorded.**

**Seen in log 89 (2026-09-09), not yet addressed**

- Vibration while hovering: 9–18 m/s² peak-to-peak on the raw accelerometer. PX4 guidance is 2–3. It was already at this level before the crash.
- Centre of gravity toward the front left: the left motors give about 10% more thrust in hover.
- About 10% thrust difference between the two diagonal motor pairs (yaw needs extra correction).

---

## 7. Tests and flights that exercised these changes

| Date / time | What ran | Result |
| --- | --- | --- |
| 2026-09-09 ~03:30 | Switch recording, not armed | Mode slots, arm and kill channels confirmed |
| 2026-09-09 13:25 (log 88) | `takeoff_land` | No lift-off: the kill switch was toggled on, off and on. Revealed the kill-then-undo bug fixed on 2026-09-10 |
| 2026-09-09 13:27 (log 89) | `takeoff_land`, 36 s | 11.2 s on the ground, then climbed to ~1.9 m. Pilot took over with Position, then Land; landed and disarmed normally. Led to the 2026-09-10 takeoff change |
| 2026-09-13 13:25 | `takeoff_land` with `max_ground_z:=5.0` (barometer height at the time) | Kill switch turned on about 1 s into takeoff → `AUTO.LAND` → disarmed 3 s later; the program did not re-arm. The kill logic worked on the real, armed drone. Whether propellers were fitted is not recorded |
| 2026-09-13 15:05 | `fm_deploy` (`goal_dist:=3.0 target_alt:=2.0 max_alt_error:=0.5 dry_run:=false max_ground_z:=100.0 v_max:=0.3 descent_speed:=0.3`, GPS height) | Takeoff and hover normal. Aborted with OFF-MAP and landed in place → octomap fix (4.2 e). GPS height drifted about +0.9 m in ~40 s, so the real hover height was about 1.1 m instead of 2.0 m. `max_alt_error` cannot catch this, because the estimate itself was wrong |

---

## 8. Open items

1. **Prop-off kill-switch test** (steps in `~/drone_ws/CLAUDE.md`). The 2026-09-13 13:25 run only covered "kill turned on after arming".
2. **Check the 2026-09-10 takeoff change with a flight log** (time from arming to lift-off, rocking at lift-off). No log after log 89 has been downloaded yet; the 2026-09-13 flights are still on the SD card.
3. **`fm_deploy` octomap fix:** dry run, then a flight. For now, a higher `target_alt` (for example 3.0) was suggested because of GPS height drift.
4. **`forward_move`:** prop-off test first (confirm the kill switch still works after arming), then a first flight with `forward_distance:=1.0`.
5. **Height estimate:** still GPS-based, drifting 1.2–3.6 m peak-to-peak at rest on 2026-09-13. Options not taken yet: a downward range finder; barometer height with GPS height fusion off (tried and reverted). The −3.4 m ground offset seen with barometer height was never explained.
6. **Hardware:** confirm whether motors, ESCs and propellers were replaced; reduce vibration; check the centre of gravity and the motor thrust imbalance.
7. **GPU inference:** TensorRT backend added and built on 2026-09-13 (4.2 f). Next: a `dry_run:=true` to confirm the GPU backend and replan times with real depth frames. torch and onnxruntime still cannot use the GPU.
8. **Git:** all changes in section 4.2 and the `forward_move` package are uncommitted. The partner decides how to commit them.
9. **Reboot the Pixhawk** so that `SDLOG_MODE = 1` and `GPS_SAT_INFO = 1` (section 3.6) take effect. Then check that a log starts at boot, and look at per-satellite signal strength with `listener satellite_info`.

---

## 9. Restore points

**Code backups on the Jetson** (next to the original files under `~/drone_ws/src/`)

| Backup | Made | Returns the file to |
| --- | --- | --- |
| `takeoff_land_node.py.bak-20260908`, `fm_inference_real_node.py.bak-20260908` | 2026-09-08 | Before the typo fix |
| `takeoff_land_node.py.bak-AB-034939`, `hold_position_node.py.bak-AB-034939`, `fm_inference_real_node.py.bak-AB-034939` | 2026-09-09 03:49 | Before the "Land aborts the mission" change |
| `takeoff_land.launch.py.bak-L-041435`, `fm_all.launch.py.bak-L-041435`, `fm_real.launch.py.bak-L-041435` | 2026-09-09 04:14 | Launch files before `require_rc_offboard` |
| `takeoff_land_node.py.bak-KILL-20260910-185232`, `hold_position_node.py.bak-KILL-20260910-185232`, `fm_inference_real_node.py.bak-KILL-20260910-185232` | 2026-09-10 18:52 | Before the kill-switch handling |
| `fm_inference_real_node.py.bak-OCTOBAND-20260913-151602` | 2026-09-13 15:16 | Before the octomap band fix |
| `fm_inference_node.py.bak-TRT-20260913-215639`, `fm_all.launch.py.bak-TRT-20260913-215639`, `fm_real.launch.py.bak-TRT-20260913-215639` | 2026-09-13 21:56 | Before the TensorRT backend. Without restoring, `model_path:=...fm_planner_20260724_190037.onnx` also returns to the CPU backend |

After restoring a file, rebuild the package (`colcon build --packages-select <package>`) and check that `install/` matches `src/`. The exact restore commands for the kill-switch and octomap changes are in `~/drone_ws/CLAUDE.md`. Restoring code on the drone needs the user's approval.

**PX4 parameters:** write back the "Before" column of section 3. The mode-slot and kill-channel values only make sense together with the transmitter setup in section 3.3. For section 3.6: `SDLOG_MODE = 0`, `GPS_SAT_INFO = 0`, then reboot the Pixhawk.

---

## 10. Sources and gaps

**Sources**

- Flight logs `log_84` to `log_89` (2026-08-27 and 2026-09-09): start-up parameters compared log by log; defaults read from the logs themselves
- Git history and working-tree diff of `~/drone_ws` on the Jetson
- `~/drone_ws/CLAUDE.md` and `~/drone_ws/src/INCIDENT_2026-08-27_gps_ekf.md` on the Jetson
- Records of the Claude Code sessions on 2026-08-31, 2026-09-08/09 and 2026-09-10 (parameter-write scripts and their read-backs)
- 2026-09-13 23:32 read-only parameter pull and 23:40 write + read-back of `SDLOG_MODE` / `GPS_SAT_INFO` (minimal MAVROS; section 3.6)
- Knowledge-base notes (in Chinese): [[wiki/howto/offboard-flight-procedure]], [[wiki/howto/gps-height-drift-diagnosis]]

**Gaps**

- Changes before 2026-08-27 14:34 are known only as an end state (section 3.1), not when or by whom.
- No flight log after log 89 has been downloaded. Parameter changes after 2026-09-09 13:27 come from session records and the Jetson change log, not from logs.
- Changes made directly in QGC or on the transmitter without a note may be missing. The 2026-08-27 and 2026-09-09 calibrations are known only because they show up in the logs.
- Hardware repairs are not recorded.
- Small offset-only changes of `CAL_ACC*_OFF`, `CAL_MAG*_OFF` and `CAL_BARO*_OFF` between logs look like PX4's automatic offset updates and are not listed.

---

## Glossary

| Term | Meaning |
| --- | --- |
| Offboard | PX4 mode in which an external computer (here the Jetson) sends the position targets |
| Setpoint | A target (position, velocity) sent to PX4. In Offboard mode it must be sent continuously |
| ULog / log N | PX4 flight log file on the Pixhawk SD card, one per arming. "log 89" = `log_89_2026-9-9-13-27-32.ulg` |
| EKF / local z | PX4's state estimator and its height output. The position controller flies on this value |
| `ground_z` | The EKF height recorded while the drone sits on the ground before takeoff; target heights are relative to it |
| SwD / SwF / SwG | Physical switches on the AT10II transmitter |
| MAVROS | ROS 2 bridge between the Jetson programs and PX4 |
| Octomap / ESDF | 3D obstacle map built from the depth camera, and the distance field the planner uses |
