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

- **Snapshot:** 2026-09-13, 17:13 (Jetson clock) for the onboard code up to 15:17, extended with the later entries in the timeline. PX4 parameters updated at 23:45 (section 3.6). `SDLOG_MODE` updated again on 2026-09-23 (section 3.7); other changes after 2026-09-13 (for example `EKF2_HGT_REF`, `EKF2_GPS_CTRL` on 2026-09-14) are not in this file. The three `*_barometer` packages added at 22:47–23:14 are documented in `~/drone_ws/CLAUDE.md`, not here. The H-Flow bus termination parameter (stored on the H-Flow, not on the Pixhawk) was set on 2026-10-18 (section 3.8). Pixhawk parameters for the H-Flow test mode were changed on 2026-10-18 at 22:14 (section 3.9). H-Flow stage 2 (the H-Flow becomes the height reference, optical flow fused) was set on 2026-10-19 at 00:22, Pixhawk rebooted at 00:26 (section 3.10). Optical flow fusion was turned off again on 2026-10-19 at 10:06 because the H-Flow flow values are too small (section 3.11).
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
| Height reference | **H-Flow range finder** (`EKF2_HGT_REF = 2`) in conditional mode (`EKF2_RNG_CTRL = 1`): used while below 5 m and slower than 2.0 m/s horizontally (to start using it: below 3.5 m and 1.4 m/s); otherwise PX4 falls back to the barometer. The barometer stays fused as the fallback (`EKF2_BARO_CTRL = 1`); GPS height fusion stays off (`EKF2_GPS_CTRL = 5`). Since 2026-10-19 00:26 (section 3.10), **not flown yet**. Before: barometer (`EKF2_HGT_REF = 0`), set by Michael on 2026-09-14 at 23:08 (recorded in `~/drone_ws/CLAUDE.md`); before that GPS (`1` / `7`; switched to barometer and back on 2026-09-13) | 2026-10-19 00:22 |
| Flight logging / GPS satellite info | `SDLOG_MODE = 0` (from arming until disarm), `GPS_SAT_INFO = 1`. `SDLOG_MODE` was 1 (from boot) from 2026-09-13 to 2026-09-23. Both in effect | 2026-09-23 16:17 |
| Onboard code | Kill-switch handling, "mode switch must be in Offboard" pre-arm check, GPS and ground-height pre-arm gates, octomap height-band fix, new `forward_move` package | 2026-09-13 |
| Git | Everything after the V5 commit (2026-09-08 22:37) is **not committed** | — |
| Jetson storage | System on a 1 TB NVMe SSD; old SD card kept as a fallback | 2026-09-08 |
| Downward range finder + optical flow | Holybro H-Flow on CAN1, DroneCAN node 125, bus termination on (`CANNODE_TERM = 1`, stored on the H-Flow). **Height reference since 2026-10-19 (stage 2):** the estimator uses its range as the height reference (row above). **Optical flow fusion off since 2026-10-19 10:06** (`EKF2_OF_CTRL = 0`, section 3.11): hand tests showed that the flow and the H-Flow's own gyro reach the Pixhawk at only about 0.5–0.7 × the true values, so the horizontal velocity comes from GPS only again. Flow direction checked on 2026-10-19 (correct). Mounted 0.195 m below the flight controller. **Not flown yet**. Stage 1 (test mode, estimator ignored it) was on 2026-10-18 at 22:14 (section 3.9), stage 2 on 2026-10-19 at 00:22 (section 3.10) | 2026-10-19 10:06 |

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
| 2026-09-13 23:40 | PX4 | `GPS_SAT_INFO` 0 → 1 (per-satellite info), `SDLOG_MODE` 0 → 1 (log from boot) (section 3.6) | In effect after the 23:52 reboot. `SDLOG_MODE` set back on 2026-09-23 |
| 2026-09-23 16:17 | PX4 + SD card | `SDLOG_MODE` 1 → 0 (log only from arming until disarm); 16 boot logs that were never armed deleted from the SD card (section 3.7) | In effect since the Pixhawk power cycle at about 17:05 |
| 2026-10-18 21:38 | PX4 (on the H-Flow) | H-Flow on CAN1 confirmed online (node 125); its CAN bus termination `CANNODE_TERM` 0 → 1, saved on the H-Flow (section 3.8). Pixhawk parameters unchanged | In effect |
| 2026-10-18 22:14 | PX4 | H-Flow **test mode**: `UAVCAN_SUB_RNG` 0 → 1, `UAVCAN_SUB_FLOW` 0 → 1, `UAVCAN_RNG_MIN` 0 → 0.08, `UAVCAN_RNG_MAX` 999 → 30; `EKF2_RNG_CTRL` 1 → 0 and `EKF2_OF_CTRL` 1 → 0 (temporary). Pixhawk rebooted at 22:14:56 (section 3.9) | Replaced by stage 2 on 2026-10-19 |
| 2026-10-19 00:22 | PX4 | H-Flow **stage 2**: `EKF2_HGT_REF` 0 → 2, `EKF2_RNG_CTRL` 0 → 1, `EKF2_OF_CTRL` 0 → 1, `EKF2_RNG_POS_Z` and `EKF2_OF_POS_Z` 0 → 0.195, `EKF2_MIN_RNG` 0.01 → 0.325, `SENS_FLOW_MAXR` 8.0 → 7.4, `SENS_FLOW_MAXHGT` 100 → 30, `EKF2_RNG_A_VMAX` 1.0 → 2.0. Pixhawk rebooted at 00:26:46 (section 3.10) | In effect, except `EKF2_OF_CTRL` (back to 0 at 10:06). **Changes the height source of both workspaces**; not flown yet |
| 2026-10-19 10:06 | PX4 | Optical flow fusion off: `EKF2_OF_CTRL` 1 → 0, because the H-Flow flow and gyro values reach the Pixhawk at only about 0.5–0.7 × the true values. The H-Flow stays the height reference. No reboot needed (section 3.11) | In effect |

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

### 3.7 2026-09-23 16:17 — logging only while armed again; boot logs deleted

| Parameter | Before | After | Reason |
| --- | --- | --- | --- |
| `SDLOG_MODE` | 1 (log from boot until disarm) | **0** (log from arming until disarm) | A boot log that never sees an arming runs until power-off (about 170 MB per hour). From 2026-09-14 to 2026-09-23 the SD card collected 16 such logs (4.97 GB) with no flight in them |

**Done by:** Claude, through a minimal MAVROS (`sys_status` + `param` plugins only, `ROS_DOMAIN_ID=73`), at the user's (Jeremy's) request. Written at 16:17:40 and read back after a full parameter pull (1150 parameters). The drone was not armed.

**Took effect:** the Pixhawk was power-cycled at about 17:05 (not by Claude). No log started at that boot. All 7 logs from 17:07 to 17:55 start at arming (87 s to 3025 s after boot) and show `SDLOG_MODE = 0` in their start-up parameters.

**Trade-off:** an attempt that is refused before arming leaves no flight log again. The reason is then only in the Jetson's `~/.ros/log/`. To study the time on the ground before takeoff, set it back to 1 for that session.

**Logs deleted** (MAVLink FTP, one file at a time, file size checked before each delete; the `log_transfer` erase was not used because it clears the whole card):

- `/fs/microsd/log/2026-09-14/`: `01_13_35.ulg`, `01_53_58.ulg`, `06_13_37.ulg`
- `/fs/microsd/log/2026-09-15/`: `00_34_00.ulg`, `00_40_21.ulg`, `01_23_40.ulg`, `02_42_59.ulg`, `08_38_49.ulg`, `11_14_28.ulg`
- `/fs/microsd/log/sess100/log100.ulg`
- `/fs/microsd/log/2026-09-21/`: `12_23_46.ulg`, `14_13_29.ulg`
- `/fs/microsd/log/2026-09-22/`: `03_19_17.ulg`, `06_21_59.ulg`, `12_09_23.ulg`
- `/fs/microsd/log/2026-09-23/05_40_34.ulg`

How they were picked: a log whose header time is 1–4 s after boot is a boot log. For each boot log only the first 400 KB and the last 2 MB were downloaded, and `vehicle_status` (`arming_state`, `armed_time`, `takeoff_time`) was read. With mode 1 a boot log stops at the first disarm, so a boot log whose last 40 s never show an arming was never armed.

**Kept:** the 5 boot logs that contain a flight (`2026-09-14/03_13_48`, `2026-09-14/16_05_07`, `2026-09-15/01_24_16`, `2026-09-16/02_46_49`, `2026-09-16/04_41_06`); `2026-09-14/05_51_29` (armed for 1.4 s by the RC switch, no takeoff; used in the height-drift analysis, kept at the user's choice); every log that starts at arming.

**Log numbers changed.** The number shown by QGC and `log_transfer` is the position in the sorted file list, so every log after a deleted one moved down. Old → new: 99 → 97, 100–104 → 98–102, 105 → 103, 106 → 104, 108 → 105, 109–120 → 106–117, 124 → 118, 125–132 → 119–126, 136 → 127, 137–142 → 128–133, 143 → 134, 144–151 → 135–142. Logs 0–96 keep their numbers; the 2026-09-23 afternoon logs are 143–149. The file path (UTC date and time) does not change.

### 3.8 2026-10-18 21:38 — H-Flow CAN bus termination

The Holybro H-Flow (downward range finder + optical flow, PX4 cannode firmware) is connected to CAN1. `uavcan status` on the Pixhawk shows one node: **125, `OK`, `OPERAT`**. This parameter lives **on the H-Flow**, not on the Pixhawk.

| Parameter | Before | After | Reason |
| --- | --- | --- | --- |
| `CANNODE_TERM` (on the H-Flow, node 125) | 0 (termination off) | **1** (termination on) | A CAN bus needs a 120 Ω termination at both ends. The Pixhawk is one end; the H-Flow is the only other device, so it is the other end ("enable termination on the last node", PX4 DroneCAN docs) |

**Done by:** Claude, at the user's (Jeremy's) request, through the PX4 MAVLink shell on `/dev/ttyACM0` (no MAVROS running; the drone was not armed): `uavcan param set 125 CANNODE_TERM 1`, then `uavcan param save 125`.

**Took effect:** the shell printed `Failed to save parameters: 2`, but the Pixhawk console (`dmesg`) shows `save request for node 125 completed OK, restarting.` followed by `restart request for node 125 failed` (the H-Flow restarts at once and cannot answer). The H-Flow came back as `OK`, `OPERAT` and read back `CANNODE_TERM 1` at 21:39, so the value is stored on the H-Flow.

**Bus before the change:** CAN1 had 0 new errors in 17 s while receiving about 463 frames/s; the 58920 errors in its counter were collected before the H-Flow was plugged in. CAN2 has nothing connected, so its error counter keeps rising (about 30/s); that is expected.

**Not changed:** all Pixhawk parameters. PX4 still ignores the H-Flow data (`UAVCAN_SUB_RNG = 0`, `UAVCAN_SUB_FLOW = 0`; `distance_sensor` and `sensor_optical_flow` are never published). Mounting offsets `EKF2_OF_POS_*` and `EKF2_RNG_POS_*` are still 0, `EKF2_RNG_QLTY_T` is 1.0.

### 3.9 2026-10-18 22:14 — H-Flow test mode (PX4 receives the data, the estimator ignores it)

Stage 1 of 2. Goal: read the H-Flow range and optical flow to check their accuracy and direction **without changing how the drone flies**.

| Parameter | Before | After | Reason |
| --- | --- | --- | --- |
| `UAVCAN_SUB_RNG` | 0 | **1** | PX4 subscribes to the H-Flow range measurements (publishes `distance_sensor`) |
| `UAVCAN_SUB_FLOW` | 0 | **1** | PX4 subscribes to the H-Flow optical flow (publishes `sensor_optical_flow`, `vehicle_optical_flow`) |
| `UAVCAN_RNG_MIN` | 0 | **0.08** | Holybro value (80 mm minimum range). In PX4 v1.17.0 `uavcan/sensors/rangefinder.cpp` a non-valid DroneCAN reading (too close / too far) gets quality −1 ("unknown"), which the estimator does **not** reject; only these limits reject it |
| `UAVCAN_RNG_MAX` | 999 | **30** | Holybro value (30 m maximum range) |
| `EKF2_RNG_CTRL` | 1 (conditional) | **0** (temporary) | Keep the estimator off the range finder while it is being checked. With 1, PX4 v1.17.0 makes the range finder the height reference during low, slow flight even when `EKF2_HGT_REF = 0` (`range_height_control.cpp` lines 140–206) |
| `EKF2_OF_CTRL` | 1 | **0** (temporary) | Keep the estimator off the optical flow until its direction is checked |

**Done by:** Claude, at the user's (Jeremy's) request and with explicit approval, through the PX4 MAVLink shell on `/dev/ttyACM0` (no MAVROS running; the drone was not armed). `param set` for each value, then `param save`, read back with `param show -a`. Pixhawk rebooted at 22:14:56 with the shell `reboot` command (`UAVCAN_SUB_*` need a reboot; `UAVCAN_RNG_MIN/MAX` are read when the driver starts). All six values read back the same after the reboot.

**Took effect (22:15):** `distance_sensor` is published: `current_distance` 0.187 m, `min_distance` 0.08, `max_distance` 30, `signal_quality` 100. `DISTANCE_SENSOR` and `OPTICAL_FLOW_RAD` arrive on USB at about 10 Hz. Flow quality 90–100 with the drone standing still. The estimator does not use either: `cs_rng_hgt` False, `cs_opt_flow` False, `cs_baro_hgt` True. 10 s at rest: 18.5–18.8 cm.

**Flight behaviour:** unchanged. The height still follows the barometer and the horizontal position still follows GPS, for both workspaces.

**Next (stage 2, needs a new approval):** after the accuracy test, the flow direction check and measuring the mounting height: `EKF2_HGT_REF` 0 → 2 (range sensor), `EKF2_RNG_CTRL` 0 → 1 (conditional), `EKF2_OF_CTRL` 0 → 1, `EKF2_RNG_POS_Z` and `EKF2_OF_POS_Z` = distance from the flight controller down to the H-Flow lens, `EKF2_MIN_RNG` 0.01 → 0.313, `SENS_FLOW_MAXR` 8.0 → 7.4, `SENS_FLOW_MAXHGT` 100 → 30, then a reboot. The barometer stays on as the fallback (`EKF2_BARO_CTRL = 1`, the user's choice). This changes the height source for **both** workspaces: all of Michael's barometer packages use `baro_gain = 0` since 2026-09-15 and fly on the PX4 height, so they will fly on the H-Flow height too, without code changes. **Done on 2026-10-19 (section 3.10)** with the measured values (`EKF2_MIN_RNG` 0.325, offsets 0.195 m), plus `EKF2_RNG_A_VMAX` 2.0.

### 3.10 2026-10-19 00:22 — H-Flow stage 2 (the estimator uses the H-Flow)

Stage 2 of 2. Goal: at low height and low speed, the PX4 height comes from the H-Flow range instead of the barometer, and the H-Flow optical flow is fused together with GPS for the horizontal velocity.

| Parameter | Before | After | Reason |
| --- | --- | --- | --- |
| `EKF2_HGT_REF` | 0 (barometer) | **2** (range sensor) | The H-Flow becomes the height reference. Needs a reboot |
| `EKF2_RNG_CTRL` | 0 (temporary, section 3.9) | **1** (conditional) | Use the range finder only while low and slow: to start, below 0.7 × `EKF2_RNG_A_HMAX` (3.5 m) and 0.7 × `EKF2_RNG_A_VMAX` (1.4 m/s); once in use, until 5 m / 2.0 m/s. Outside these limits, or when the reading jumps (for example over a box), PX4 falls back to the barometer |
| `EKF2_RNG_A_VMAX` | 1.0 | **2.0** (the maximum PX4 allows) | The box-circling (orbit) policy flies about 0.6 m/s in simulation. With 1.0 the H-Flow would be dropped above 1.0 m/s and only taken back below 0.7 m/s. With 2.0 both limits (2.0 and 1.4 m/s) are above the orbit program's own abort limit (1.5 m/s). Michael's programs fly at most 0.5 m/s |
| `EKF2_OF_CTRL` | 0 (temporary) | **1** | Fuse the optical flow with the GPS velocity |
| `EKF2_RNG_POS_Z`, `EKF2_OF_POS_Z` | 0 | **0.195** | The lens is 0.195 m below the flight controller (IMU): flight-controller centre 0.325 m above the floor, lens 0.125–0.135 m (tape measure, 2026-10-18). X and Y stay 0 (`EKF2_IMU_POS_*` are 0) |
| `EKF2_MIN_RNG` | 0.01 | **0.325** | Flight-controller height above the ground when landed. PX4 v1.17.0 also resets the terrain to "current height − `EKF2_MIN_RNG`" at the moment it detects takeoff (`terrain_control.cpp`), so with 0.01 the whole flight would be about 0.34 m lower than commanded. Must be set together with `EKF2_HGT_REF` |
| `SENS_FLOW_MAXR` | 8.0 | **7.4** | Holybro value (maximum measurable flow rate, rad/s) |
| `SENS_FLOW_MAXHGT` | 100 | **30** | Holybro value (maximum height for the optical flow, m) |

**Not changed, on purpose:** `EKF2_BARO_CTRL = 1` (barometer fused as the fallback, the user's choice), `EKF2_GPS_CTRL = 5` (no GPS height), `EKF2_RNG_A_HMAX = 5`, `SENS_FLOW_ROT = 0`, `SENS_FLOW_MINHGT = 0.08`.

**Done by:** Claude, at the user's (Jeremy's) request with explicit approval on 2026-10-19 at 00:16 ("change what can be changed today; rebooting is fine"), through the PX4 MAVLink shell on `/dev/ttyACM0` (no MAVROS or ROS program running, port free; the drone was not armed). 00:21 read-only dump: all 1912 parameters identical to the 2026-10-18 23:28 dump. 00:22 `param set` for each value, then `param save`; read back with `param show -a`: exactly these 9 changed. Pixhawk rebooted at 00:26:46 with the shell `reboot` command. After the reboot all 9 read back the same. `CAL_BARO0_OFF` (211.66 → 206.22) and `CAL_BARO1_OFF` (410.09 → 402.77) changed by themselves at the reboot: PX4's automatic barometer calibration at boot (see section 10, "Gaps"), not set by us.

**Took effect (00:27:30, 42 s after boot; drone standing indoors on a black box, no GPS fix):** in all three EKF2 instances (`EKF2_MULTI_IMU = 3`): `cs_rng_hgt` True and `cs_rng_terrain` False (= the H-Flow is the height reference), `cs_baro_hgt` True, `cs_opt_flow` True, `cs_rng_kin_consistent` True, `cs_rng_fault` False, `cs_in_air` False; GPS not in use (`cs_gnss_pos` False, indoors). `estimator_aid_src_rng_hgt`: `fused` True, `innovation_rejected` False, observation 0.374 m (= H-Flow reading 0.179 + 0.195) in all three. `estimator_aid_src_optical_flow`: `fused` True in all three. `vehicle_local_position`: `z` 0.03, `dist_bottom` 0.375, `dist_bottom_valid` False (expected: PX4 sets it only while the range finder estimates the terrain, which it does not do while it is the height reference). `vehicle_optical_flow`: `max_flow_rate` 7.4, `max_ground_distance` 30 (the new values are in use). The black surface makes the H-Flow read about 5 cm long; this is known (2026-10-18 test) and not corrected.

**Flight behaviour:** changes for **both** workspaces. While low and slow, the PX4 height is the distance to the ground measured by the H-Flow (relative to the ground under the takeoff point), no longer the barometer. All barometer packages fly on the PX4 height: `baro_gain = 0` in `~/drone_ws`; in `~/drone_ws_jeremy` the gain is picked from the PX4 settings and is 0 as well. No code was changed. The user's copy prints two outdated lines before takeoff ("EKF height follows the BAROMETER", "falls back to the barometer"); the gain is still 0, so they do not matter.

**Still to do before the first flight:** the optical-flow direction check (push forward → forward speed positive, push right → right speed positive). If the direction is wrong, `SENS_FLOW_ROT` must be changed and the Pixhawk rebooted (needs approval). Then a first hover in Position mode. Details in the knowledge base (`wiki/howto/downward-rangefinder-setup.md`). **Done on 2026-10-19 at 09:19: the direction is correct, but the flow values are too small, so the optical flow was turned off at 10:06 (section 3.11).**

### 3.11 2026-10-19 10:06 — optical flow fusion off (the H-Flow flow values are too small)

Hand tests with the propellers off, not armed and no MAVROS running (09:19–10:01, indoors, textured floor; listen-only logger on the Jetson) found:

- **Direction: correct.** Pushing forward gives a positive forward speed, pushing right a positive right speed. Walking 2 m straight along a tape gave almost no sideways component (less than 1°), so the module is not mounted crooked. `SENS_FLOW_ROT = 0` is right.
- **Range: accurate.** With the lens held at 1.00 m (tape measure) it read 1.044 m (102.7–106.8 cm).
- **Flow speed: too small.** Three 2 m walks (taped, ±5 cm) came out as 1.42, 1.28 and 1.33 m (0.64–0.71 ×). The third walk was done with the arms stretched out, so the feet were out of the camera view; same result.
- **The H-Flow's own gyro reads about half.** Compared with the Pixhawk's own gyro (`ATTITUDE`, 93 Hz) while tilting the drone: H-Flow gyro 0.52 × (pitch, R² 0.98) and 0.51 × (roll, R² 0.99); H-Flow flow 0.55 × and 0.63 ×. Shifting the time alignment by ±40 ms does not change the ratios. Because the flow and the H-Flow gyro are both too small, the check in the PX4 docs (flow ≈ H-Flow gyro during pure rotation; 1.00–1.10 here) does not catch this. Compare with the Pixhawk gyro instead.
- **It comes from the H-Flow, not from the Pixhawk.** On the Pixhawk, `sensor_optical_flow` and `vehicle_optical_flow` both run at 55 Hz and every sample carries `integration_timespan_us = 15872` (two PAA3905 frames, 2 × 7936 µs); the Pixhawk passes the samples on without adding them up. The cause (H-Flow firmware or its own settings; `uavcan param list 125` shows `SENS_FLOW_RATE` 70, `SENS_FLOW_SCALE` 1.0, `IMU_GYRO_RATEMAX` 1000, `IMU_INTEG_RATE` 200) is not found yet.

Effect: whenever the estimator relies on the flow, it thinks the drone moves about one third slower than it really does.

| Parameter | Before | After | Reason |
| --- | --- | --- | --- |
| `EKF2_OF_CTRL` | 1 | **0** | Keep the estimator off the wrongly scaled optical flow. The horizontal velocity comes from GPS only, as before stage 2. The H-Flow stays the height reference (`EKF2_HGT_REF = 2`, `EKF2_RNG_CTRL = 1`) |

**Done by:** Claude, at the user's (Jeremy's) request with explicit approval on 2026-10-19 at about 10:05, through the PX4 MAVLink shell on `/dev/ttyACM0` (no `mavros_node` running; the drone was not armed). 10:06 `param show -a` (1912 parameters), read-only queries, `param set EKF2_OF_CTRL 0` (`curr: 1 -> new: 0`), `param save`, read back 0 (saved), `param show -a` again: only this line differs. No reboot needed.

**Took effect (10:06):** in all three EKF2 instances `cs_opt_flow` False, `cs_rng_hgt` True, `cs_rng_terrain` False, `cs_baro_hgt` True, `cs_rng_kin_consistent` True. GPS had a fix indoors at that time (`cs_gnss_pos`, `cs_gnss_vel` True).

**Flight behaviour:** for both workspaces the horizontal estimate is GPS-only again; the height still comes from the H-Flow. Indoors, Position mode can now be armed only while GPS has a fix (with the flow on, the flow gave a horizontal estimate indoors).

**Next:** find the cause on the H-Flow side, repeat the same tests (logger and analysis scripts in the knowledge base: `scripts/jetson-hflow-diag.py`, `scripts/analyze-hflow-diag.py`), and turn the flow back on only after they pass (needs approval). Raw data: `C:\Users\jerem\Downloads\log\hflow-tests-2026-10-19\` on Jeremy's PC.

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
| 2026-10-19 09:19–10:01 | H-Flow hand tests, propellers off, not armed (listen-only logger on the Jetson) | Direction correct; range 1.044 m at 1.00 m; flow speed 0.64–0.71 × the truth (three taped 2 m walks); H-Flow gyro 0.51–0.52 × the Pixhawk gyro. Led to section 3.11 |

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
9. ~~Reboot the Pixhawk so that `SDLOG_MODE = 1` and `GPS_SAT_INFO = 1` (section 3.6) take effect.~~ Done: rebooted at 2026-09-13 23:52 and a log started at boot. `SDLOG_MODE` was set back to 0 on 2026-09-23 (section 3.7). The `listener satellite_info` check is not recorded here.
10. **H-Flow (sections 3.8–3.11):** stage 1 (test mode) done on 2026-10-18 at 22:14, stage 2 on 2026-10-19 at 00:22, direction check on 2026-10-19 at 09:19 (correct). The flow values reach the Pixhawk too small (0.5–0.7 ×), so the optical flow was turned off at 10:06 (`EKF2_OF_CTRL = 0`); the H-Flow is still the height reference. Still to do: find the cause on the H-Flow side and repeat the tests, then turn the flow back on (`EKF2_OF_CTRL = 1`, needs approval); a first hover in Position mode, then check the flight log (`cs_rng_hgt` should stay True in the air). Range accuracy at flight height: one point measured (1.044 m at 1.00 m). Every Pixhawk parameter change needs approval. Steps are in the knowledge base (`wiki/howto/downward-rangefinder-setup.md`).

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

**PX4 parameters:** write back the "Before" column of section 3. The mode-slot and kill-channel values only make sense together with the transmitter setup in section 3.3. For section 3.6: `SDLOG_MODE = 0`, `GPS_SAT_INFO = 0`, then reboot the Pixhawk. For section 3.7: `SDLOG_MODE = 1`, then reboot the Pixhawk. The deleted logs cannot be restored (no copy was kept). For section 3.8 (on the H-Flow, in the MAVLink shell): `uavcan param set 125 CANNODE_TERM 0`, then `uavcan param save 125` (the H-Flow restarts). For section 3.9: `UAVCAN_SUB_RNG = 0`, `UAVCAN_SUB_FLOW = 0`, `UAVCAN_RNG_MIN = 0`, `UAVCAN_RNG_MAX = 999`, `EKF2_RNG_CTRL = 1`, `EKF2_OF_CTRL = 1`, then reboot the Pixhawk. For section 3.10 (back to the stage-1 test mode): `EKF2_HGT_REF = 0`, `EKF2_RNG_CTRL = 0`, `EKF2_OF_CTRL = 0`, `EKF2_RNG_POS_Z = 0`, `EKF2_OF_POS_Z = 0`, `EKF2_MIN_RNG = 0.01`, `SENS_FLOW_MAXR = 8.0`, `SENS_FLOW_MAXHGT = 100`, `EKF2_RNG_A_VMAX = 1.0`, then reboot the Pixhawk. `EKF2_RNG_CTRL` must go to 0 as well: with 1, PX4 makes the range finder the height reference at low height even when `EKF2_HGT_REF = 0`. For section 3.11: `EKF2_OF_CTRL = 1` (no reboot needed); do this only after the H-Flow flow values are fixed and re-tested.

---

## 10. Sources and gaps

**Sources**

- Flight logs `log_84` to `log_89` (2026-08-27 and 2026-09-09): start-up parameters compared log by log; defaults read from the logs themselves
- Git history and working-tree diff of `~/drone_ws` on the Jetson
- `~/drone_ws/CLAUDE.md` and `~/drone_ws/src/INCIDENT_2026-08-27_gps_ekf.md` on the Jetson
- Records of the Claude Code sessions on 2026-08-31, 2026-09-08/09 and 2026-09-10 (parameter-write scripts and their read-backs)
- 2026-09-13 23:32 read-only parameter pull and 23:40 write + read-back of `SDLOG_MODE` / `GPS_SAT_INFO` (minimal MAVROS; section 3.6)
- 2026-09-23 16:17 write + read-back of `SDLOG_MODE`, and partial downloads (first 400 KB, last 2 MB) of the boot logs to check for arming (section 3.7)
- 2026-10-18 22:02 read-only parameter dump and 22:14 write + read-back (before and after the reboot) through the PX4 MAVLink shell; PX4 v1.17.0 source files `range_height_control.cpp`, `height_control.cpp`, `optical_flow_control.cpp`, `optical_flow_fusion.cpp`, `uavcan/sensors/rangefinder.cpp` (section 3.9)
- 2026-10-19 00:21 read-only parameter dump, 00:22 write + read-back, 00:26 reboot and 00:27 read-back of the parameters and the EKF2 status through the PX4 MAVLink shell; PX4 v1.17.0 source files `range_height_control.cpp`, `terrain_control.cpp`, `MulticopterLandDetector.cpp` read on 2026-10-18 (section 3.10)
- 2026-10-19 09:19–10:01 listen-only H-Flow logs on the Jetson (`jetson-hflow-live.py`, `jetson-hflow-diag.py`), and the 10:06 parameter dump, read-only queries, write and read-back through the PX4 MAVLink shell (section 3.11)
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
