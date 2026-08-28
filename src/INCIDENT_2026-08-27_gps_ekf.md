# Incident report — 2026-08-27: takeoff on a broken position estimate

**Outcome:** the drone lifted off, drifted sideways instead of climbing straight
up, and ended up caught in a tree. Ctrl-C on the Jetson did not stop it.

**Root cause:** the vehicle was allowed to arm and take off while its position
estimate was worthless. Every safeguard that existed at the time only reacted
*after* the drone was already airborne, and their abort path (`AUTO.LAND`)
relied on the same broken estimate that caused the problem.

**Fix:** refuse to leave the ground. Two new pre-arm gates — one on GPS
quality, one on the trustworthiness of `ground_z` — plus removal of a masking
behaviour that hid exactly this failure.

---

## 1. What actually happened

Three runs were logged that afternoon (`~/.ros/log/`). All used
`takeoff_land_node`.

| Run | Log file | `ground_z` accepted | Outcome |
|---|---|---|---|
| 16:26 | `python3_6164_…` | **8.379 m** (std 0.0017) | ABORT "Horizontal drift 1.48 m" **~2.3 ms** after the takeoff command |
| 16:32 | `python3_7028_…` | — | Stopped by user (Ctrl-C) while still IDLE on the ground; `z` had climbed to 15.00 m in telemetry without the drone moving |
| 16:36 | `python3_3198_…` | **5.459 m** (std 0.0166) | Took off, ABORT "Horizontal drift 0.51 m" 3.0 s later → `AUTO.LAND` → tree |

Two details are worth staring at:

- **`ground_z` = 8.379 m and 5.459 m for a drone sitting on the ground.** Local
  frame z should be near zero at takeoff. These numbers were accepted as
  "stable" because they were *steady*, not because they were *right*.
- **1.48 m of "drift" detected 2.3 ms after the takeoff command.** No physical
  aircraft moves 1.48 m in 2.3 ms. That was the position estimate jumping, not
  the drone.

### The z trace that fooled the check

From the 16:36 log, sampled every 2 s while the drone sat **stationary on the
ground**:

```
4.99 → 5.22 → 4.76 → 3.62 → 2.41 → 1.46 → 0.98 → 0.92 → 0.87 → 1.43 → 2.19
→ 2.72 → 3.73 → 4.85 → 5.94 → 7.14 → 8.30 → 9.60 → 10.02 → 10.07 → 9.18
→ 7.79 → 6.40 → 5.76 → 5.53 → 5.45
```

A **±5 metre swing over ~50 seconds** on a vehicle that was not moving.

### Why the stability check passed anyway

`_wait_ekf_stable()` took the standard deviation of the **last ten samples**.
The node samples at 10 Hz, so that is a **one second window**. At the turning
points of a slow swing the one-second spread is tiny — the log recorded
`std=0.0166` — so the check declared success and handed back `ground_z =
5.459 m`.

Steadiness was being used as a proxy for correctness. It is not one. A wrong
number that sits still is still wrong.

### GPS state

Checked live against the same FCU afterwards:

```
fix_type: 0            (0 = no GPS at all)
satellites_visible: 0
eph: 9999, epv: 9999   (HDOP/VDOP 99.99)
h_acc: 4294967295      (position uncertainty unknown / 13151 m once decoded)
```

Nothing in the code looked at any of this. `NavSatFix`, the only GPS topic the
reader forwarded, carries a coarse `status.status` and no satellite count, no
dilution of precision, no accuracy estimate.

---

## 2. Why the existing safeguards did not help

They all fire **after** liftoff, and their escape hatch depends on the very
thing that broke:

| Safeguard | When it fires | Why it did not save the flight |
|---|---|---|
| `max_pos_error` drift check | Airborne | Fired correctly — but only *after* the drone was already flying on a bad estimate |
| `AUTO.LAND` abort path | Airborne | PX4's landing uses the position controller, which uses the same broken estimate → kept drifting while descending |
| EKF "stability" check | Pre-arm | Passed. Measured variance, never plausibility |
| `|ground_z| > 3.0` guard (fm_deploy) | Pre-arm | **Warned, then forced the value to 0.0 and flew anyway** — see §3.3 |

### Why Ctrl-C did nothing

Two independent reasons:

1. **The node had already exited.** `_sanity_abort()` ran 3.0 s after takeoff
   and called `_safe_shutdown()`. By the time a human could react there was no
   process left to interrupt. The log contains no "Stopped by user" line for
   that run (the 16:32 run, which *was* Ctrl-C'd, does have one).
2. **Control had moved to the flight controller.** Once the mode switched to
   `AUTO.LAND`, PX4 executed the landing autonomously. Killing the Jetson
   process — or the whole Jetson — has no effect on an `AUTO.LAND` already in
   progress. That is by design, and it is the correct design; the problem was
   that `AUTO.LAND` itself was flying on bad data.

---

## 3. What was changed

Two packages, both flight nodes and both readers.

### 3.1 New: GPS quality pre-arm gate

The readers now subscribe to `/mavros/gpsstatus/gps1/raw` (`GPSRAW`) and
publish a `gps_quality` block in `/px4/sensors`, plus a once-per-second
terminal line so a bad sky view is visible **before** anyone starts a mission:

```
[GPS+] fix=NO GPS  sats=0  HDOP=99.99  <-- NOT SAFE TO FLY
```

The flight nodes refuse to ARM until quality is good and **stays** good.
MAVLink "unknown" sentinels (255 satellites, 65535 DOP) count as **bad**, not
as "fine" — an unknown fix is exactly the situation that caused this incident.

Files: `takeoff_land/px4_sensor_reader.py`, `takeoff_land/takeoff_land_node.py`,
`fm_deploy/px4_sensor_reader.py`, `fm_deploy/fm_inference_real_node.py`.

### 3.2 Rewritten: `ground_z` trust check

Three independent criteria must now hold **continuously** for `stable_dur`,
because each catches a failure the others miss:

| Criterion | Catches |
|---|---|
| `std < tol` over the window | fast jitter — the original check |
| peak-to-peak `<= max_ground_drift` | **slow drift a short-window std cannot see** |
| `\|mean z\| <= max_ground_z` | **an estimate that is steady but far from zero** — we take off *from the ground* |

The window was also lengthened from 1 s to 5 s.

On timeout the function now returns `None` and the caller aborts. Previously it
returned whatever the mean happened to be and flew on it — "could not confirm
the EKF, so use it anyway" is backwards for a safety check.

Files: `takeoff_land/takeoff_land_node.py`,
`fm_deploy/fm_inference_real_node.py`.

> `fm_deploy/fm_inference_base.py` has the same flaw but was deliberately left
> untouched: it is kept byte-identical to the simulation pipeline so planner
> results stay comparable. Ground truth for takeoff is a hardware pre-flight
> concern, not planner logic, and `fm_inference_real_node` already owned an
> override of that method — so the hardware-grade version lives there.

### 3.3 Removed: a masking behaviour that made things worse

`fm_inference_real_node` used to do this:

```python
self._ground_z = self._wait_ekf_stable()
if abs(self._ground_z) > 3.0:
    self.get_logger().warn("ground_z looks unreasonable -> 0.0")
    self._ground_z = 0.0          # ← and then fly
```

Forcing the number does not fix the estimate it came from. With the EKF reading
8.38 m on the ground, pinning `ground_z` to 0.0 makes `cruise_z = 0 +
target_alt`, so the drone would be commanded to a z it currently reads as ~7 m
**below** itself — ordered to descend into the ground. This now aborts.

---

## 4. Safety parameter reference

### New — GPS quality gate

| Parameter | Default | Meaning | Value during incident |
|---|---|---|---|
| `require_gps` | `true` | Master switch. `false` **only** for indoor flight with a healthy VIO / optical-flow source | — |
| `min_fix_type` | `3` | 3D fix required (GPSRAW enum: 0 = no GPS, 2 = 2D, 3 = 3D, 4 = DGPS, 5/6 = RTK) | **0** ✗ |
| `min_satellites` | `8` | Minimum satellites visible | **0** ✗ |
| `max_hdop` | `2.0` | Horizontal dilution of precision | **99.99** ✗ |
| `gps_wait_timeout` | `120.0` s | How long to wait for a usable fix before giving up | — |
| `gps_stable_dur` | `5.0` s | Quality must hold **continuously** for this long | — |

Skipped automatically when `dry_run:=true` in `fm_deploy` — stage 1 of the test
ladder is meant to run on a bench indoors where there is no sky view at all.

### New — `ground_z` trust criteria

| Parameter | Default | Meaning |
|---|---|---|
| `max_ground_z` | `1.0` m | Max \|local-frame z\| accepted **while on the ground** |
| `ekf_window_s` | `5.0` s | Window length for std and peak-to-peak |
| `max_ground_drift` | `0.20` m | Max peak-to-peak spread within that window |

### Pre-existing — unchanged, all in-flight only

| Parameter | takeoff_land | fm_deploy |
|---|---|---|
| `max_pos_error` | 1.0 m | — |
| `max_vz` | 3.0 m/s | — |
| `max_alt_error` | 1.5 m | 1.0 m |
| `max_home_dist` | — | 15.0 m |

> The incident log shows `pos:0.5m`, i.e. the run used an explicit
> `max_pos_error:=0.5` override, not the default.

---

## 5. Verification

Offline replay against the recorded incident data, **plus one on-vehicle
integration run** (§5.3). No test *flight* has been attempted.

### GPS gate — 9 cases, real node classes, both packages

Instantiated the actual `TakeoffLandNode` and `FMInferenceRealNode`, injected
`gps_quality` dicts, checked the verdict. All 9 passed on both:

| Case | Verdict |
|---|---|
| Incident data (`fix_type=0, sats=0`) | BLOCKED — `fix_type=NO GPS (need >= 3 = 3D fix)` |
| No data from reader yet | BLOCKED |
| 2D fix only | BLOCKED |
| 3D fix, 5 satellites | BLOCKED |
| 3D fix, 12 sats, HDOP 4.5 | BLOCKED |
| Satellites unknown (255 sentinel) | BLOCKED |
| HDOP unknown (65535 sentinel) | BLOCKED |
| 3D fix, 12 sats, HDOP 0.9 | allowed |
| RTK-fixed, 20 sats, HDOP 0.6 | allowed |

The two "allowed" cases matter as much as the blocks: a gate that refuses
everything is not a gate.

### `ground_z` check — replayed against the real incident trace

The logged z values were resampled from the log's 2 s spacing to the 10 Hz the
node actually samples at, then run through a model of the node's
hold-for-`stable_dur` state machine.

| Trace | Old check | New check |
|---|---|---|
| Incident, `takeoff_land` (`stable_dur` 5.0 s) | accepted `ground_z = 0.89 m` | **REFUSED** |
| Incident, `fm_deploy` (`stable_dur` 3.0 s) | accepted `ground_z = 4.91 m` | **REFUSED** |
| Healthy stationary, 1 cm noise | accepted | accepted |
| Healthy stationary, 3 cm noise | — | accepted |

Margin: on the incident trace the new criteria never hold for more than
**0.2 s**, against a 3–5 s requirement (~15×).

### On-vehicle integration run (2026-08-27 evening)

Conditions: FCU powered from the Jetson over USB, **no flight battery**
(motors physically cannot spin), drone indoors on the 3rd floor.

| Stage | Result |
|---|---|
| GPS gate | **PASSED** — `fix_type=3, sats=11, HDOP=1.36, h_acc=1.921 m` |
| `ground_z` check | **REFUSED** — `EKF NEVER BECAME TRUSTWORTHY within the timeout` |
| ARM | never happened |
| TAKEOFF | never happened |
| Shutdown | clean, no stale processes |

Two things worth recording:

- **GPS indoors on the 3rd floor was genuinely good** (11-12 satellites, 3D
  fix, HDOP 1.36), unlike the `fix_type=0 / 0 satellites` measured earlier the
  same day. The gate passed it correctly — it is not a blanket "indoors =
  refuse" rule.
- **The refusal came from the `ground_z` check**, on real grounds: z wandered
  `11.79 -> 13.31 -> 9.97 m` (~3.4 m) while the drone sat motionless on a
  bench, with `std` between 0.108 and 0.336 m against a 0.08 m limit. Same
  class of failure as the incident, caught before liftoff this time.

### How the thresholds were chosen

Not by guessing. A sweep over `ekf_window_s` × `max_ground_drift` measured, for
each combination, the longest continuous "looks good" streak on the incident
trace versus on healthy traces:

| window | drift | incident streak | healthy (1 cm) | healthy (3 cm) |
|---|---|---|---|---|
| 3.0 s | 0.30 m | **3.0 s** ← original attempt, zero margin | 49.1 s | 49.1 s |
| 5.0 s | 0.30 m | 0.7 s | 49.1 s | 49.1 s |
| **5.0 s** | **0.20 m** | **0.2 s** ← chosen | 49.1 s | 49.1 s |
| 5.0 s | 0.10 m | 0.0 s | 49.1 s | **2.4 s** ← too tight, would cause false refusals |

The first attempt at these values (`window 3.0`, `drift 0.30`) let the incident
trace through `fm_deploy` with **exactly zero margin**. It was caught only
because the replay test was made faithful to the real 10 Hz sampling rate — an
earlier version of that test used the log's 2 s spacing and produced a
misleading pass.

---

## 6. Known gaps — not fixed

1. **In-flight GPS degradation is still unhandled.** Every gate added here is
   pre-arm. The realistic tree scenario — good fix on open ground, degrading
   after climbing under canopy — is **not** covered. Note that simply wiring
   "GPS degraded → `AUTO.LAND`" would repeat the original mistake, since
   `AUTO.LAND` depends on the same estimate; the correct degraded-mode action
   is an open design question.

2. **No test flight.** The gates have been exercised end-to-end on the real
   vehicle (§5.3) and correctly refused, but nothing has yet been flown with
   them in place — i.e. no run has been observed *passing* both gates and then
   taking off.

3. **`max_ground_z` assumes the EKF origin is at the takeoff point.** During
   the integration run the drone was on the 3rd floor and z read 10.01 m —
   which may be *physically correct* if the EKF origin sits at ground level.
   Powering on at the takeoff spot (normal practice) puts z near zero and the
   1.0 m default is right; taking off from a roof or balcony while the origin
   is below could cause a false refusal. Raise `max_ground_z` deliberately in
   that case rather than assuming the gate is broken.

4. **`h_acc` is collected but unused.** The reader publishes `h_acc_m`
   (position uncertainty in metres, which read **13151 m** during the incident)
   but no criterion uses it. It is arguably more meaningful than HDOP, which is
   only a geometry factor.

5. **`mavros_tf_broadcaster_node` shutdown race.** Unrelated to this incident:
   it exits with code 1 and an `rcl_shutdown already called` traceback on
   Ctrl-C. Cosmetic — it happens during teardown and affects nothing in flight.

---

## 7. Operating notes

- **Before any flight**, watch the `[GPS+]` line from the reader. If it says
  `NOT SAFE TO FLY`, the gate will refuse anyway — but seeing it early saves
  waiting out the 120 s timeout.
- **Indoor / VIO flight:** `require_gps:=false`. The gate prints a loud warning
  when disabled. Confirming the alternative position source is healthy is then
  entirely your responsibility.
- **`COM_OBL_RC_ACT` is currently `0`** (Position mode) on this FCU — if the
  Jetson stops sending setpoints, control returns to the RC sticks rather than
  the vehicle holding or landing itself. Verify the label in QGroundControl;
  the value read over MAVROS is just an integer.
- **If the gate refuses and you believe it is wrong**, move to open sky and
  retry rather than lowering the thresholds. The thresholds have measured
  margin behind them (§5); loosening them re-opens the failure this document
  describes.
