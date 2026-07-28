#!/usr/bin/env python3
"""
fm_inference_base.py  (LIGHTWEIGHT inference pipeline — ABSTRACT base class)
==========================================================================
Everything an inference flight needs EXCEPT the trajectory generator: mission
state machine, MAVROS/PX4 plumbing, ESDF + octomap band, MINCO refinement,
safety guards, escape mode, markers. `fm_inference_node.py` subclasses this and
supplies the FM (flow-matching) candidate generator.

2026-07-27: renamed from `nn_inference_node.py` and made abstract when the NN
planner was removed from this package. The three hooks a subclass MUST provide
are `_load_model()`, `_initial_guesses()` and `_warm_up()`; everything else is
model-agnostic. The input encoding (`_form_model_input`) is unchanged and stays
byte-identical to `expert_planner_node._form_model_input`, which is what makes
the labels and the deployed model see the same convention.

Not a mirror of expert_planner_node (data collection) — the rigidity only
needed while collecting the dataset is dropped, the safety guards are kept:
  - Hard post-check on the trajectory (hard_clearance, reject grazing paths)
  - Real-time collision guard + look-ahead guard 0.3-1.5s ahead
  - Time-scaling + clamp v_max
  - Dynamic octomap band following cruise_z + map reset
  - Simple escape mode (8-direction probe) on replan failure / no progress

Reuse: ESDF from esdf_ros2.py, MinJerkPlanner from min_jerk_planner.py.
"""
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, ParamSetV2
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterValue as RclParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty as EmptySrv
from nav_msgs.msg import OccupancyGrid
from pyquaternion import Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
from esdf_ros2 import ESDF
from min_jerk_planner import MinJerkPlanner, PlannerConfig

# ── Model input geometry (must match the expert recorder & the trainer) ──────
IMG_WIDTH         = 640
IMG_HEIGHT        = 480
MOTION_INPUT_SIZE = 24

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class DroneState:
    def __init__(self):
        self.global_pos = np.zeros(3)
        self.global_vel = np.zeros(3)
        self.local_vel  = np.zeros(3)
        self.attitude   = Quaternion()
        self.yaw        = 0.0

    def copy_from(self, other):
        self.global_pos = other.global_pos.copy()
        self.global_vel = other.global_vel.copy()
        self.local_vel  = other.local_vel.copy()
        self.attitude   = other.attitude
        self.yaw        = other.yaw


class TrajectorySegment:
    def __init__(self):
        self.t_start         = 0.0
        self.state_cmd       = None
        self.cmd_hz          = 100
        self.base_total_time = 0.0   # ORIGINAL duration (before time-scaling)
        self.time_scale      = 1.0   # >1 = slowed down so v <= v_max
        self.total_time      = 0.0   # = base_total_time * time_scale

    def is_valid(self):
        return self.state_cmd is not None and len(self.state_cmd) > 0

    def time_remaining(self, t_now):
        if not self.is_valid():
            return 0.0
        return max(0.0, self.total_time - (t_now - self.t_start))

    def invalidate(self):
        self.state_cmd = None

    def sample_at(self, t_now):
        """Sample with TIME-SCALING. If the base trajectory peaks above v_max,
        time_scale > 1 slows it down: the spatial path is identical, only
        traversed over a longer time. Velocity is divided by time_scale,
        acceleration by time_scale^2."""
        if not self.is_valid():
            return None, None, None
        elapsed = t_now - self.t_start
        if elapsed < 0:
            elapsed = 0.0
        if elapsed >= self.total_time:
            last = self.state_cmd[-1]
            return last[0], np.zeros(2), np.zeros(2)
        k = self.time_scale if self.time_scale > 1e-6 else 1.0
        orig_t    = elapsed / k              # time on the original trajectory
        idx_float = orig_t * self.cmd_hz
        idx_low   = int(idx_float)
        idx_high  = min(idx_low + 1, len(self.state_cmd) - 1)
        alpha     = idx_float - idx_low
        s_low  = self.state_cmd[idx_low]
        s_high = self.state_cmd[idx_high]
        pos = s_low[0] * (1 - alpha) + s_high[0] * alpha
        vel = (s_low[1] * (1 - alpha) + s_high[1] * alpha) / k
        acc = (s_low[2] * (1 - alpha) + s_high[2] * alpha) / (k * k)
        return pos, vel, acc

try:
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    HAS_ONNX = False


# Real-time guard (safety, NOT dataset rigidity — do not relax carelessly)
# 2026-07-14 INPUT CONTRACT MIGRATION (Gemini 2): fixed-range depth
# normalization to 10.0 m — MUST be identical to expert_planner_node.py
# (see the note there). A model trained on the per-frame-max dataset
# (before 2026-07-14) is NOT compatible with this input convention.
DEPTH_NORM_MAX_M = 10.0

# ── SAFETY DISTANCES — MEASURED FROM THE CENTER OF MASS ──────────────────────
# 2026-07-24: ported VERBATIM from expert_planner_node.py so the student flies
# under the same safety geometry its labels were produced under. Rationale is
# two-fold:
#   (1) Frame consistency. safe_dis/hard_clearance used to be compared directly
#       against get_edt_dis(), which — because the ESDF inflates obstacles by the
#       body radius — is the distance from the drone BODY EDGE, not its center.
#       Every number therefore carried a hidden +0.30 m you had to add mentally.
#   (2) Expert/student parity. MINCO still refines every FM candidate at
#       inference, so a different safe_dis here means the refinement pulls
#       toward a different geometry than the one that generated the labels.
# The one physical fact linking the frames is BODY_RADIUS_M — the REAL ESDF
# inflation (0.25 m nominal rounds UP to ceil(0.25/0.1) = 3 cells = 0.30 m at
# 0.1 m octomap resolution). Conversion, applied exactly where we compare
# against the raw ESDF:
#       clearance_from_center = get_edt_dis(pt) + BODY_RADIUS_M
#       edge_threshold        = center_threshold - BODY_RADIUS_M   (fed to MINCO)
# The ESDF itself and has_collision are left untouched — only how this node
# INTERPRETS the ESDF and sets its own thresholds changes.
BODY_RADIUS_M      = 0.30   # effective body radius = real ESDF inflation

# Two-tier collision model, stated from the CENTER OF MASS (= expert values):
#   clearance >= safe_dis_center             -> free (cost 0)
#   hard_dis_center <= clr < safe_dis_center -> soft cubic penalty (may enter)
#   clearance <  hard_dis_center             -> hard barrier (nearly impenetrable)
HARD_DIS_CENTER    = 0.60   # m from center -> hard barrier      (= 0.30 body-edge)
SAFE_DIS_CENTER    = 0.65   # m from center -> soft-zone at rest (= 0.35 body-edge)
# Speed-aware margin (expert parity): the reference path is pushed further from
# obstacles in proportion to speed, absorbing tracking error on turns — the
# physical drone always rides slightly outside the reference path.
K_SPEED_MARGIN     = 0.25   # m per (m/s) -> 0.90 m soft-start at cruise 1.0 m/s
# POST-CHECK reject line, DECOUPLED from safe_dis (expert 2026-07-24 rewrite):
# _validate_planned_traj rejects a trajectory only when a point's center-of-mass
# clearance dips below this. Previously the post-check reused hard_clearance as a
# reject line ABOVE the optimizer's own hard barrier, so it discarded exactly the
# grazing trajectories the soft cost was built to accept -> reject-loop near tight
# obstacles. Now the optimizer owns the soft zone and the post-check only vetoes
# what dips below the hard barrier.
POST_CHECK_CLEAR_MIN = HARD_DIS_CENTER - 0.05   # m from center (0.55)
# Escape / look-ahead guard trip line, also from the center of mass. Value keeps
# the pre-rewrite EFFECTIVE behavior (was 0.40 body-edge) — this guard is
# inference-only (the expert has no counterpart), so only its units changed.
GUARD_CLEAR_CENTER   = 0.70   # m from center (= 0.40 body-edge)

LOOKAHEAD_HORIZON_S    = 1.5
LOOKAHEAD_STEP_S       = 0.3
LOOKAHEAD_CHECK_PERIOD = 10    # check every N publish ticks (10 Hz @ cmd_hz=100)
GAP_INVALIDATE         = 2.5   # m, tracking gap -> trajectory discarded immediately

# Simple escape
ESCAPE_VELOCITY   = 0.4
ESCAPE_PROBE_DIST = 1.5
ESCAPE_EXIT_DIST  = 1.0
ESCAPE_MAX_TIME   = 10.0
NO_PROGRESS_TIME  = 8.0
NO_PROGRESS_DELTA = 0.8

# ── Stale-map recovery (2026-07-27) ──────────────────────────────────────────
# The octomap is only reset twice (pre-ARM and at takeoff->FLYING); after that
# it accumulates for the whole multi-leg mission. It also never FORGETS: a voxel
# is cleared only when a ray passes through it, and the depth cone is just 91
# deg wide over 0.5..4.0 m, so anything outside the current view keeps its old
# state indefinitely. Meanwhile every cloud is stamped with the node clock at
# PROCESSING time (see depth_to_pointcloud_node v2 — mandatory, because the
# Gazebo image carries sim-time while the MAVROS TF is wall-clock), so any
# camera/queue latency L smears each point by v*L in world coordinates. Over
# minutes those mis-registered points settle as permanent phantom voxels.
# MEASURED (gauntlet_train2, 2026-07-27): 35/54 ESDF samples matched the world
# geometry within +-2 cells, but 19 samples — all at one spot (26.75, 2.75) —
# read esdf=0.15 where the true clearance was 0.96-1.00 m (~5.4 cells off). The
# drone parked next to that phantom, and because it sat outside the live view it
# could never be cleared: 17+ consecutive replan failures, hovering forever.
# Fix: when replanning keeps failing, wipe the map and let it rebuild from the
# current viewpoint. Fires later than escape (2 failures) so it stays a last
# resort, and is rate-limited so a genuinely blocked corridor is not thrashed.
STUCK_MAP_RESET_FAILS   = 8      # consecutive replan failures before wiping
MAP_RESET_COOLDOWN_S    = 20.0   # min seconds between two wipes
MAP_REBUILD_SETTLE_S    = 1.5    # let the camera repopulate before replanning
# v2.2 FIX (2026-07-09): get_edt_dis() returns 10000.0 ("safe") for any
# point outside the CURRENTLY-MAPPED octomap region -- correct for MINCO's
# collision cost (can't penalize what hasn't been observed yet) but
# pathological for escape's own max-clearance scoring: a bounded bias term
# (center_w <= 1.5) can never outweigh a ~10000 vs ~3 gap, so ANY probe
# direction that happens to poke into unexplored space (common near an
# occluding obstacle cluster, or the arena edge where camera FOV has less
# history) wins regardless of center/goal bias -- confirmed via isolated
# replay of the real wall_gauntlet failure. Cap it here, LOCAL to escape
# scoring only (not the shared ESDF query -- other consumers, e.g. MINCO's
# own collision cost, are correct to treat unobserved space as unpenalized).
# 2.5 m ~= what a genuinely open, already-explored corridor reads as in
# practice (see [INF] esdf= log values) -- "as good as clearly open",
# not an unbeatable outlier that swamps every other direction.
ESCAPE_UNMAPPED_CAP = 2.5

# ── ANTI-COLLISION HARDENING (2026-07-27) ────────────────────────────────────
# Four consecutive gauntlet_train2 runs in the *fair* test mode (GUARDS 1+2,
# invalidate+hover, NO escape) ended in wall contact, each time followed by a
# Gazebo impulse that threw the drone out of the arena, after which the run was
# unrecoverable BY DESIGN (no escape) and just hovered or flew blind:
#   run g8  -> ended (14.6,  9.0, z=-0.30) @ 4.28 m/s
#   run g9  -> ended (14.1, 16.3)          esdf=0.00
#   run ga  -> ended (37.8,-40.0)          esdf=10000, still "Replan ok"
#   run gb  -> ended (17.9,  7.2)          esdf=0.00, hovered ~10 min
# The gb log pins the mechanism exactly. Approaching the G4 wall the drone got
# to centre clearance 0.45 m (esdf 0.15 + BODY_RADIUS_M) at 0.69 m/s, and there
#   [1785150660.676] FLYING pos=(15.8,0.0) esdf=0.15 speed=0.69
#   [1785150660.762] Replan ok #36 cost=2007.6 t=1962ms cand#8/8   <-- ACCEPTED
# 2 s later it was at (17.7,7.7,z=-0.09): contact. Two independent holes let
# that plan through, and a third let the drone arrive that fast in the first
# place. All three are closed below.
#
# HOLE 1 — the post-check bar could fall to the contact line. The adaptive rule
# in _validate_planned_traj (thr = min(hard, cur_clear-0.05)) exists so a drone
# that is ALREADY inside the soft zone can still plan its way out. But it
# lowered the bar unconditionally: at cur_clear 0.45 the bar became 0.40 m from
# the centre = 0.10 m of physical gap (BODY_RADIUS_M 0.30), which tracking error
# at 0.69 m/s eats whole. POST_CHECK_FLOOR_CENTER is the absolute floor that
# relaxation may never cross, and the relaxed band now additionally demands the
# trajectory actually LEAVE (see _validate_planned_traj) rather than merely
# staying as close as it already is — which is what the rule always MEANT.
POST_CHECK_FLOOR_CENTER = 0.45   # m from centre = 0.15 m physical gap
#
# HOLE 2 — no quality floor on the accepted plan. plan_once() only rejects on
# the COLLISION component (weighted_cost[3] > collision_cost_tol); energy, time
# and velocity-violation are unbounded, so a numerically blown-up solve (note
# the "overflow encountered in scalar multiply" RuntimeWarning at
# min_jerk_planner.py:311 seconds earlier) is reported as "Replan ok". Measured
# over 1434 accepted plans in this session's logs: p50=9.0, p75=12.6, p90=24.1
# — then a heavy tail, 87 plans above 50 and 46 above 100, up to 2007.6. Those
# are violently swinging paths, not plans. The gate keeps >93% of normal plans.
MAX_PLAN_COST = 50.0
#
# HOLE 3 — the drone arrived at the wall too fast for a hover to stop it. The
# guard trips at guard_clearance (0.70 m centre = 0.40 m physical) and answers
# with "hold current position", but PX4 still needs its braking distance, which
# at ~0.7-0.9 m/s is of the same order as that margin — so the guard fires and
# the drone coasts in anyway. Raising the trip line instead would refuse the
# 2.0 m gaps this world is built around (centre clearance at mid-gap is only
# ~1.0 m). The fix is to arrive slower: cap speed by the clearance actually
# available, the same relation the planner's K_SPEED_MARGIN already encodes
# (margin needed = k * speed), inverted (speed allowed = margin / k). This is a
# speed limit, not a manoeuvre: it never changes WHERE the drone goes, so it
# cannot rescue or disguise a bad candidate the way escape does.
SPEED_LIMIT_FLOOR   = 0.15   # m/s — still creeps, so a tight gap is threadable
SETPOINT_LEAD_MAX_S = 0.5    # s — cap on how far the setpoint may lead the drone
#
# The look-ahead guard had the same units problem as HOLE 3: a horizon fixed in
# TIME (1.5 s) shrinks in metres exactly when speed makes stopping distance
# grow. Sample far enough along the path to cover the braking distance too.
LOOKAHEAD_MIN_DIST_M    = 2.5   # m of path, whichever is longer than the 1.5 s
LOOKAHEAD_MAX_HORIZON_S = 6.0   # s, cap so a slow crawl does not scan forever
#
# Finally: once out of the mapped world there is nothing left to measure. With
# escape off (correctly — see MEMO_PENGUJIAN.md) such a run cannot recover, so
# it should END, not hover for ten minutes (gb) or keep reporting "Replan ok"
# while flying blind at 4 m/s (ga). Abort -> land -> DONE, with a verdict line.
BLIND_ABORT_S = 20.0   # s continuously off-map / outside the arena -> abort
# Same argument for the OTHER terminal state of a no-escape run: parked at the
# guard line in front of an obstacle the model cannot get past. That is the
# CORRECT outcome in fair mode (a bad candidate must show up as a stall rather
# than be disguised as a manoeuvre — see MEMO_PENGUJIAN.md), but the run is
# over and nothing is left to measure, so it should end itself instead of
# needing the manual cutoff the older protocol notes complain about. Observed
# on the return leg of the 2026-07-27 validation run: parked at (8.8, -2.1),
# clearance 0.60 m, speed 0.00, indefinitely. Landing is not a rescue — the
# drone does not go anywhere it could not already reach.
STUCK_ABORT_S = 90.0   # s without closing on the goal -> abort (0 = never)
# When a guard invalidates the trajectory the drone holds position until the
# NEXT scheduled replan — up to replan_period (1.0 s) of standing still, every
# time. That dead time is most of what reads as "slow and hesitant": the guards
# stop the drone in ~0.1 s but recovery waits out the clock. Asking for a
# replan immediately instead removes the wait without touching a single safety
# threshold. Floored so a guard chattering at 10 Hz cannot start a replan storm
# (a replan costs 40-160 ms and must not crowd out the setpoint stream).
REPLAN_ASAP_MIN_GAP_S = 0.3


class FMInferenceBase(Node):
    """Abstract inference node. Subclasses supply the candidate generator via
    `_load_model`, `_initial_guesses` and `_warm_up`; see fm_inference_node."""

    STATE_IDLE    = "IDLE"
    STATE_TAKEOFF = "TAKEOFF"
    STATE_FLYING  = "FLYING"
    STATE_LANDING = "LANDING"
    STATE_DONE    = "DONE"

    def __init__(self):
        super().__init__("fm_inference_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("goal_x",            20.0)
        self.declare_parameter("target_alt",         2.0)
        # 2026-07-26: default 0.5 -> 1.0 (matches the expert's replan_period).
        # A/B in gauntlet_train2 (guard 0.60, fair mode) showed 1.0 s is slightly
        # SMOOTHER, not worse: median speed 0.53->0.67 m/s, near-stop 14.6%->10.2%,
        # replan-fail/lap 2.64->1.67, hover/lap ~unchanged, 0 explosions either way.
        # The predicted "slower reaction to the live octomap -> longer hovers"
        # backfire did NOT materialize at v=1.0 m/s (map stays ~4 m ahead).
        self.declare_parameter("replan_period",      1.0)
        self.declare_parameter("cmd_hz",           100)
        # 2026-07-14: STAYS 5.0 — follows the expert (the target_pos
        # distribution in the model input must match between training and
        # inference). See the 7.5 -> 5.0 revert note in
        # expert_planner_node.py (it destabilized dense worlds: cost blew up
        # -> escape ping-pong -> physics explosion).
        self.declare_parameter("longitu_step",       5.0)
        self.declare_parameter("lateral_step",       1.0)
        self.declare_parameter("v_max",              1.0)
        self.declare_parameter("arm_timeout",       30.0)
        self.declare_parameter("connect_timeout",   30.0)
        # 2026-07-24: all three are now CENTER-OF-MASS clearances (m), matching
        # expert_planner_node.py — previously body-EDGE values with a hidden
        # +BODY_RADIUS_M. See the constants block above.
        self.declare_parameter("safe_dis",           SAFE_DIS_CENTER)
        self.declare_parameter("hard_clearance",     POST_CHECK_CLEAR_MIN)
        self.declare_parameter("guard_clearance",    GUARD_CLEAR_CENTER)
        self.declare_parameter("collision_cost_tol", 1.0)
        self.declare_parameter("planning_time_ahead", 0.3)
        # ── Anti-collision hardening (2026-07-27) ────────────────────────────
        # Parameters, not constants, so a run can reproduce the old behaviour
        # for comparison: max_plan_cost:=0 disables the quality gate,
        # use_speed_limit:=false the clearance-proportional cap,
        # blind_abort_s:=0 the off-map abort. See the constants block above for
        # the four crashes that motivated each.
        self.declare_parameter("max_plan_cost",  MAX_PLAN_COST)
        self.declare_parameter("use_speed_limit", True)
        self.declare_parameter("blind_abort_s",  BLIND_ABORT_S)
        self.declare_parameter("stuck_abort_s",  STUCK_ABORT_S)
        # ── "neo-parity" levers (2026-07-27) ─────────────────────────────────
        # Three knobs are what separate this node's FLIGHT DYNAMICS from the
        # older legacy_inference_node (= neo_planner_node in neo_planner_ws),
        # which flew visibly faster: planning_time_ahead (above), the
        # speed-aware margin, and the PX4 velocity cap. They are parameters so
        # the old behaviour can be reproduced without editing code. The
        # defaults below ARE the current behaviour — nothing changes unless a
        # launch overrides them.
        #
        # speed_margin_k — K_SPEED_MARGIN in m per (m/s). 0.0 disables the
        #   speed-aware margin, so safe_dis stays fixed like the legacy node.
        self.declare_parameter("speed_margin_k", K_SPEED_MARGIN)
        # px4_vel_cap — what to write to PX4's horizontal speed limits:
        #   == 0  DEFAULT (2026-07-27). Write PX4's STOCK defaults (12 / 5 /
        #         -10) instead of deriving MPC_XY_VEL_MAX/CRUISE from v_max —
        #         PX4's own ceiling/cruise reference decides the physical
        #         limit, not the planner. This is what the legacy node
        #         effectively flew under, since it never wrote MPC_XY_* at
        #         all. (sitl.launch.py deletes rootfs/parameters.bson on every
        #         launch, so PX4 already boots at stock values — writing them
        #         is belt-and-braces for a PX4 started some other way.)
        #         NOTE: the commanded SETPOINT velocity is still shaped/
        #         clamped by v_max (MINCO cost soft penalty + the explicit
        #         clamps below) — that safety net is unchanged; only PX4's
        #         own ceiling/cruise param stopped being tied to v_max.
        #    < 0  automatic: v_max * 1.5 (old default, pre-2026-07-27)
        #    > 0  use this value as the hard cap.
        self.declare_parameter("px4_vel_cap", 0.0)
        self.declare_parameter("depth_max_lag",      0.5)
        self.declare_parameter("auto_reverse",      True)
        # "Pure" ablation (2026-07-06): the original NEO-Planner paper has NO
        # escape mode / reactive guards / stuck-detection — only the learned
        # initializer + MINCO optimization ("the subsequent optimization step
        # ensures flight safety"). Those extra layers are OUR scaffolding on
        # top, and they can mask a genuinely bad NN/FM candidate (drone gets
        # rescued before we see it fail). Set false to strip them out for a
        # head-to-head NN-vs-FM comparison of the MODEL itself: no proximity
        # guard, no look-ahead guard, no no-progress escape, no
        # replan-fail escape. The hard post-check on a NEWLY PLANNED
        # trajectory (_validate_planned_traj) and the tracking-gap
        # invalidation (Guard 3 — discards a stale/unreachable reference, not
        # an obstacle decision) stay ON regardless; they are core plumbing,
        # not safety scaffolding. WARNING: with this off, a bad candidate can
        # genuinely fly into an obstacle (Gazebo PHYSICS EXPLOSION) — that is
        # the point (measuring true failure), but expect to restart SITL.
        self.declare_parameter("use_safety_guards", True)
        # v2.3 (2026-07-10): middle ground for the fair NN-vs-FM comparison.
        # `use_safety_guards:=false` turned wall_gauntlet into a "both models
        # fail" world (measured 2026-07-10: NN stalled at the widest wall, FM
        # collided + physics-exploded) because removing the LOOK-AHEAD guard
        # also removes the only thing compensating for octomap lag -> a
        # candidate that looked clear at plan time flies into a wall the map
        # had not filled in yet. That is an OCTOMAP-latency artifact, not a
        # model-quality signal. This flag keeps ONLY the map-based look-ahead
        # guard (Guard 2) active while use_safety_guards is false, but its
        # action is to INVALIDATE + hover (stop short of the obstacle), NOT to
        # enter the escape maneuver. So the drone is prevented from crashing
        # into an unmapped wall, yet gets NO active rescue (no escape probe,
        # no no-progress recovery, no proximity guard) that could mask a
        # genuinely bad candidate. Same map-safety for both models -> a fair
        # comparison that can actually COMPLETE instead of both crashing.
        # Ignored when use_safety_guards is true (full scaffolding already on).
        self.declare_parameter("use_lookahead_guard", False)
        # v2.2 (2026-07-09): arena bounds -- MISSING entirely before this,
        # unlike expert_planner_node.py which always passes these to ESDF()
        # AND biases escape back toward the center when out of bounds. Root
        # cause of a real wall_gauntlet failure: get_edt_dis() returns
        # 10000.0 ("safe") for any point outside the CURRENTLY-MAPPED
        # octomap region (which only grows where the depth camera has
        # looked, unlike the expert's ground-truth-pcd octomap that covers
        # the whole world quickly). _find_escape_direction() then reliably
        # scores "toward unexplored space" as safest -- near an occluding
        # pole/wall cluster or the arena edge, unexplored is systematically
        # OUTWARD, and nothing pulled the drone back (no arena awareness at
        # all here before this fix) -- escapes compounded in the same
        # direction across replans until the drone got pinned at the arena
        # edge with collapsing altitude. Same defaults as expert_planner_node.
        self.declare_parameter("arena_x_min", -2.0)
        self.declare_parameter("arena_x_max", 25.0)
        self.declare_parameter("arena_y_min", -6.0)
        self.declare_parameter("arena_y_max", 6.0)

        self._model_path     = str(self.get_parameter("model_path").value)
        self._goal_x         = float(self.get_parameter("goal_x").value)
        self._alt            = float(self.get_parameter("target_alt").value)
        self._replan_period  = float(self.get_parameter("replan_period").value)
        self._cmd_hz         = int(self.get_parameter("cmd_hz").value)
        self._longitu_step   = float(self.get_parameter("longitu_step").value)
        self._lateral_step   = float(self.get_parameter("lateral_step").value)
        self._v_max          = float(self.get_parameter("v_max").value)
        self._arm_timeout    = float(self.get_parameter("arm_timeout").value)
        self._conn_timeout   = float(self.get_parameter("connect_timeout").value)
        self._safe_dis       = float(self.get_parameter("safe_dis").value)
        self._hard_clearance = float(self.get_parameter("hard_clearance").value)
        self._guard_clear    = float(self.get_parameter("guard_clearance").value)
        self._coll_tol       = float(self.get_parameter("collision_cost_tol").value)
        self._dt_ahead       = float(self.get_parameter("planning_time_ahead").value)
        self._max_plan_cost  = float(self.get_parameter("max_plan_cost").value)
        self._use_speed_lim  = bool(self.get_parameter("use_speed_limit").value)
        self._blind_abort_s  = float(self.get_parameter("blind_abort_s").value)
        self._stuck_abort_s  = float(self.get_parameter("stuck_abort_s").value)
        self._speed_margin_k = float(self.get_parameter("speed_margin_k").value)
        self._px4_vel_cap    = float(self.get_parameter("px4_vel_cap").value)
        self._depth_max_lag  = float(self.get_parameter("depth_max_lag").value)
        self._auto_reverse   = bool(self.get_parameter("auto_reverse").value)
        self._use_guards     = bool(self.get_parameter("use_safety_guards").value)
        # look-ahead-only mode (see declare): active only when full guards off.
        self._use_lookahead  = (bool(self.get_parameter("use_lookahead_guard").value)
                                and not self._use_guards)
        self._arena_x_min    = float(self.get_parameter("arena_x_min").value)
        self._arena_x_max    = float(self.get_parameter("arena_x_max").value)
        self._arena_y_min    = float(self.get_parameter("arena_y_min").value)
        self._arena_y_max    = float(self.get_parameter("arena_y_max").value)
        self._arena_center   = np.array([
            (self._arena_x_min + self._arena_x_max) / 2.0,
            (self._arena_y_min + self._arena_y_max) / 2.0])

        if not os.path.isfile(self._model_path):
            self.get_logger().error(f"Model not found: {self._model_path}")
            raise FileNotFoundError(self._model_path)

        self._use_pth = self._model_path.endswith('.pth')
        self._device  = torch.device(
            "cuda:0" if torch.cuda.is_available() else "cpu")
        backend_str = self._load_model()

        cfg = PlannerConfig()
        cfg.v_max              = self._v_max
        cfg.T_min              = 0.5
        cfg.T_max              = 5.0
        # safe_dis/hard_dis are held in CENTER-OF-MASS metres (self._safe_dis,
        # HARD_DIS_CENTER) and converted to the BODY-EDGE frame the MINCO cost
        # actually compares against the (inflated) ESDF — same handling as the
        # expert. cfg.safe_dis is overwritten every replan with the adaptive +
        # speed-aware value (see _replan); hard_dis is set once here.
        cfg.safe_dis           = max(0.0, self._safe_dis - BODY_RADIUS_M)
        cfg.hard_dis           = max(0.0, HARD_DIS_CENTER - BODY_RADIUS_M)
        cfg.init_wpts_num      = 2
        cfg.init_T             = 2.5
        cfg.delta_t            = 0.1
        cfg.collision_cost_tol = self._coll_tol
        cfg.opt_tol            = 1e-2
        cfg.weights            = [1.0, 1.0, 1.0, 10000.0]
        self._planner          = MinJerkPlanner(cfg)

        # arena_bounds -> virtual wall in the ESDF (see esdf_ros2.py): cells
        # outside the arena are marked occupied, not just "not yet mapped".
        self._esdf = ESDF(arena_bounds=(
            self._arena_x_min, self._arena_x_max,
            self._arena_y_min, self._arena_y_max))

        self._connected   = False
        self._armed       = False
        self._mode        = ""
        self._drone_state = DroneState()
        self._cruise_z    = None

        self._traj        = TrajectorySegment()
        self._traj.cmd_hz = self._cmd_hz
        self._traj_lock   = threading.Lock()

        self._global_target  = np.array([self._goal_x, 0.0])
        self._target_state   = np.zeros((2, 2))
        self._reached_target = False
        self._mission_state  = self.STATE_IDLE

        self._replan_fail_count = 0
        # Separate from _replan_fail_count, which _enter_escape() zeroes: with
        # full guards the escape path would otherwise keep it under the
        # stale-map threshold forever, so a phantom voxel could ping-pong the
        # escape indefinitely (39-40 escapes observed) without ever wiping the
        # map. This streak clears only on a SUCCESSFUL replan.
        self._map_stall_fails   = 0
        self._last_map_reset_t  = 0.0   # stale-map recovery rate limit
        self._escape_active     = False
        self._escape_vel_xy     = np.zeros(2)
        self._escape_start_t    = None
        self._progress_ref_dist = None
        self._progress_ref_time = None
        self._lookahead_tick    = 0
        # Anti-collision hardening (2026-07-27) — see the constants block.
        self._blind_since   = None    # first tick of a continuous off-map spell
        self._stall_ref_dist = None   # best distance-to-goal seen so far
        self._stall_ref_time = None   # when that best was set
        self._replan_asap    = False  # a guard just invalidated -> replan now
        self._abort_reason  = None    # set -> mission loop lands and reports
        self._n_cost_reject = 0       # plans vetoed by the quality gate
        self._n_graze_reject = 0      # plans vetoed by the post-check
        self._v_limited_min = None    # tightest speed cap actually applied

        self._latest_depth       = None
        self._latest_depth_state = None
        self._latest_depth_time  = None
        self._bridge             = CvBridge()
        self._depth_lock         = threading.Lock()

        self._inference_times = []
        self._replan_count    = 0

        self._pub_sp = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10)

        # ── Visualization: model candidate paths (bimodal fan) + flown path ──
        # Publishes the RAW model guesses (before MINCO) as coarse polylines so
        # RViz shows what the MODEL proposed: NN -> 1 line (unimodal, aimed at
        # obstacle centre); FM -> K lines fanning left/right (bimodal). The
        # MINCO-refined path that is actually flown is drawn separately (yellow).
        # Purely a diagnostic publisher — never touches control.
        self.declare_parameter("publish_markers", True)
        self._publish_markers = bool(
            self.get_parameter("publish_markers").value)
        # /projected_map (the octomap RViz shows) is published in the `odom`
        # frame; the ESDF world coordinates + waypoints share that frame, so
        # markers must live in `odom` to overlay the map (NOT the `map` frame
        # used to label MAVROS setpoints).
        self.declare_parameter("marker_frame", "odom")
        self._marker_frame = str(self.get_parameter("marker_frame").value)
        # depth 10 (was 1): a replan publishes TWO MarkerArrays back-to-back
        # (candidates, then flown) milliseconds apart. With depth 1 the second
        # can evict the first before RViz reads it -> flickering markers.
        self._pub_markers = self.create_publisher(
            MarkerArray, "/planner/candidates", 10) if self._publish_markers \
            else None
        # Highest candidate marker id published last time. Used to DELETE only
        # the now-stale candidate ids instead of a global DELETEALL (which also
        # wiped ns="flown" — see _publish_candidate_markers).
        self._marker_cand_max = 0
        self._marker_escape_on = False
        self.create_subscription(String, "/px4/state",   self._cb_state,   10)
        self.create_subscription(String, "/px4/sensors", self._cb_sensors, 10)
        self.create_subscription(
            Image, "/realsense/depth/float32", self._cb_depth, _SENSOR_QOS)
        self.create_subscription(
            OccupancyGrid, "/projected_map", self._esdf.occupancy_map_cb, 10)

        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client   = self.create_client(SetMode,     "/mavros/set_mode")
        self._param_client  = self.create_client(ParamSetV2,  "/mavros/param/set")
        self._octo_param_client = self.create_client(
            SetParameters, "/octomap_server_unknown/set_parameters")
        self._octo_reset_client = self.create_client(
            EmptySrv, "/octomap_server_unknown/reset")

        self._cmd_timer    = self.create_timer(1.0 / self._cmd_hz, self._publish_cmd)
        self._status_timer = self.create_timer(5.0, self._print_status)

        self.get_logger().info("=" * 62)
        self.get_logger().info("  FM Inference (standalone, relaxed motion)")
        self.get_logger().info("=" * 62)
        self.get_logger().info(f"  Model         : {os.path.basename(self._model_path)}")
        self.get_logger().info(f"  Backend       : {backend_str}")
        self.get_logger().info(f"  v_max         : {self._v_max} m/s")
        self.get_logger().info(f"  Goal X        : {self._goal_x} m  alt={self._alt} m")
        # All three are CENTER-OF-MASS clearances (see BODY_RADIUS_M).
        self.get_logger().info(
            f"  safe_dis      : {self._safe_dis:.2f} m soft / "
            f"{HARD_DIS_CENTER:.2f} m hard  [center-of-mass, "
            f"body radius {BODY_RADIUS_M:.2f} m]")
        self.get_logger().info(
            f"  post-check    : {self._hard_clearance:.2f} m (center) "
            f"| +{self._speed_margin_k:.2f} m per m/s speed margin")
        self.get_logger().info(f"  guard_clear   : {self._guard_clear:.2f} m (center)")
        self.get_logger().info(f"  Replan        : {self._replan_period} s")
        # Flight-dynamics levers (see the declare block): the setpoint leads the
        # drone by roughly dTf x speed, and PX4 turns that standing position
        # error into extra speed until its own cap saturates.
        px4_cap_str = ("stock 12/5 m/s (cap OFF)" if self._px4_vel_cap == 0.0
                       else f"{self._px4_vel_cap:.2f} m/s"
                       if self._px4_vel_cap > 0.0
                       else f"{self._v_max * 1.5:.2f} m/s (auto = v_max x1.5)")
        self.get_logger().info(
            f"  dTf (lead)    : {self._dt_ahead:.2f} s  | PX4 cap: {px4_cap_str}")
        guard_mode = ("FULL (guards+escape)" if self._use_guards
                      else "GUARDS 1+2, invalidate+hover, NO escape"
                      if self._use_lookahead else "OFF (no guards/escape)")
        self.get_logger().info(f"  Guard mode    : {guard_mode}")
        # Anti-collision hardening (2026-07-27) — printed so a run can be told
        # apart from the pre-fix logs at a glance.
        self.get_logger().info(
            f"  Plan gate     : cost <= "
            + (f"{self._max_plan_cost:.0f}" if self._max_plan_cost > 0 else "OFF")
            + f" | post-check floor {POST_CHECK_FLOOR_CENTER:.2f} m")
        self.get_logger().info(
            f"  Speed limit   : "
            + ("clearance-proportional (floor "
               f"{SPEED_LIMIT_FLOOR:.2f} m/s)" if self._use_speed_lim else "OFF")
            + f" | look-ahead >= {LOOKAHEAD_MIN_DIST_M:.1f} m")
        self.get_logger().info(
            f"  Blind abort   : "
            + (f"{self._blind_abort_s:.0f} s off-map -> land"
               if self._blind_abort_s > 0 else "OFF")
            + " | stall abort "
            + (f"{self._stuck_abort_s:.0f} s" if self._stuck_abort_s > 0
               else "OFF"))
        self.get_logger().info("=" * 62)

    # ── Model backend — ABSTRACT, supplied by the subclass ───────────────────

    def _load_model(self):
        """Load the model from self._model_path; return a backend string for
        the startup banner. Implemented by fm_inference_node."""
        raise NotImplementedError(
            "_load_model() must be implemented by a subclass "
            "(see fm_inference_node.FMInferenceNode)")

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _cb_state(self, msg):
        try:
            d = json.loads(msg.data)
            self._connected = d.get("connected", False)
            self._armed     = d.get("armed",     False)
            self._mode      = d.get("mode",      "")
        except Exception:
            pass

    def _cb_sensors(self, msg):
        try:
            d = json.loads(msg.data)
            x  = float(d.get("local_x", 0)); y  = float(d.get("local_y", 0))
            z  = float(d.get("local_z", 0))
            vx = float(d.get("vel_x",  0)); vy = float(d.get("vel_y",  0))
            vz = float(d.get("vel_z",  0)); yaw = float(d.get("yaw", 0))
            self._drone_state.global_pos = np.array([x, y, z])
            self._drone_state.global_vel = np.array([vx, vy, vz])
            self._drone_state.yaw        = yaw
            qw = math.cos(yaw / 2); qz = math.sin(yaw / 2)
            self._drone_state.attitude = Quaternion(qw, 0.0, 0.0, qz)
            self._drone_state.local_vel = (
                self._drone_state.attitude.inverse.rotate(
                    self._drone_state.global_vel))
        except Exception:
            pass

    def _cb_depth(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            if depth.shape != (IMG_HEIGHT, IMG_WIDTH):
                depth = cv2.resize(depth, (IMG_WIDTH, IMG_HEIGHT))
            snap = DroneState()
            snap.copy_from(self._drone_state)
            with self._depth_lock:
                self._latest_depth       = depth.copy()
                self._latest_depth_state = snap
                self._latest_depth_time  = time.time()
        except Exception:
            pass

    # ── Setpoint publisher + real-time guard ────────────────────────────────

    def _publish_cmd(self):
        msg = PositionTarget()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE)

        if (self._mission_state == self.STATE_FLYING
                and self._escape_active and self._cruise_z is not None):
            drone_pos = self._drone_state.global_pos[:2]
            tgt = drone_pos + self._escape_vel_xy * 0.5
            # 2026-07-26: CLAMP the escape setpoint inside the arena (0.5 m inset).
            # Root-cause fix for the "chaotic flight" cascade: once a bad octomap
            # pushed escape past the boundary, EVERY cell outside the arena reads
            # esdf=0.00 (the ESDF virtual wall), so the collision guard re-fired
            # forever and the drone drifted out to x=-15 with no way back. A
            # position setpoint that can never exceed the boundary makes that trap
            # impossible; the center-biased escape direction then pulls it back in.
            tgt[0] = float(np.clip(tgt[0], self._arena_x_min + 0.5,
                                   self._arena_x_max - 0.5))
            tgt[1] = float(np.clip(tgt[1], self._arena_y_min + 0.5,
                                   self._arena_y_max - 0.5))
            msg.position.x = float(tgt[0]); msg.position.y = float(tgt[1])
            msg.position.z = float(self._cruise_z)
            msg.velocity.x = float(self._escape_vel_xy[0])
            msg.velocity.y = float(self._escape_vel_xy[1])
            msg.velocity.z = 0.0
            if np.linalg.norm(self._escape_vel_xy) > 0.05:
                msg.yaw = math.atan2(self._escape_vel_xy[1],
                                     self._escape_vel_xy[0])
            else:
                msg.yaw = self._drone_state.yaw
            self._pub_sp.publish(msg)
            return

        if self._mission_state == self.STATE_FLYING and self._cruise_z is not None:
            t_now = time.time()
            with self._traj_lock:
                pos, vel, _ = self._traj.sample_at(t_now)
            if pos is None:
                pos = self._drone_state.global_pos[:2]
                vel = np.zeros(2)

            drone_pos = self._drone_state.global_pos[:2]

            # Guard 1: the drone's own position is too close to an obstacle.
            # 2026-07-24: clearance compared in the CENTER-OF-MASS frame
            # (get_edt_dis is body-edge); _guard_clear is a center distance.
            # 2026-07-27 FIX: this used to run only under self._use_guards, so
            # look-ahead-only mode had NO check on where the drone actually IS —
            # Guard 2 only validates the PLANNED trajectory ahead. Measured in
            # gauntlet_train2: clearance decayed 0.67 -> 0.47 -> 0.34 -> 0.15
            # (centre 0.45, well under guard_clear 0.60) with nothing firing,
            # until the body touched a wall and Gazebo's contact impulse threw
            # the drone at 35 m/s. Open worlds (poles) never got close enough to
            # expose it. Now it runs in look-ahead-only mode too, with the same
            # invalidate+hover action Guard 2 uses there (still no escape).
            g1_close = (self._esdf.get_edt_dis(drone_pos) + BODY_RADIUS_M
                        < self._guard_clear)
            # 2026-07-27: also trip when the drone itself is OUTSIDE the mapped
            # region. get_edt_dis() reports 10000 ("safe") there, so after a
            # contact impulse threw the drone clear of the map it kept cruising
            # blind through walls for 60+ s with every replan reporting success.
            g1_blind = self._esdf.is_unobserved(drone_pos)
            # 2026-07-27: a drone that is off-map or outside the arena has
            # already lost the run — with escape off (the correct test mode,
            # see MEMO_PENGUJIAN.md) nothing can bring it back. Time the spell
            # and end the mission instead of hovering for ten minutes (run gb)
            # or reporting "Replan ok" while flying blind (run ga).
            if self._blind_abort_s > 0.0 and self._esdf.is_ready():
                if g1_blind or self._is_out_of_bounds(drone_pos):
                    if self._blind_since is None:
                        self._blind_since = t_now
                    elif (t_now - self._blind_since > self._blind_abort_s
                            and self._abort_reason is None):
                        self._abort_reason = (
                            f"drone off-map / outside arena at "
                            f"({drone_pos[0]:.1f},{drone_pos[1]:.1f}) for "
                            f"{self._blind_abort_s:.0f}s — unrecoverable "
                            "without escape")
                else:
                    self._blind_since = None
            if ((self._use_guards or self._use_lookahead)
                    and self._esdf.is_ready() and not self._escape_active
                    and (g1_close or g1_blind)):
                why = "ESDF too close" if g1_close else "drone is OFF-MAP"
                with self._traj_lock:
                    self._traj.invalidate()
                pos, vel = drone_pos, np.zeros(2)
                self._replan_asap = True   # do not wait out replan_period
                if self._use_guards:
                    self.get_logger().warn(
                        f"[INF] Collision guard! {why} — escape",
                        throttle_duration_sec=0.5)
                    self._enter_escape()
                else:
                    self.get_logger().warn(
                        f"[INF] Collision guard! {why} — hover "
                        "(no escape)", throttle_duration_sec=0.5)

            # Guard 2: look-ahead — trajectory 0.3..1.5s ahead checked against the
            # latest ESDF (the map changes after planning). v2.3: also runs in
            # look-ahead-only mode (self._use_lookahead), where its action is
            # invalidate + hover instead of escape (see use_lookahead_guard).
            self._lookahead_tick += 1
            if ((self._use_guards or self._use_lookahead) and self._esdf.is_ready()
                    and not self._escape_active
                    and self._lookahead_tick % LOOKAHEAD_CHECK_PERIOD == 0):
                hit = None
                blind = False
                # 2026-07-27: horizon in TIME alone shrinks in METRES exactly
                # when it matters — at 0.9 m/s the old fixed 1.5 s scanned only
                # ~1.3 m, less than the braking distance, so the guard could
                # see the wall and still not stop before it. Extend the horizon
                # until the sampled arc covers LOOKAHEAD_MIN_DIST_M of path
                # (capped, so a near-stationary drone does not scan forever).
                #
                # The two vetoes below get DIFFERENT horizons, and that split is
                # load-bearing. The clearance test may look as far as we like:
                # it only fires on obstacles the map already knows. The
                # UNOBSERVED test may not — it rests on "the camera reaches 4 m,
                # so anything this close must already be mapped", which holds at
                # ~1.3 m and fails at 2.5 m (the depth cone is 91 deg over
                # 0.5-4.0 m, so off-axis and just-turned-toward cells are
                # legitimately still blank). Measured: extending BOTH made the
                # guard hover almost every cycle — "path ahead UNOBSERVED" went
                # from 4 trips in a whole run to continuous, and the drone
                # crawled at 0.03-0.12 m/s. Braking distance is a property of
                # KNOWN obstacles; blind-space refusal stays where it was.
                speed_now = float(np.linalg.norm(
                    self._drone_state.global_vel[:2]))
                horizon_s = LOOKAHEAD_HORIZON_S
                if speed_now > 0.05:
                    horizon_s = float(np.clip(
                        LOOKAHEAD_MIN_DIST_M / speed_now,
                        LOOKAHEAD_HORIZON_S, LOOKAHEAD_MAX_HORIZON_S))
                blind_horizon_s = LOOKAHEAD_HORIZON_S
                with self._traj_lock:
                    if self._traj.is_valid():
                        for dt in np.arange(LOOKAHEAD_STEP_S,
                                            horizon_s + 1e-6,
                                            LOOKAHEAD_STEP_S):
                            p, _, _ = self._traj.sample_at(t_now + dt)
                            if p is None:
                                break
                            # 2026-07-27: refuse to fly into space the drone has
                            # not actually LOOKED at. Unknown cells read as free
                            # (see esdf_ros2.occupancy_map_cb), so a wall the
                            # octomap has not inserted yet is invisible to the
                            # clearance test below — that is exactly how the G2
                            # wall was hit with every replan reporting success.
                            # The horizon is only LOOKAHEAD_HORIZON_S (1.5 s,
                            # ~2 m at cruise) while the depth camera reaches
                            # 4 m, so anything in this window SHOULD already be
                            # observed; if it is not, stopping is correct.
                            # Only inside blind_horizon_s — see the note above.
                            if (dt <= blind_horizon_s
                                    and self._esdf.is_unobserved(p)):
                                blind = True
                                break
                            # center-of-mass clearance (see BODY_RADIUS_M)
                            d = float(self._esdf.get_edt_dis(p)) + BODY_RADIUS_M
                            if d < self._guard_clear:
                                hit = d
                                break
                if hit is not None or blind:
                    why = ("path ahead UNOBSERVED" if blind
                           else f"clearance ahead {hit:.2f}m")
                    with self._traj_lock:
                        self._traj.invalidate()
                    pos, vel = drone_pos, np.zeros(2)
                    self._replan_asap = True   # do not wait out replan_period
                    if self._use_guards:
                        self.get_logger().warn(
                            f"[INF] Look-ahead guard! {why} — escape",
                            throttle_duration_sec=0.5)
                        self._enter_escape()
                    else:
                        # look-ahead-only: stop short, no escape maneuver.
                        self.get_logger().warn(
                            f"[INF] Look-ahead guard! {why} — hover "
                            "(no escape)", throttle_duration_sec=0.5)

            # Guard 3: large tracking gap -> stale trajectory, discard immediately
            # (without a persistence timer like the dataset pipeline).
            if np.linalg.norm(pos - drone_pos) > GAP_INVALIDATE:
                self.get_logger().warn(
                    "[INF] Large tracking gap — trajectory discarded",
                    throttle_duration_sec=1.0)
                with self._traj_lock:
                    self._traj.invalidate()
                pos, vel = drone_pos, np.zeros(2)

            v_norm = float(np.linalg.norm(vel))
            if v_norm > self._v_max:
                vel = vel * (self._v_max / v_norm)

            # ── Clearance-proportional speed limit (2026-07-27) ──────────────
            # HOLE 3 of the crash analysis: the guards fire in TIME to see a
            # wall but not in DISTANCE to stop before it. The planner already
            # states the relation between speed and the margin it needs —
            # safe_dis_eff = safe_dis + K_SPEED_MARGIN * speed. Inverting it
            # gives the speed the CURRENT clearance can pay for:
            #     v_allow = (clearance - guard_clear) / K_SPEED_MARGIN
            # At mid-gap in a 2.0 m corridor (clearance ~1.0 m) that is 1.2 m/s
            # — above v_max, so open flight and gap threading are untouched.
            # Closing on a wall it decays to SPEED_LIMIT_FLOOR, so the drone
            # arrives at the guard's trip line slowly enough for "hold
            # position" to actually hold. It only ever scales the speed along
            # the path the model chose — direction is never altered, so unlike
            # escape this cannot rescue or mask a bad candidate.
            if (self._use_speed_lim and self._esdf.is_ready()
                    and self._speed_margin_k > 1e-3):
                v_allow, clear_c = self._speed_allowance(drone_pos, vel)
                v_norm = float(np.linalg.norm(vel))
                if v_norm > v_allow:
                    vel = vel * (v_allow / v_norm)
                    self._v_limited_min = (v_allow if self._v_limited_min is None
                                           else min(self._v_limited_min, v_allow))
                    self.get_logger().info(
                        f"[INF] Speed limited {v_norm:.2f} -> {v_allow:.2f} m/s "
                        f"(clearance {clear_c:.2f} m)",
                        throttle_duration_sec=2.0)
                # PX4 chases the POSITION setpoint, so clamping the velocity
                # field alone would not slow it down: a setpoint sitting far
                # ahead is itself a full-throttle command (GAP_INVALIDATE lets
                # it lead by up to 2.5 m). Keep the lead consistent with the
                # speed we just allowed — but ONLY while actually throttling.
                # 2026-07-27: this clamp first ran on every tick, so even in
                # open space the setpoint could lead by no more than
                # v_max * 0.5 = 0.5 m where the pipeline previously allowed up
                # to 2.5 m. Since PX4 converts standing position error into
                # speed, that quietly throttled cruise everywhere and made the
                # flight look hesitant. Binding it to the throttled case keeps
                # the braking behaviour next to obstacles and leaves open-space
                # dynamics exactly as they were.
                if v_allow < self._v_max - 1e-6:
                    lead = pos - drone_pos
                    lead_n = float(np.linalg.norm(lead))
                    lead_max = max(0.15, v_allow * SETPOINT_LEAD_MAX_S)
                    if lead_n > lead_max:
                        pos = drone_pos + lead * (lead_max / lead_n)

            msg.position.x = float(pos[0]); msg.position.y = float(pos[1])
            msg.position.z = float(self._cruise_z)
            msg.velocity.x = float(vel[0]); msg.velocity.y = float(vel[1])
            msg.velocity.z = 0.0
            if abs(vel[0]) > 0.05 or abs(vel[1]) > 0.05:
                msg.yaw = math.atan2(vel[1], vel[0])
            else:
                msg.yaw = self._drone_state.yaw
        else:
            pos      = self._drone_state.global_pos
            target_z = self._cruise_z if self._cruise_z is not None else self._alt
            msg.position.x = float(pos[0]); msg.position.y = float(pos[1])
            msg.position.z = float(target_z)
            msg.velocity.x = 0.0; msg.velocity.y = 0.0; msg.velocity.z = 0.0
            msg.yaw        = self._drone_state.yaw

        self._pub_sp.publish(msg)

    # ── Simple escape ─────────────────────────────────────────────────────────

    def _is_out_of_bounds(self, pos) -> bool:
        return not (self._arena_x_min <= pos[0] <= self._arena_x_max
                    and self._arena_y_min <= pos[1] <= self._arena_y_max)

    def _find_escape_direction(self):
        """v2.2 FIX (2026-07-09): add the SAME arena-center bias
        expert_planner_node.py already uses. Without it, clearance alone
        reliably steers escape toward UNEXPLORED octomap space (which
        get_edt_dis() treats as 10000.0 = "safe", see arena_bounds comment
        at __init__) rather than genuinely open space -- near an occluding
        obstacle cluster or the arena edge, unexplored is systematically
        OUTWARD, so escapes compounded outward across replans with nothing
        pulling back (real wall_gauntlet failure: drone pinned at the arena
        edge, altitude collapsed). center_bias is weak normally (matches
        goal_bias's old 0.2 order of magnitude) but dominates once actually
        out of bounds -- same weighting expert_planner_node.py uses."""
        drone_pos = self._drone_state.global_pos[:2]
        to_goal = self._global_target - drone_pos
        n = np.linalg.norm(to_goal)
        to_goal_dir = to_goal / n if n > 1e-3 else np.array([1.0, 0.0])
        to_center = self._arena_center - drone_pos
        cn = np.linalg.norm(to_center)
        to_center_dir = to_center / cn if cn > 1e-3 else np.array([0.0, 0.0])
        out = self._is_out_of_bounds(drone_pos)
        center_w = 1.5 if out else 0.15
        goal_w   = 0.1 if out else 0.2
        best_score, best_dir = -np.inf, np.array([1.0, 0.0])
        for ang in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            d = np.array([np.cos(ang), np.sin(ang)])
            far = min(self._esdf.get_edt_dis(drone_pos + d * ESCAPE_PROBE_DIST),
                      ESCAPE_UNMAPPED_CAP)
            mid = min(self._esdf.get_edt_dis(drone_pos + d * ESCAPE_PROBE_DIST * 0.5),
                      ESCAPE_UNMAPPED_CAP)
            score = (min(far, mid) + goal_w * float(d @ to_goal_dir)
                     + center_w * float(d @ to_center_dir))
            if score > best_score:
                best_score, best_dir = score, d
        return best_dir

    def _reset_stale_map(self):
        """Wipe the octomap after repeated replan failures (see the
        STUCK_MAP_RESET_* block for the measured failure this addresses).

        Safe to do mid-flight precisely because it only fires when the drone is
        already stuck: the trajectory is invalid, so the drone is holding
        position, and the planner only looks ~5 m ahead while the camera sees
        4 m — a clean map rebuilds well within MAP_REBUILD_SETTLE_S. Rate
        limited so a genuinely blocked corridor is not wiped over and over."""
        now = time.time()
        if now - self._last_map_reset_t < MAP_RESET_COOLDOWN_S:
            return
        if (self._octo_reset_client is None
                or not self._octo_reset_client.service_is_ready()):
            self.get_logger().warn(
                "[INF] Stale-map reset wanted but octomap reset service is "
                "unavailable", throttle_duration_sec=10.0)
            return
        self._last_map_reset_t = now
        self.get_logger().warn(
            f"[INF] {self._map_stall_fails} replans failed — wiping a likely "
            "STALE map (phantom voxels the live view cannot clear)")
        self._call_srv(self._octo_reset_client, EmptySrv.Request(), timeout=5.0)
        time.sleep(MAP_REBUILD_SETTLE_S)
        self._replan_fail_count = 0
        self._map_stall_fails   = 0

    def _enter_escape(self):
        if self._escape_active or not self._esdf.is_ready():
            return
        self._escape_vel_xy  = self._find_escape_direction() * ESCAPE_VELOCITY
        self._escape_active  = True
        self._escape_start_t = time.time()
        self._replan_fail_count = 0
        self._reset_progress()
        with self._traj_lock:
            self._traj.invalidate()
        self.get_logger().warn(
            f"[INF] >>> ESCAPE vel=({self._escape_vel_xy[0]:.2f},"
            f"{self._escape_vel_xy[1]:.2f})")

    def _maybe_exit_escape(self):
        if not self._escape_active:
            return
        timeout = time.time() - self._escape_start_t > ESCAPE_MAX_TIME
        clear   = (self._esdf.get_edt_dis(
            self._drone_state.global_pos[:2]) > ESCAPE_EXIT_DIST)
        if timeout or clear:
            self.get_logger().info("[INF] <<< Exit escape")
            self._escape_active  = False
            self._escape_start_t = None

    def _reset_progress(self):
        self._progress_ref_dist = None
        self._progress_ref_time = None
        # The stall watchdog rearms with it — otherwise a turnaround (distance
        # jumps from ~0 back to the full corridor) would read as "no progress"
        # from the very first tick of the new leg.
        self._stall_ref_dist = None
        self._stall_ref_time = None

    def _check_no_progress(self):
        now = time.time()
        d = float(np.linalg.norm(
            self._drone_state.global_pos[:2] - self._global_target))
        if (self._progress_ref_dist is None
                or d < self._progress_ref_dist - NO_PROGRESS_DELTA):
            self._progress_ref_dist = d
            self._progress_ref_time = now
            return False
        return (now - self._progress_ref_time) > NO_PROGRESS_TIME

    # ── Local target (deterministic — no random seed like the dataset) ──────

    def _set_local_target(self, shorter=False, ref_pos=None):
        curr = self._drone_state.global_pos[:2] if ref_pos is None else ref_pos
        step = 2.0 if shorter else self._longitu_step
        to_goal = self._global_target - curr
        dist = np.linalg.norm(to_goal)
        if dist < step:
            self._target_state[0] = self._global_target
            self._target_state[1] = np.zeros(2)
            return
        fwd = to_goal / dist
        lat = np.array([[fwd[1], -fwd[0]], [-fwd[1], fwd[0]]])
        local_tgt = curr + step * fwd
        lat_flag, lat_move = 0, self._lateral_step
        for _ in range(8):
            if not (self._esdf.is_ready()
                    and self._esdf.has_collision(local_tgt)):
                break
            local_tgt = local_tgt + lat_move * lat[lat_flag]
            lat_flag  = 1 - lat_flag
            lat_move += self._lateral_step
        goal_dir = self._global_target - local_tgt
        n = np.linalg.norm(goal_dir)
        self._target_state[0] = local_tgt
        self._target_state[1] = (goal_dir / n * self._v_max * 0.8
                                 if n > 1e-3 else np.zeros(2))

    # ── RViz markers: model candidates (bimodal fan) + flown path ───────────

    def _publish_candidate_markers(self, guesses, head, tail_xy):
        """Draw each raw model guess as a coarse polyline head->w1->w2->tail.
        Best (rank 0) = green & thick; others coloured by side (left=blue,
        right=red). For NN there is a single green line; for FM the K lines
        fan out to both sides at an ambiguous obstacle = bimodality made
        visible. No effect on control."""
        if not self._publish_markers or self._pub_markers is None:
            return
        z = float(self._cruise_z) if self._cruise_z is not None else self._alt
        hx, hy = float(head[0][0]), float(head[0][1])
        tx, ty = float(tail_xy[0]), float(tail_xy[1])
        ax, ay = tx - hx, ty - hy
        an = math.hypot(ax, ay) or 1.0
        ax, ay = ax / an, ay / an
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        # 2026-07-27 FIX: this used to send Marker.DELETEALL, which is GLOBAL —
        # it also wiped ns="flown" (the yellow flown path). Because
        # _publish_flown_marker() only runs after a SUCCESSFUL replan, every
        # FAILED replan erased the flown path and never redrew it, so the path
        # vanished from RViz until the next success. Now only the candidate ids
        # that are no longer used are deleted, scoped to ns="candidates".
        for stale in range(len(guesses), self._marker_cand_max):
            d = Marker()
            d.header.frame_id = self._marker_frame
            d.header.stamp = now
            d.ns = "candidates"
            d.id = stale
            d.action = Marker.DELETE
            arr.markers.append(d)
        self._marker_cand_max = len(guesses)
        for i, (int_wpts, ts) in enumerate(guesses):
            m = Marker()
            m.header.frame_id = self._marker_frame
            m.header.stamp = now
            m.ns = "candidates"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.10 if i == 0 else 0.05
            w1 = (float(int_wpts[0, 0]), float(int_wpts[1, 0]))
            w2 = (float(int_wpts[0, 1]), float(int_wpts[1, 1]))
            for px, py in [(hx, hy), w1, w2, (tx, ty)]:
                p = Point()
                p.x, p.y, p.z = px, py, z
                m.points.append(p)
            lat = ax * (w2[1] - hy) - ay * (w2[0] - hx)   # +left / -right
            if i == 0:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.2, 1.0
            elif lat >= 0.0:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.5, 1.0, 0.7
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.2, 0.7
            arr.markers.append(m)
        self._pub_markers.publish(arr)

    def _publish_flown_marker(self):
        """Draw the MINCO-refined trajectory that is actually flown (yellow)."""
        if not self._publish_markers or self._pub_markers is None:
            return
        try:
            pos = self._planner.get_pos_array()
        except Exception:
            return
        z = float(self._cruise_z) if self._cruise_z is not None else self._alt
        m = Marker()
        m.header.frame_id = self._marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "flown"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.13
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.95, 0.1, 1.0
        for p in pos:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2]) if len(p) > 2 else z
            m.points.append(pt)
        arr = MarkerArray()
        arr.markers.append(m)
        # A fresh trajectory means escape is over — drop the escape arrow.
        if self._marker_escape_on:
            d = Marker()
            d.header.frame_id = self._marker_frame
            d.header.stamp = m.header.stamp
            d.ns = "escape"
            d.id = 0
            d.action = Marker.DELETE
            arr.markers.append(d)
            self._marker_escape_on = False
        self._pub_markers.publish(arr)

    def _publish_escape_marker(self):
        """2026-07-27 FIX: during escape _replan() returns early, so NO markers
        were published at all — the last flown path stayed frozen on screen
        while the drone flew off on _escape_vel_xy (a raw velocity command with
        no trajectory behind it). That is what looked like "the drone leaves the
        path / the path does not update". The trajectory really IS invalidated
        in _enter_escape(), so drawing it is actively misleading: delete it and
        show the escape direction (magenta arrow) instead."""
        if not self._publish_markers or self._pub_markers is None:
            return
        now = self.get_clock().now().to_msg()
        z = float(self._cruise_z) if self._cruise_z is not None else self._alt
        p0 = self._drone_state.global_pos[:2]
        arr = MarkerArray()
        stale = Marker()
        stale.header.frame_id = self._marker_frame
        stale.header.stamp = now
        stale.ns = "flown"
        stale.id = 0
        stale.action = Marker.DELETE
        arr.markers.append(stale)
        a = Marker()
        a.header.frame_id = self._marker_frame
        a.header.stamp = now
        a.ns = "escape"
        a.id = 0
        a.type = Marker.ARROW
        a.action = Marker.ADD
        a.scale.x, a.scale.y, a.scale.z = 0.12, 0.25, 0.0
        a.color.r, a.color.g, a.color.b, a.color.a = 1.0, 0.1, 0.9, 1.0
        tip = p0 + self._escape_vel_xy * 3.0    # 0.4 m/s cmd -> 1.2 m arrow
        for px, py in ((float(p0[0]), float(p0[1])),
                       (float(tip[0]), float(tip[1]))):
            pt = Point()
            pt.x, pt.y, pt.z = px, py, z
            a.points.append(pt)
        arr.markers.append(a)
        self._marker_escape_on = True
        self._pub_markers.publish(arr)

    def _get_head_state(self):
        """Planning head state: from the active trajectory if it is still healthy,
        otherwise from the drone's actual position."""
        t_now = time.time()
        with self._traj_lock:
            if self._traj.is_valid():
                remaining = self._traj.time_remaining(t_now)
                pos, vel, _ = self._traj.sample_at(t_now + self._dt_ahead)
            else:
                remaining, pos, vel = 0.0, None, None
        head = np.zeros((3, 2))
        drone_pos = self._drone_state.global_pos[:2]
        if (pos is not None and remaining > self._dt_ahead + 0.3
                and np.linalg.norm(pos - drone_pos) < GAP_INVALIDATE):
            head[0], head[1] = pos, vel
        else:
            head[0] = drone_pos
            v = self._drone_state.global_vel[:2].copy()
            s = float(np.linalg.norm(v))
            if s > self._v_max:
                v *= self._v_max / s
            if s < 0.1:
                v = np.zeros(2)
            head[1] = v
        return head

    # ── Model input / candidate generation ───────────────────────────────────

    def _form_model_input(self, depth_img, drone_state, head_pos, head_vel):
        """Encode (depth, state, target) exactly as expert_planner_node records
        it — this convention is what ties the labels to the deployed model."""
        # FIXED-RANGE normalization (DEPTH_NORM_MAX_M) — identical to the expert.
        d = np.nan_to_num(depth_img, nan=0.0,
                          posinf=DEPTH_NORM_MAX_M, neginf=0.0)
        depth_u8 = np.clip(
            d / DEPTH_NORM_MAX_M * 255.0, 0, 255).astype(np.uint8)

        quat = drone_state.attitude
        z = self._cruise_z or self._alt
        init_pos = np.array([head_pos[0], head_pos[1], z])
        init_vel = np.array([head_vel[0], head_vel[1], 0.0])
        tgt_pos  = np.array([self._target_state[0, 0],
                             self._target_state[0, 1], z])
        tgt_vel  = np.array([self._target_state[1, 0],
                             self._target_state[1, 1], 0.0])
        motion_info = np.concatenate([
            drone_state.local_vel,
            quat.rotation_matrix.reshape(-1),
            quat.inverse.rotate(init_pos - drone_state.global_pos),
            quat.inverse.rotate(init_vel - drone_state.global_vel),
            quat.inverse.rotate(tgt_pos  - drone_state.global_pos),
            quat.inverse.rotate(tgt_vel  - drone_state.global_vel),
        ]).astype(np.float32)
        return depth_u8, motion_info

    def _initial_guesses(self, depth_img, drone_state, head_state, tail_state):
        """Warm-start candidates as [(int_wpts, ts), ...], best first.
        Implemented by fm_inference_node (K flow-matching samples, ranked by
        ESDF clearance)."""
        raise NotImplementedError(
            "_initial_guesses() must be implemented by a subclass "
            "(see fm_inference_node.FMInferenceNode)")

    # ── Replan (LIGHTWEIGHT: max 2 attempts, deterministic) ──────────────────

    def _speed_allowance(self, drone_pos, vel):
        """Speed the CURRENT clearance can pay for -> (v_allow, clearance).

        Inverts the planner's own speed/margin relation (safe_dis_eff =
        safe_dis + K_SPEED_MARGIN * speed), so the drone reaches the guard trip
        line slowly enough that "hold position" actually holds — HOLE 3 of the
        2026-07-27 crash analysis.

        The margin is scaled by how directly the drone is CLOSING on the
        obstacle. The ESDF gradient points away from the nearest one, so -grad
        points at it and closing = max(0, v_hat . -grad_hat) is 1 heading
        straight at it, ~0 flying parallel, 0 flying away. Without that factor
        the cap also throttles a drone merely passing a wall at arm's length:
        replayed over this session's logged clearances, an omnidirectional cap
        sat at its floor for 24% of on-map samples, mostly corridor transits
        with the wall beside the drone rather than ahead of it.

        Only the speed along the path is scaled — the direction is never
        touched, so unlike escape this cannot rescue or disguise a bad
        candidate."""
        clear_c = float(self._esdf.get_edt_dis(drone_pos)) + BODY_RADIUS_M
        closing = 1.0
        v_norm = float(np.linalg.norm(vel))
        if v_norm > 1e-3:
            g = np.asarray(self._esdf.get_edt_grad(drone_pos), dtype=float)
            gn = float(np.linalg.norm(g))
            if gn > 1e-6:
                closing = max(0.0, float((vel / v_norm) @ (-g / gn)))
        if closing < 1e-3:
            return self._v_max, clear_c        # not approaching anything
        return float(np.clip(
            (clear_c - self._guard_clear) / (self._speed_margin_k * closing),
            SPEED_LIMIT_FLOOR, self._v_max)), clear_c

    def _check_plan_quality(self):
        """Reject a numerically blown-up solve before it is ever flown.

        plan_once() only vetoes the COLLISION component (weighted_cost[3] vs
        collision_cost_tol); costs [energy, time, velocity-violation] carry
        weight 1 each and are unbounded, so an L-BFGS-B solve that diverges is
        still reported as "Replan ok". Run gb accepted cost=2007.6 (normal:
        p50=9.0, p90=24.1) seconds after min_jerk_planner.py:311 logged an
        arithmetic overflow, and flew it into a wall. The gate is on the TOTAL
        because that is what carries the divergence — the collision term alone
        looked healthy in exactly the plan that killed the run."""
        if self._max_plan_cost <= 0.0:
            return                      # gate disabled by parameter
        cost = self._planner.final_cost
        if cost is None or not np.isfinite(cost):
            self._n_cost_reject += 1
            raise ValueError("Plan cost is not finite")
        if cost > self._max_plan_cost:
            self._n_cost_reject += 1
            raise ValueError(
                f"Plan cost {cost:.1f} > max {self._max_plan_cost:.1f} "
                "(diverged solve)")

    def _validate_planned_traj(self):
        """Hard post-check (2026-07-24: CENTER-OF-MASS frame, expert parity):
        reject a trajectory whose center-of-mass clearance dips below
        hard_clearance (= POST_CHECK_CLEAR_MIN, set just under the optimizer's
        hard barrier and DECOUPLED from safe_dis, so the post-check no longer
        throws away the grazing trajectories the soft cost is meant to accept).
        Adaptive threshold kept: if the drone is ALREADY closer than that, do
        not reject a trajectory that takes it OUT.

        2026-07-27 — the adaptive branch used to be written as
            thr = min(hard_clearance, max(cur_clear - 0.05, 0.1))
        which lowered the bar for EVERY trajectory whenever the drone was
        close, including trajectories that simply stayed as close as it already
        was. That is how run gb accepted a path grazing 0.40 m from the centre
        (0.10 m of physical gap) and flew into the G4 wall. The intent — "do not
        reject a trajectory that takes it OUT" — is now written literally:
          * the bar never drops below POST_CHECK_FLOOR_CENTER (contact line);
          * inside the relaxed band the trajectory must actually LEAVE, i.e.
            end farther from obstacles than the drone is now, so a plan is
            admitted for escaping the soft zone and not for lingering in it."""
        cur_clear = float(self._esdf.get_edt_dis(
            self._drone_state.global_pos[:2])) + BODY_RADIUS_M
        relaxed = cur_clear - 0.05 < self._hard_clearance
        # "Do not get worse than where we already are" ...
        thr = min(self._hard_clearance, cur_clear - 0.05)
        # ... but never below the contact floor — UNLESS the drone is already
        # inside it, in which case the floor would reject every plan including
        # the outward one (a trajectory starts at the drone's own position), and
        # a drone that is too close would be unable to plan its way out at all.
        # That is the deadlock the adaptive rule was written to prevent, so the
        # floor follows the drone down instead of pinning the bar above it.
        thr = max(thr, min(POST_CHECK_FLOOR_CENTER, cur_clear))
        clears = [float(self._esdf.get_edt_dis(p[:2])) + BODY_RADIUS_M
                  for p in self._planner.get_pos_array()]
        for d in clears:
            if d < thr:
                self._n_graze_reject += 1
                raise ValueError(
                    f"Trajectory grazing: clearance(center)={d:.2f}m "
                    f"< hard {thr:.2f}m")
        if relaxed and clears:
            # Already inside the soft zone: only an OUTWARD plan is acceptable.
            tail_clear = min(clears[-max(1, len(clears) // 10):])
            if tail_clear < cur_clear + 0.05:
                self._n_graze_reject += 1
                raise ValueError(
                    f"Trajectory does not leave the soft zone: end clearance "
                    f"{tail_clear:.2f}m <= current {cur_clear:.2f}m")

    def _replan(self):
        if not self._esdf.is_ready():
            self.get_logger().warn("[INF] ESDF not ready yet",
                                   throttle_duration_sec=2.0)
            return
        if self._escape_active:
            self._maybe_exit_escape()
            if self._escape_active:
                self._publish_escape_marker()
                return
        if self._use_guards and self._check_no_progress():
            self.get_logger().warn("[INF] No progress — escape")
            self._enter_escape()
            return

        with self._depth_lock:
            if self._latest_depth is None:
                return
            lag = time.time() - self._latest_depth_time
            if lag > self._depth_max_lag:
                self.get_logger().warn(
                    f"[INF] Skip replan, depth lag {lag*1000:.0f}ms",
                    throttle_duration_sec=2.0)
                return
            depth_snapshot = self._latest_depth.copy()
            sync_state = DroneState()
            sync_state.copy_from(self._latest_depth_state)

        if np.linalg.norm(
                sync_state.global_pos[:2] - self._global_target) < 0.8:
            self._reached_target = True
            return

        # Max 2 attempts: normal target then short target. For each attempt,
        # warm_start_plan already retries 5x internally with perturbation.
        #
        # ADAPTIVE safe_dis (anti-deadlock): if the drone is already closer than
        # safe_dis to an obstacle, the (fixed) trajectory start point must violate
        # it -> the collision cost at the start alone already > tol -> all plans
        # fail and the drone hovers in place. Lower the reference to follow the
        # current clearance (-margin), floored at the hard barrier.
        # 2026-07-24: all CENTER-OF-MASS metres, plus the expert's speed-aware
        # margin (K_SPEED_MARGIN) so the reference path backs off from obstacles
        # as speed rises. Converted to the body-edge frame before MINCO sees it.
        cur_clear = (float(self._esdf.get_edt_dis(sync_state.global_pos[:2]))
                     + BODY_RADIUS_M)
        speed_now = float(np.linalg.norm(sync_state.global_vel[:2]))
        safe_dis_base = float(np.clip(
            cur_clear - 0.05, HARD_DIS_CENTER, self._safe_dis))
        safe_dis_eff  = safe_dis_base + self._speed_margin_k * speed_now  # center
        self._planner.safe_dis = max(0.0, safe_dis_eff - BODY_RADIUS_M)  # -> edge
        ok = False
        for shorter in (False, True):
            self._set_local_target(shorter=shorter,
                                   ref_pos=sync_state.global_pos[:2])
            head = self._get_head_state()
            tail = np.zeros((3, 2))
            tail[0], tail[1] = self._target_state[0], self._target_state[1]
            t0 = time.time()
            try:
                guesses = self._initial_guesses(
                    depth_snapshot, sync_state, head, tail)
            except Exception:
                continue
            self._publish_candidate_markers(guesses, head, self._target_state[0])
            for cand_i, (int_wpts, ts) in enumerate(guesses):
                try:
                    self._planner.warm_start_plan(
                        self._esdf, head, tail, int_wpts, ts)
                    self._check_plan_quality()
                    self._validate_planned_traj()
                except Exception:
                    continue
                self._replan_count += 1
                avg_inf = (np.mean(self._inference_times)
                           if self._inference_times else 0)
                cand_str = (f" cand#{cand_i + 1}/{len(guesses)}"
                            if len(guesses) > 1 else "")
                self.get_logger().info(
                    f"[INF] Replan ok #{self._replan_count} "
                    f"cost={self._planner.final_cost:.1f} "
                    f"t={(time.time()-t0)*1000:.0f}ms (inf={avg_inf:.1f}ms)"
                    f"{cand_str}{' short' if shorter else ''}",
                    throttle_duration_sec=1.0)
                self._publish_flown_marker()
                ok = True
                break
            if ok:
                break

        if not ok:
            self._replan_fail_count += 1
            self._map_stall_fails   += 1
            self.get_logger().warn(
                f"[INF] Replan failed ({self._replan_fail_count}x)")
            # Runs in BOTH modes and is checked BEFORE escape, because
            # _enter_escape() zeroes _replan_fail_count (see _map_stall_fails).
            if self._map_stall_fails >= STUCK_MAP_RESET_FAILS:
                # The blocker may be a stale phantom voxel that the live view
                # can never clear — wipe the map and let it rebuild.
                self._reset_stale_map()
            elif self._use_guards and self._replan_fail_count >= 2:
                self._enter_escape()
            return
        self._replan_fail_count = 0
        self._map_stall_fails   = 0

        state_cmd = self._planner.get_full_state_cmd(hz=self._cmd_hz)
        self._install_trajectory(state_cmd, float(np.sum(self._planner.ts)))

    def _install_trajectory(self, state_cmd, base_total_time):
        peak = 0.0
        try:
            speeds = np.linalg.norm(state_cmd[:, 1, :], axis=1)
            peak = float(np.max(speeds)) if len(speeds) else 0.0
        except Exception:
            pass
        scale = peak / self._v_max if (self._v_max > 1e-3
                                       and peak > self._v_max) else 1.0
        with self._traj_lock:
            self._traj.t_start         = time.time()
            self._traj.state_cmd       = state_cmd
            self._traj.base_total_time = base_total_time
            self._traj.time_scale      = scale
            self._traj.total_time      = base_total_time * scale
            self._traj.cmd_hz          = self._cmd_hz

    # ── Status / util ────────────────────────────────────────────────────────

    def _print_status(self):
        pos = self._drone_state.global_pos
        speed = float(np.linalg.norm(self._drone_state.global_vel[:2]))
        esc = " [ESCAPE]" if self._escape_active else ""
        esdf_info = (f" | esdf={float(self._esdf.get_edt_dis(pos[:2])):.2f}"
                     if self._esdf.is_ready() else "")
        avg_inf = np.mean(self._inference_times) if self._inference_times else 0
        self.get_logger().info(
            f"[INF] {self._mission_state}{esc} | "
            f"pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.2f}){esdf_info} | "
            f"speed={speed:.2f} | "
            f"goal=({self._global_target[0]:.1f},{self._global_target[1]:.1f})"
            f" | inf={avg_inf:.1f}ms")

    def _call_srv(self, client, request, timeout=8.0):
        fut = client.call_async(request)
        t0  = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.05)
        return fut.result()

    def _set_px4_param_int(self, name, value):
        if not self._param_client.wait_for_service(timeout_sec=5.0):
            return False
        req = ParamSetV2.Request()
        req.param_id = name
        pv = RclParameterValue()
        pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, int(value)
        req.value = pv
        res = self._call_srv(self._param_client, req)
        return bool(res and res.success)

    def _set_px4_param_float(self, name, value):
        if not self._param_client.wait_for_service(timeout_sec=5.0):
            return False
        req = ParamSetV2.Request()
        req.param_id = name
        pv = RclParameterValue()
        pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
        req.value = pv
        res = self._call_srv(self._param_client, req)
        ok = bool(res and res.success)
        self.get_logger().info(
            f"[INF] {'ok' if ok else 'FAILED'} PX4 param {name} = {value}")
        return ok

    def _configure_octomap_band(self, cruise_z):
        """Occupancy band follows the actual cruise_z (important collision fix)."""
        occ_min = max(0.35, cruise_z - 0.7)
        occ_max = cruise_z + 1.0
        if not self._octo_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"[INF] octomap set_parameters absent — set manually: "
                f"occ_min_z~{occ_min:.2f} occ_max_z~{occ_max:.2f}")
            return
        req = SetParameters.Request()
        for name, val in (("occupancy_min_z", occ_min),
                          ("occupancy_max_z", occ_max)):
            p = RclParameter()
            p.name = name
            pv = RclParameterValue()
            pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(val)
            p.value = pv
            req.parameters.append(p)
        res = self._call_srv(self._octo_param_client, req)
        ok  = bool(res and all(r.successful for r in res.results))
        self.get_logger().info(
            f"[INF] {'ok' if ok else 'FAILED'} Octomap band "
            f"[{occ_min:.2f}, {occ_max:.2f}]")
        if ok and self._octo_reset_client.wait_for_service(timeout_sec=3.0):
            self._call_srv(self._octo_reset_client, EmptySrv.Request(),
                           timeout=5.0)

    def _set_mode(self, mode):
        if not self._mode_client.wait_for_service(timeout_sec=3.0):
            return False
        req = SetMode.Request(); req.custom_mode = mode
        res = self._call_srv(self._mode_client, req)
        return bool(res and res.mode_sent)

    def _arm(self, value):
        if not self._arming_client.wait_for_service(timeout_sec=3.0):
            return False
        req = CommandBool.Request(); req.value = value
        res = self._call_srv(self._arming_client, req)
        return bool(res and res.success)

    def _wait_altitude(self, target_z, tol=0.2, timeout=30.0):
        t0, stable = time.time(), None
        while time.time() - t0 < timeout:
            z  = self._drone_state.global_pos[2]
            vz = abs(self._drone_state.global_vel[2])
            if abs(z - target_z) < tol and vz < 0.2:
                stable = stable or time.time()
                if time.time() - stable >= 2.0:
                    return True
            else:
                stable = None
            time.sleep(0.1)
        return False

    def _wait_ekf_stable(self, tol: float = 0.08, stable_dur: float = 3.0,
                         timeout: float = 45.0) -> float:
        """Wait for EKF Z to converge before takeoff (std < tol for stable_dur).
        Identical to expert_planner — required so ground_z is accurate."""
        self.get_logger().info(
            f"[INF] Waiting for EKF Z to stabilize (tol={tol}m, dur={stable_dur}s)...")
        t0 = time.time()
        z_history: list = []
        stable_start = None
        while time.time() - t0 < timeout:
            z_now = self._drone_state.global_pos[2]
            z_history.append(z_now)
            if len(z_history) > 50:
                z_history.pop(0)
            if len(z_history) >= 10:
                z_std = float(np.std(z_history[-10:]))
                if z_std < tol:
                    if stable_start is None:
                        stable_start = time.time()
                    elif time.time() - stable_start >= stable_dur:
                        z_ground = float(np.mean(z_history[-10:]))
                        self.get_logger().info(
                            f"[INF] ✓ EKF stable. ground_z={z_ground:.3f}m "
                            f"(std={z_std:.4f})")
                        return z_ground
                else:
                    stable_start = None
            elapsed = time.time() - t0
            if int(elapsed) % 5 == 0 and elapsed % 1.0 < 0.15:
                z_std_log = float(np.std(z_history[-10:])) if len(z_history) >= 10 else 999.0
                self.get_logger().info(
                    f"[INF] EKF Z={z_now:.3f}m std={z_std_log:.4f} "
                    f"({elapsed:.0f}s/{timeout:.0f}s)...")
            time.sleep(0.1)
        z_ground = float(np.mean(z_history[-10:])) if z_history else 0.0
        self.get_logger().warn(
            f"[INF] EKF stable timeout — using ground_z={z_ground:.3f}m")
        return z_ground

    def _warm_up(self):
        """Run the model a few times on a dummy input so the first REAL replan
        is not slowed by lazy CUDA/ONNX initialization. Subclasses override this
        with their own forward pass; the base only waits out the setpoint
        warm-up so an unimplemented subclass still arms safely."""
        time.sleep(50 / self._cmd_hz + 0.5)

    # ── Mission sequence ─────────────────────────────────────────────────────

    def run_sequence(self):
        self.get_logger().info("[INF] Waiting for MAVROS...")
        t0 = time.time()
        while not self._connected:
            if time.time() - t0 > self._conn_timeout:
                rclpy.shutdown(); return
            time.sleep(0.5)

        self.get_logger().info("[INF] Waiting for depth + ESDF...")
        t0 = time.time()
        while self._latest_depth is None or not self._esdf.is_ready():
            if time.time() - t0 > 60.0:
                self.get_logger().error("[INF] Depth/ESDF timeout!")
                rclpy.shutdown(); return
            time.sleep(0.5)

        # Force the EKF to use GPS as the height reference (not the drifting SITL
        # barometer). Wait 2s so PX4 processes the parameter before calibration.
        self._set_px4_param_int("EKF2_HGT_REF", 1)

        # Cap the PX4 horizontal speed (same as expert _configure_px4_height_ref):
        # v_max only lives in the planner — without the MPC caps PX4 chases a
        # position error at up to ~12 m/s (escape/resume near obstacles) and a
        # Gazebo contact impulse then triggers a PHYSICS EXPLOSION.
        # px4_vel_cap == 0 -> write PX4's STOCK defaults instead (neo / legacy
        # parity: that node never wrote MPC_XY_* at all). MEASURED 2026-07-27:
        # with no cap, a contact impulse on the return leg threw the drone at
        # 13.7 -> 29.2 m/s (physics explosion) — which is exactly why the cap
        # was added. Prefer a finite value well above cruise over 0.
        if self._px4_vel_cap == 0.0:
            self._set_px4_param_float("MPC_XY_VEL_ALL", -10.0)   # <0 = disabled
            self._set_px4_param_float("MPC_XY_VEL_MAX", 12.0)
            self._set_px4_param_float("MPC_XY_CRUISE",   5.0)
            self.get_logger().warn(
                "[INF] PX4 speed cap DISABLED (stock 12/5 m/s) — "
                "px4_vel_cap:=0. Physics explosions are possible on a "
                "tracking gap; this is the legacy/neo_planner behaviour.")
        else:
            vmax_phys = (self._px4_vel_cap if self._px4_vel_cap > 0.0
                         else self._v_max * 1.5)
            self._set_px4_param_float("MPC_XY_VEL_MAX", vmax_phys)
            self._set_px4_param_float("MPC_XY_CRUISE",  self._v_max)
            self._set_px4_param_float("MPC_XY_VEL_ALL", vmax_phys)
        time.sleep(2.0)

        # Wait for EKF Z to converge then record ground_z as the height reference.
        # Without this, cruise_z = target_alt assumes ground=0.0m, but the EKF can
        # have an offset of a few decimeters → the physical drone flies lower.
        ground_z = self._wait_ekf_stable()
        if abs(ground_z) > 2.0:
            self.get_logger().warn(
                f"[INF] ground_z unreasonable ({ground_z:.2f}m) — using 0.0m")
            ground_z = 0.0

        self.get_logger().info(
            f"[INF] ground_z={ground_z:.3f}m → cruise target="
            f"{ground_z + self._alt:.3f}m (physical ≈{self._alt:.1f}m)")

        # Set cruise_z BEFORE warm-up so the pre-ARM setpoint is already correct.
        # The ground_z offset ensures physical = target_alt even if the EKF frame is non-zero.
        self._cruise_z = ground_z + self._alt

        self._configure_octomap_band(self._alt)
        self._warm_up()

        self.get_logger().info("[INF] ARM...")
        t0, armed = time.time(), False
        while time.time() - t0 < self._arm_timeout:
            if self._arm(True):
                time.sleep(0.8)
                if self._armed:
                    armed = True; break
            time.sleep(2.0)
        if not armed:
            rclpy.shutdown(); return

        if not self._set_mode("OFFBOARD"):
            self._arm(False); rclpy.shutdown(); return
        t0 = time.time()
        while self._mode != "OFFBOARD" and time.time() - t0 < 5.0:
            time.sleep(0.1)

        self._mission_state = self.STATE_TAKEOFF
        takeoff_target      = self._cruise_z
        self.get_logger().info(f"[INF] Takeoff z={takeoff_target:.2f}m (physical ≈{self._alt:.1f}m)...")
        if not self._wait_altitude(takeoff_target):
            self.get_logger().warn("[INF] Takeoff not stable — continuing")
        time.sleep(1.0)

        # 2026-07-26: RESET the octomap AGAIN now, at the takeoff->FLYING
        # transition. The first reset happens pre-ARM (_configure_octomap_band),
        # but the drone then integrates depth all through the CLIMB — at low
        # altitude and while tilting, near-field/ground artifacts (plus the
        # startup TF-cache drops) burn spurious occupied cells into the map right
        # around the origin. Those made the drone read esdf=0.00 at its own start
        # cell -> every replan born in collision -> freeze (mode-fair) or false
        # collision-guard -> bad escape (full guards). Wiping the map here, once
        # the drone is stable at cruise_z with TF flowing, lets a CLEAN map
        # rebuild from the correct viewpoint before the first real plan.
        if self._octo_reset_client.wait_for_service(timeout_sec=2.0):
            self._call_srv(self._octo_reset_client, EmptySrv.Request(), timeout=5.0)
            self.get_logger().info("[INF] Octomap reset at FLYING (clean-map start)")
            time.sleep(1.5)   # let the camera repopulate a clean map before planning

        self._mission_state = self.STATE_FLYING
        self._reset_progress()
        self.get_logger().info(
            f"[INF] Start -> goal=({self._global_target[0]:.1f},"
            f"{self._global_target[1]:.1f})")

        last_replan, direction = 0.0, 1
        while rclpy.ok():
            now = time.time()
            if self._abort_reason is not None:
                self.get_logger().error(
                    f"[INF] MISSION ABORTED: {self._abort_reason}")
                break
            # Replan on the period, OR right away when a guard just stopped
            # the drone (see REPLAN_ASAP_MIN_GAP_S) so recovery does not stand
            # still waiting out the clock.
            due = now - last_replan >= self._replan_period
            asap = (self._replan_asap
                    and now - last_replan >= REPLAN_ASAP_MIN_GAP_S)
            if due or asap:
                last_replan = now
                self._replan_asap = False
                self._replan()
            dist = np.linalg.norm(
                self._drone_state.global_pos[:2] - self._global_target)
            # Stall watchdog (2026-07-27). Distinct from _check_no_progress,
            # which belongs to the escape scaffolding and is off in fair mode:
            # this one never manoeuvres, it just ends a run that has stopped
            # being measurable. Any real progress toward the goal rearms it.
            if self._stuck_abort_s > 0.0 and self._mission_state == self.STATE_FLYING:
                if (self._stall_ref_dist is None
                        or dist < self._stall_ref_dist - NO_PROGRESS_DELTA):
                    self._stall_ref_dist = dist
                    self._stall_ref_time = now
                elif (now - self._stall_ref_time > self._stuck_abort_s
                        and self._abort_reason is None):
                    p = self._drone_state.global_pos
                    self._abort_reason = (
                        f"STUCK at ({p[0]:.1f},{p[1]:.1f}) — no progress "
                        f"toward the goal for {self._stuck_abort_s:.0f}s "
                        f"(still {dist:.1f} m away)")
            if not self._escape_active and (dist < 1.5 or self._reached_target):
                if not self._auto_reverse:
                    self.get_logger().info("[INF] ok Goal reached!")
                    break
                self._reached_target = False
                direction = -direction
                self._global_target = (np.array([0.0, 0.0]) if direction < 0
                                       else np.array([self._goal_x, 0.0]))
                self._reset_progress()
                with self._traj_lock:
                    self._traj.invalidate()
                self.get_logger().info(
                    f"[INF] New goal: ({self._global_target[0]:.1f},"
                    f"{self._global_target[1]:.1f})")
            time.sleep(0.05)

        if not rclpy.ok():
            return

        self._mission_state = self.STATE_LANDING
        self._cruise_z      = None
        self.get_logger().info("[INF] Landing...")
        land_floor = ground_z + 0.25
        curr_z = self._drone_state.global_pos[2]
        while curr_z > land_floor and rclpy.ok():
            curr_z = max(ground_z + 0.05, curr_z - 0.08)
            sp = PositionTarget()
            sp.header.stamp     = self.get_clock().now().to_msg()
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            sp.type_mask = (
                PositionTarget.IGNORE_VX  | PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE)
            pos = self._drone_state.global_pos
            sp.position.x = float(pos[0]); sp.position.y = float(pos[1])
            sp.position.z = float(curr_z)
            self._pub_sp.publish(sp)
            time.sleep(0.1)

        time.sleep(1.5)
        try:
            self._arm(False)
            self._set_mode("MANUAL")
        except Exception:
            pass
        self._mission_state = self.STATE_DONE
        # Safety-layer accounting (2026-07-27) — how often each veto actually
        # fired. A run with many cost rejections points at a diverging solver,
        # many graze rejections at a model aiming through obstacles, and a low
        # v_limited_min at a drone that kept closing on walls.
        self.get_logger().info(
            f"[INF] Safety vetoes: cost-gate {self._n_cost_reject}, "
            f"post-check {self._n_graze_reject}, tightest speed cap "
            + (f"{self._v_limited_min:.2f} m/s" if self._v_limited_min
               is not None else "none"))
        if self._abort_reason is not None:
            self.get_logger().error(f"[INF] DONE (ABORTED: {self._abort_reason})")
        else:
            self.get_logger().info("[INF] DONE ok")
        if rclpy.ok():
            rclpy.shutdown()
