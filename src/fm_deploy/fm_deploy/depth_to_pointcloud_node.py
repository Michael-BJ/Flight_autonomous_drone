#!/usr/bin/env python3
"""
depth_to_pointcloud_node.py  (v2 — FIX TF time sync)

Converts a depth image + camera_info into a PointCloud2 to be consumed by
octomap_server. Does not need depth_image_proc — the conversion is done
manually with NumPy.

v2 FIX (FIX "Message Filter dropping message ... earlier than transform cache"):
  Previously the cloud header was copied from the depth image (carrying Gazebo
  SIM-TIME), while the TF odom->base_link from MAVROS uses WALL-CLOCK.
  As a result octomap tf2 could not transform -> ALL clouds dropped ->
  /projected_map never published -> ESDF not ready -> drone stuck.

  FIX: stamp the cloud with THIS NODE's clock (self.get_clock().now()), instead
  of copying the sim-time from Gazebo. This way the cloud uses the SAME time base
  as the MAVROS TF. Robust for two cases:
    - Without use_sim_time (default): now() = wall-clock = same as MAVROS TF
    - With use_sim_time on all nodes: now() = sim-time = consistent too
  frame_id is preserved (camera_depth_frame).

v3 (2026-07-28): ALTITUDE GATE. The back-projection + octomap insertion now only
  runs while the drone is inside an altitude band around the cruise altitude
  (see the "ALTITUDE GATE" section below). The depth image itself
  (/realsense/depth/float32) is untouched, so RViz still shows depth at every
  altitude — only the cloud that feeds octomap is gated.

────────────────────────────────────────────────────────────────────────────
ALTITUDE GATE
────────────────────────────────────────────────────────────────────────────
Why: obstacles only matter at cruise height. Below the band the camera mostly
sees the floor (which octomap would burn in as an obstacle wall right in front
of the drone), and back-projecting ~76k pixels at 9 Hz costs Jetson CPU that
buys nothing while the drone is climbing or descending.

Band:  [target_alt - margin, target_alt + margin]  above the GROUND, where
       ground is latched the same way fm_inference_real_node.py latches
       `ground_z`: collect poses for `gate_ground_settle_s` (default 10 s,
       matching that node's `ekf_pre_wait_s`), take the MEDIAN, and fall back
       to 0.0 if |z| > 3 m. Do NOT latch the first pose instead — measured
       2026-07-28 on the bench, the first pose read -2.51 m while the settled
       value the planner latched was -8.18 m. The two nodes would then disagree
       about where the ground is by metres and the band would sit at an
       altitude the drone never reaches.

ON THE GROUND THE GATE IS OPEN. This is deliberate and load-bearing:
fm_inference_real_node.py waits for depth + ESDF BEFORE it arms (step 4,
90 s timeout). A gate that closed below the band would make that wait fail
every time and the drone would never take off. So the gate stays open until
the drone has climbed into the band for the first time (`_armed_gate`); only
after that does it enforce the band. The stale ground voxels collected during
that window are discarded anyway — fm_inference_real_node resets octomap
after takeoff settles (step 13).

Set gate_enabled:=false to restore the v2 behaviour (always process).

v4 (2026-10-06, VOXCONF): VOXEL CONFIRMATION. Optional — confirm_hits >= 2
  switches it on; the default 1 leaves the output of v3 untouched, point for
  point. See the section below.

────────────────────────────────────────────────────────────────────────────
VOXEL CONFIRMATION  (NEW 2026-10-06, VOXCONF)
────────────────────────────────────────────────────────────────────────────
Why: octomap marks a voxel occupied on its FIRST hit (log-odds +0.85 > 0),
this node drops everything beyond max_range (so no ray ever clears the far
side of the map), and keep_octomap:=true disables the map wipes. A depth blob
that shows up in ONE frame therefore stays in the map for the rest of the
flight. Measured on the bags of 2026-10-06 (five flights, very light rain):
61-71 % of the occupied voxels in the planner band had been hit exactly once,
most of them born 3.3-4 m ahead of the nose, and together they closed the way
to the goal. They form connected clouds, so octomap_server's filter_speckles
does nothing against them.

What: a point goes to octomap only if the voxel it falls into — in the FIXED
frame `confirm_frame`, on octomap's own grid floor(p / resolution) — was hit
in at least `confirm_hits` different depth frames within `confirm_window_s`
seconds (the current frame counts). A voxel that passed once stays accepted
while it keeps being seen (`confirm_keep_s`), so a real obstacle is delayed
only the first time it shows up.

Replayed with this very class on the hit histories of six recorded flights
(10/02 + five of 10/06; one frame per cloud octomap inserted, ~6 Hz):

    rule            single-hit voxels gone   saturated (= real) voxels kept
    2 hits / 1.0 s          96 %                     98.4 %
    2 hits / 2.0 s          96 %                     99.6 %      <- default
    2 hits / 3.0 s          96 %                     99.8 %
    3 hits / 1.0 s          97 %                     84.5 %      <- do not

The window hardly matters for the noise (a voxel hit once in a whole flight
fails any window) but a short one loses real voxels that are seen only about
once a second, and 3 hits loses a lot of them. With 2 hits / 2.0 s a route to
the goal existed at 37 of 37 sampled moments (raw map: 29 of 37).

What it costs:
  1. a real obstacle enters the map later: one depth frame (~0.1 s) when it is
     seen steadily; on those flights (patchy depth, 6 Hz) the median was
     0.3-1.1 s per voxel, and the drone was at most 0.18 m closer to the
     obstacle by the time its first voxels were in the map;
  2. something the camera catches only now and then — a thin branch, a wire
     at the edge of range — is NOT inserted until it is seen `confirm_hits`
     times within the window;
  3. the rays of the held-back points do not clear free space either (fewer
     cells known-free; matters to the "unobserved" guards if they are on).
The FM model is not affected: it reads /realsense/depth/float32, which this
node never touches.

FAIL-OPEN. Whenever the filter cannot do its job — no TF `confirm_frame` <-
cloud frame, TF older than `confirm_tf_max_age_s`, any exception, or a big
cloud being rejected almost completely for seconds (= the transform is not
stable, see _BYPASS_* below) — the whole frame is published unfiltered,
exactly like v3, and a warning is logged ("bypassed"). The same goes, without
a warning, for a frame that has nothing to be compared with: the first one
after start-up or after a pause longer than the window ("blind"). The filter
can only ever REMOVE points from a frame; it cannot stop the cloud.

The transform used is the LATEST `confirm_frame` <- camera TF, not the one at
the cloud stamp (that one does not exist yet when the image arrives; octomap
waits for it). Measured on the same bags the two differ by 1 cm (median) /
2-3 cm (p95) for a point 4 m away, against a 15 cm voxel. The TF listener
runs on a node and a thread of its own (see _setup_confirm for why).

With the filter on, the cloud message is also filled through the fast path
(see _to_pointcloud2): 7 / 13 ms per frame for 13k / 53k points instead of
30 / 107 ms. Same bytes on the wire. Off = v3, timing included.

Input:
  /realsense/depth/float32      sensor_msgs/Image      (32FC1, meters)
  /realsense/depth/camera_info  sensor_msgs/CameraInfo
  /mavros/local_position/pose   geometry_msgs/PoseStamped   (altitude gate)
  /tf, /tf_static               (VOXCONF — only subscribed when confirm_hits >= 2)

Output:
  /realsense/depth/points       sensor_msgs/PointCloud2
  /realsense/depth/gate_status  std_msgs/String   (JSON: open, alt, band)
  /realsense/depth/points_rejected  sensor_msgs/PointCloud2   (VOXCONF: the
                                points held back; only while someone listens)
  /realsense/depth/confirm_status   std_msgs/String   (VOXCONF: JSON counters, 1 Hz)
"""

import array
import json
import time

import numpy as np
import rclpy
import rclpy.time
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String

# MAVROS publishes local_position/pose BEST_EFFORT — a RELIABLE subscription
# here would silently never match and the gate would never see an altitude.
_MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


# ══════════════════════════════════════════════════════════════════════════
# NEW (2026-10-06, VOXCONF): voxel confirmation — see the module docstring
# ══════════════════════════════════════════════════════════════════════════

# Safety net of the filter. A static scene keeps its voxels whatever the drone
# does, so a BIG cloud that is held back almost completely for seconds can
# only mean the transform is not stable (jumping pose, wrong frame). Then the
# filter steps aside and the frames go out unfiltered until the confirmations
# come back. Frames that hold nothing but noise are far too small to trip it.
_BYPASS_MIN_POINTS = 5000      # only clouds at least this big are judged
_BYPASS_MAX_PASS   = 0.10      # "almost completely" = under 10 % let through
_BYPASS_AFTER_S    = 2.0       # ... for this long


def _quat_to_matrix(x, y, z, w):
    """Rotation matrix of a quaternion (need not be perfectly normalised)."""
    n = x * x + y * y + z * z + w * w
    if not n > 1e-12:
        raise ValueError("TF rotation is not a valid quaternion")
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w),       s * (x * z + y * w)],
        [s * (x * y + z * w),       1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),       s * (y * z + x * w),       1.0 - s * (x * x + y * y)],
    ])


class VoxelConfirmFilter:
    """Lets a point through only once its voxel has been hit often enough.

    Pure NumPy, no ROS: give it the points of one depth frame, already in a
    FIXED frame, plus the time of that frame, and it answers with a mask.

        hits      frames that must have hit a voxel, the current one included
        window_s  ... within this many seconds
        keep_s    a voxel that passed stays accepted until it has not been
                  seen for this long; after that it has to earn it again

    The grid is octomap's: voxel index = floor(coordinate / voxel).

    A frame that COULD not be confirmed — fewer than `hits` frames arrived
    inside the window: start-up, a pause, frames coming in slower than the
    window — is let through completely (`blind`). The filter must never be
    the reason why nothing reaches the map. Call update() for EVERY frame,
    also one without points: an empty frame still proves the camera was
    looking, so the blob in the next one is "unconfirmed", not "blind".
    """

    _BITS = 21                     # per axis in the packed 63-bit key
    _OFF  = 1 << (_BITS - 1)       # shifts the indices to >= 0
    _LIM  = _OFF - 1               # |index| limit: +/-157 km at 0.15 m

    def __init__(self, voxel=0.15, hits=2, window_s=2.0, keep_s=60.0,
                 max_voxels=500000):
        self.voxel      = float(voxel)
        self.hits       = max(1, int(hits))
        self.window_s   = float(window_s)
        self.keep_s     = max(float(keep_s), self.window_s)
        self.max_voxels = int(max_voxels)
        self._inv       = 1.0 / self.voxel
        self.reset()

    def reset(self):
        # One entry per frame inside the window: (time, sorted keys of the
        # voxels that frame hit and that were NOT accepted yet).
        self._pending   = []
        # Accepted voxels: sorted keys + the last time each one was seen.
        self._conf_keys = np.zeros(0, dtype=np.int64)
        self._conf_t    = np.zeros(0, dtype=np.float64)
        self._t_last    = None
        # About the last update(), for logging.
        self.blind       = True
        self.n_voxels    = 0       # distinct voxels in the frame
        self.n_held      = 0       # ... of which were held back
        self.n_confirmed = 0       # size of the accepted set

    def update(self, pts, t):
        """pts: (N, 3) metres in the fixed frame. t: seconds, one clock for
        every call. Returns a bool mask (N,): True = let the point through.
        Raises ValueError on points that are not finite / absurdly far."""
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        n = pts.shape[0]
        t = float(t)

        if self._t_last is not None and t < self._t_last:
            # The clock went backwards: the stored times mean nothing any
            # more. Count from scratch, keep what was accepted.
            self._pending = []
            np.minimum(self._conf_t, t, out=self._conf_t)
        self._t_last = t

        # Forget what is too old.
        self._pending = [p for p in self._pending if t - p[0] <= self.window_s]
        if len(self._conf_keys):
            alive = (t - self._conf_t) <= self.keep_s
            if not alive.all():
                self._conf_keys = self._conf_keys[alive]
                self._conf_t    = self._conf_t[alive]

        self.blind = len(self._pending) + 1 < self.hits

        if n == 0:
            self._pending.append((t, np.zeros(0, dtype=np.int64)))
            self.n_voxels = self.n_held = 0
            self.n_confirmed = len(self._conf_keys)
            return np.zeros(0, dtype=bool)

        idx = np.floor(pts * self._inv)
        if not (np.abs(idx).max() <= self._LIM):          # also catches NaN / inf
            raise ValueError("point outside the voxel grid (NaN, inf or > 100 km)")
        idx = idx.astype(np.int64)
        keys = (((idx[:, 0] + self._OFF) << (2 * self._BITS))
                | ((idx[:, 1] + self._OFF) << self._BITS)
                | (idx[:, 2] + self._OFF))
        uniq, inv = np.unique(keys, return_inverse=True)

        # 1. Voxels accepted earlier pass at once.
        nc = len(self._conf_keys)
        if nc:
            pos   = np.searchsorted(self._conf_keys, uniq)
            known = self._conf_keys[np.minimum(pos, nc - 1)] == uniq
        else:
            pos   = np.zeros(len(uniq), dtype=np.intp)
            known = np.zeros(len(uniq), dtype=bool)

        # 2. The others need `hits` frames inside the window; this frame is one.
        cand = uniq[~known]
        cnt  = np.ones(len(cand), dtype=np.int32)
        for _, pk in self._pending:
            if len(pk) and len(cand):
                j = np.searchsorted(pk, cand)
                cnt += pk[np.minimum(j, len(pk) - 1)] == cand
        fresh = cnt >= self.hits                  # all False while blind

        # 3. Remember.
        if nc:
            self._conf_t[pos[known]] = t
        if fresh.any():
            at = pos[~known][fresh]               # where they go in the sorted set
            self._conf_keys = np.insert(self._conf_keys, at, cand[fresh])
            self._conf_t    = np.insert(self._conf_t, at, t)
        self._pending.append((t, cand[~fresh]))

        if len(self._conf_keys) > self.max_voxels:
            # Not reached in a normal flight (keep_s bounds the set). If it is,
            # drop the voxels that have not been seen for the longest time.
            order = np.argsort(self._conf_t, kind="stable")
            alive = np.zeros(len(order), dtype=bool)
            alive[order[len(order) - self.max_voxels // 2:]] = True
            self._conf_keys = self._conf_keys[alive]
            self._conf_t    = self._conf_t[alive]

        self.n_voxels    = len(uniq)
        self.n_confirmed = len(self._conf_keys)
        if self.blind:
            self.n_held = 0
            return np.ones(n, dtype=bool)
        ok = known
        ok[~known] = fresh
        self.n_held = int(len(ok) - np.count_nonzero(ok))
        return ok[inv]


class DepthToPointcloudNode(Node):

    def __init__(self):
        super().__init__("depth_to_pointcloud_node")

        # 2026-07-26: default 0.15 -> 0.5. Returns closer than 0.5 m are almost
        # always near-field noise / self-view and, at 0.15 m, burned SPURIOUS
        # occupied cells into the octomap right at the drone -> esdf=0.00 at its
        # own cell -> stuck / chaotic escape. The launches override this too.
        self.declare_parameter("min_range",   0.5)
        self.declare_parameter("max_range",   4.0)
        self.declare_parameter("skip_pixels", 2)    # downsample: 1 point per N pixels

        # ── Altitude gate (see the module docstring) ────────────────────────
        self.declare_parameter("gate_enabled",    True)
        # Centre of the band. Keep this equal to the planner's target_alt —
        # fm_real.launch.py passes the same LaunchConfiguration to both.
        self.declare_parameter("gate_target_alt", 1.2)
        # Half-width of the band, in metres. 0.5 -> 0.7..1.7 m for target 1.2.
        self.declare_parameter("gate_alt_margin", 0.5)
        self.declare_parameter("gate_pose_topic", "/mavros/local_position/pose")
        # EKF convergence window before ground is latched. Keep this equal
        # to ekf_pre_wait_s in fm_inference_real_node.py.
        self.declare_parameter("gate_ground_settle_s", 10.0)

        # ── NEW (2026-10-06, VOXCONF): voxel confirmation (module docstring) ─
        # dynamic_typing: `-p confirm_hits:=2.0` or `-p confirm_window_s:=1` on
        # a command line must not kill the node at start-up.
        _dyn = ParameterDescriptor(dynamic_typing=True)
        # 1 = OFF (output of v3). 2 = a voxel needs two frames. Launch arg
        # confirm_hits of fm_real_barometer / fm_all_barometer.
        self.declare_parameter("confirm_hits",         1,    _dyn)
        self.declare_parameter("confirm_window_s",     2.0,  _dyn)
        # MUST equal octomap's resolution — the launch passes the same value.
        self.declare_parameter("confirm_voxel",        0.15, _dyn)
        self.declare_parameter("confirm_keep_s",       60.0, _dyn)
        # MUST equal octomap's frame_id.
        self.declare_parameter("confirm_frame",        "odom")
        self.declare_parameter("confirm_tf_max_age_s", 0.5,  _dyn)
        # One [CONFIRM] summary line per this many seconds (0 = none).
        self.declare_parameter("confirm_log_period_s", 10.0, _dyn)

        self.min_range  = self.get_parameter("min_range").value
        self.max_range  = self.get_parameter("max_range").value
        self.skip       = max(1, int(self.get_parameter("skip_pixels").value))

        self.gate_on    = bool(self.get_parameter("gate_enabled").value)
        self.gate_alt   = float(self.get_parameter("gate_target_alt").value)
        self.gate_marg  = abs(float(self.get_parameter("gate_alt_margin").value))
        gate_topic      = str(self.get_parameter("gate_pose_topic").value)
        self.gate_settle = float(self.get_parameter("gate_ground_settle_s").value)
        self.gate_lo    = self.gate_alt - self.gate_marg
        self.gate_hi    = self.gate_alt + self.gate_marg

        self._ground_z   = None    # latched after gate_settle_s (median), see below
        self._z_samples  = []      # poses collected during the settle window
        self._t_first_z  = None
        self._rel_alt    = None    # altitude above that ground
        self._armed_gate = False   # False = still open (pre-takeoff), see docstring
        self._gate_open  = True    # current gate state, for logging/status

        self.bridge    = CvBridge()
        self.cam_info  = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_cloud = self.create_publisher(
            PointCloud2, "/realsense/depth/points", sensor_qos
        )

        # NEW (2026-10-06, VOXCONF)
        self._setup_confirm(sensor_qos)

        self.sub_info = self.create_subscription(
            CameraInfo, "/realsense/depth/camera_info", self._on_info, sensor_qos
        )
        self.sub_depth = self.create_subscription(
            Image, "/realsense/depth/float32", self._on_depth, sensor_qos
        )

        self.pub_gate = self.create_publisher(
            String, "/realsense/depth/gate_status", 10
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, gate_topic, self._on_pose, _MAVROS_QOS
        )

        self.get_logger().info(
            f"DepthToPointcloudNode v3 active | skip={self.skip} | "
            f"range=[{self.min_range}, {self.max_range}]m | "
            f"cloud stamped with node clock (FIX TF sync)"
        )
        if self.gate_on:
            self.get_logger().info(
                f"  altitude gate ON: band [{self.gate_lo:.2f}, {self.gate_hi:.2f}] m "
                f"above ground (target {self.gate_alt:.2f} +/- {self.gate_marg:.2f}) "
                f"| pose {gate_topic} | OPEN until first climb into band"
            )
        else:
            self.get_logger().warn("  altitude gate OFF — processing at every altitude")
        # NEW (2026-10-06, VOXCONF)
        if self._confirm_on:
            self.get_logger().info(
                f"  voxel confirmation ON: a point reaches octomap once its "
                f"{self._confirm_voxel:.2f} m voxel ({self._confirm_frame}) was hit in "
                f">= {self._confirm_hits} frames within {self._confirm_window:.1f} s "
                f"| accepted voxels kept {self._confirm_keep:.0f} s "
                f"| TF older than {self._confirm_tf_age:.2f} s -> frame goes out unfiltered"
            )
        else:
            self.get_logger().info(
                "  voxel confirmation OFF (confirm_hits < 2) — every point goes to octomap")

    def _on_info(self, msg: CameraInfo):
        self.cam_info = msg

    # ── NEW (2026-10-06, VOXCONF): voxel confirmation ───────────────────────

    def _num_param(self, name, default, lo, hi):
        """A numeric parameter that survives the wrong type and a silly value."""
        raw = self.get_parameter(name).value
        try:
            val = float(raw)
            if not np.isfinite(val):
                raise ValueError(raw)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"[CONFIRM] {name}={raw!r} is not a number -> using {default}")
            val = float(default)
        lim = min(float(hi), max(float(lo), val))
        if lim != val:
            self.get_logger().warn(
                f"[CONFIRM] {name}={val:g} is outside [{lo:g}, {hi:g}] -> using {lim:g}")
        return lim

    def _setup_confirm(self, sensor_qos):
        self._confirm_hits   = int(self._num_param("confirm_hits", 1, 1, 20))
        self._confirm_window = self._num_param("confirm_window_s", 2.0, 0.1, 10.0)
        self._confirm_voxel  = self._num_param("confirm_voxel", 0.15, 0.02, 1.0)
        self._confirm_keep   = self._num_param(
            "confirm_keep_s", 60.0, self._confirm_window, 3600.0)
        self._confirm_frame  = str(self.get_parameter("confirm_frame").value)
        self._confirm_tf_age = self._num_param("confirm_tf_max_age_s", 0.5, 0.05, 5.0)
        self._confirm_log_s  = self._num_param("confirm_log_period_s", 10.0, 0.0, 3600.0)
        self._confirm_on     = self._confirm_hits >= 2
        if not self._confirm_on:
            return                           # v3: no TF listener, no extra topics

        try:
            # Imported here so that the node with the filter off does not even
            # depend on tf2.
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer   = Buffer()
            # node=None + spin_thread: the listener gets a node and a thread of
            # its own. It must NOT share this node's executor: rclpy takes one
            # message per subscription per round, so whenever _on_depth is slow
            # the 30 Hz /tf queue falls behind (measured in the end-to-end
            # test: "latest" TF 2-3 s old while this node was saturated).
            self._tf_listener = TransformListener(self._tf_buffer, None, spin_thread=True)
        except Exception as e:               # no tf2 -> no fixed frame -> no filter
            self.get_logger().error(
                f"[CONFIRM] cannot listen to TF ({e}) -> voxel confirmation OFF")
            self._confirm_on = False
            return
        self._tf_latest = rclpy.time.Time()  # tf2: time 0 = the latest transform

        self._confirm = VoxelConfirmFilter(
            voxel=self._confirm_voxel, hits=self._confirm_hits,
            window_s=self._confirm_window, keep_s=self._confirm_keep)

        self._pub_rejected = self.create_publisher(
            PointCloud2, "/realsense/depth/points_rejected", sensor_qos)
        self._pub_confirm = self.create_publisher(
            String, "/realsense/depth/confirm_status", 10)

        # Counters since start (the status topic) ...
        self._ctot = {"frames": 0, "blind": 0, "bypassed": 0,
                      "pts_in": 0, "pts_out": 0, "ms": 0.0, "tf_ms": 0.0}
        # ... and what the periodic log line needs on top.
        self._clog_prev   = dict(self._ctot)
        self._clog_t      = None
        self._clog_ms_max = 0.0
        self._clog_tf_max = 0.0
        self._cstat_t     = None
        self._clast_ms    = 0.0
        self._clast_tf_ms = 0.0
        self._cstarve_t0  = None             # see _BYPASS_*

    def _confirm_filter(self, points, frame_id, now, stamp):
        """The points of this frame that octomap may see.

        Returns `points` unchanged whenever the filter cannot do its job
        (fail-open, module docstring). Never raises."""
        n     = len(points)
        tot   = self._ctot
        now_s = now.nanoseconds * 1e-9
        keep  = None                         # None = whole frame goes out
        blind = False
        t0    = time.perf_counter()
        try:
            if n == 0:
                # Nothing in range, but the frame DID arrive: tell the filter,
                # or the blob in the next frame would look like "first frame
                # after a pause" and be waved through.
                self._confirm.update(points, now_s)
            else:
                tf = self._tf_buffer.lookup_transform_core(
                    self._confirm_frame, frame_id, self._tf_latest)
                st   = tf.header.stamp
                tf_s = st.sec + st.nanosec * 1e-9
                age  = now_s - tf_s if tf_s > 0.0 else 0.0   # stamp 0 = static chain
                self._clast_tf_ms = age * 1e3
                if abs(age) > self._confirm_tf_age:
                    raise RuntimeError(
                        f"TF {self._confirm_frame} <- {frame_id} is {age:+.2f} s "
                        f"away from now (limit {self._confirm_tf_age:.2f} s)")
                q, tr = tf.transform.rotation, tf.transform.translation
                r = _quat_to_matrix(q.x, q.y, q.z, q.w)
                # Column by column, NOT `points @ r.T`: a matrix product goes to
                # OpenBLAS, whose worker threads then spin on every core between
                # frames (measured: 5 threads x 4.7 s of CPU in a 22 s run, more
                # than the whole node needs) — CPU the planner has to live on.
                px = points[:, 0].astype(np.float64)
                py = points[:, 1].astype(np.float64)
                pz = points[:, 2].astype(np.float64)
                fixed = np.empty((n, 3), dtype=np.float64)
                fixed[:, 0] = r[0, 0] * px + r[0, 1] * py + r[0, 2] * pz + tr.x
                fixed[:, 1] = r[1, 0] * px + r[1, 1] * py + r[1, 2] * pz + tr.y
                fixed[:, 2] = r[2, 0] * px + r[2, 1] * py + r[2, 2] * pz + tr.z
                keep  = self._confirm.update(fixed, now_s)
                blind = self._confirm.blind

                # Safety net, see _BYPASS_* at the top of the file.
                n_keep = int(np.count_nonzero(keep))
                if (not blind and n >= _BYPASS_MIN_POINTS
                        and n_keep < _BYPASS_MAX_PASS * n):
                    if self._cstarve_t0 is None or now_s < self._cstarve_t0:
                        self._cstarve_t0 = now_s
                    starved = now_s - self._cstarve_t0
                    if starved >= _BYPASS_AFTER_S:
                        raise RuntimeError(
                            f"only {n_keep} of {n} points confirmed for "
                            f"{starved:.1f} s — is the {self._confirm_frame} pose "
                            f"jumping?")
                else:
                    self._cstarve_t0 = None
        except Exception as e:
            keep = None
            if n:
                tot["bypassed"] += 1
                self.get_logger().warn(
                    f"[CONFIRM] filter bypassed, all {n} points of this frame go "
                    f"to octomap: {e}", throttle_duration_sec=5.0)
        ms = (time.perf_counter() - t0) * 1e3

        out = points if keep is None else points[keep]

        # Bookkeeping and side outputs — must never cost us the frame.
        try:
            tot["frames"] += 1
            tot["blind"]  += int(blind)
            tot["pts_in"]  += n
            tot["pts_out"] += len(out)
            tot["ms"]      += ms
            tot["tf_ms"]   += self._clast_tf_ms
            self._clast_ms    = ms
            self._clog_ms_max = max(self._clog_ms_max, ms)
            self._clog_tf_max = max(self._clog_tf_max, abs(self._clast_tf_ms))
            if self._pub_rejected.get_subscription_count() > 0:
                held = points[:0] if keep is None else points[~keep]
                self._pub_rejected.publish(self._to_pointcloud2(held, frame_id, stamp))
            self._confirm_report(now_s)
        except Exception as e:
            self.get_logger().warn(
                f"[CONFIRM] status/log failed: {e}", throttle_duration_sec=30.0)
        return out

    def _confirm_report(self, now_s):
        """1 Hz JSON on /realsense/depth/confirm_status + one log line per
        confirm_log_period_s."""
        tot = self._ctot
        if self._cstat_t is None or abs(now_s - self._cstat_t) >= 1.0:
            self._cstat_t = now_s
            self._pub_confirm.publish(String(data=json.dumps({
                "hits":      self._confirm_hits,
                "window_s":  round(self._confirm_window, 3),
                "voxel_m":   round(self._confirm_voxel, 3),
                "frames":    int(tot["frames"]),      # counters since start
                "blind":     int(tot["blind"]),       # unfiltered: too few frames in the window
                "bypassed":  int(tot["bypassed"]),    # unfiltered: TF / error / safety net
                "pts_in":    int(tot["pts_in"]),
                "pts_out":   int(tot["pts_out"]),
                "accepted_voxels": int(self._confirm.n_confirmed),
                "last_voxels":     int(self._confirm.n_voxels),   # in the last frame
                "last_held_voxels": int(self._confirm.n_held),    # ... of which held back
                "last_ms":   round(float(self._clast_ms), 2),
                "tf_age_ms": round(float(self._clast_tf_ms), 1),
            })))

        if self._confirm_log_s <= 0.0:
            return
        if self._clog_t is None:
            self._clog_t = now_s
            return
        dt = now_s - self._clog_t
        if abs(dt) < self._confirm_log_s:
            return
        d = {k: tot[k] - self._clog_prev[k] for k in tot}
        fr = max(1, d["frames"])
        held = d["pts_in"] - d["pts_out"]
        self.get_logger().info(
            f"[CONFIRM] {abs(dt):.1f} s: {d['frames']} frames "
            f"({d['frames'] / max(1e-3, abs(dt)):.1f} Hz) | "
            f"{d['pts_in']} points in -> {d['pts_out']} to octomap, {held} held back "
            f"({100.0 * held / max(1, d['pts_in']):.1f} %) | "
            f"accepted voxels {self._confirm.n_confirmed} | "
            f"filter {d['ms'] / fr:.1f} ms/frame (max {self._clog_ms_max:.1f}) | "
            f"TF age {d['tf_ms'] / fr:.0f} ms (max {self._clog_tf_max:.0f}) | "
            f"unfiltered frames: {d['bypassed']} bypassed, {d['blind']} after a pause")
        self._clog_prev   = dict(tot)
        self._clog_t      = now_s
        self._clog_ms_max = 0.0
        self._clog_tf_max = 0.0

    # ── altitude gate ───────────────────────────────────────────────────────

    def _on_pose(self, msg: PoseStamped):
        z   = float(msg.pose.position.z)
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._ground_z is None:
            # Settle window: the EKF z is still walking towards its final value
            # right after boot. Latching it too early puts the whole band at the
            # wrong height for the entire flight — see the module docstring.
            if self._t_first_z is None:
                self._t_first_z = now
            self._z_samples.append(z)
            if now - self._t_first_z < self.gate_settle:
                return                       # gate stays open meanwhile
            med = float(np.median(self._z_samples))
            self._ground_z = med if abs(med) <= 3.0 else 0.0
            if abs(med) > 3.0:
                self.get_logger().warn(
                    f"[GATE] ground_z looks unreasonable ({med:.2f} m) -> 0.0 "
                    "(same as fm_inference_real_node)")
            self.get_logger().info(
                f"[GATE] ground_z latched = {self._ground_z:.2f} m "
                f"(median of {len(self._z_samples)} poses / "
                f"{self.gate_settle:.0f}s)")
            self._z_samples = []

        self._rel_alt = z - self._ground_z

    def _gate_allows(self) -> bool:
        """True = process this frame into a cloud.

        Open while the drone has not yet reached the band (pre-flight / climb),
        because fm_inference_real_node blocks on depth+ESDF before it arms.
        """
        if not self.gate_on:
            return True
        if self._rel_alt is None:
            return True                      # no MAVROS pose yet -> do not block
        in_band = self.gate_lo <= self._rel_alt <= self.gate_hi
        if in_band:
            self._armed_gate = True          # latch: from now on the band applies
            return True
        return not self._armed_gate

    def _report_gate(self, is_open: bool):
        if is_open == self._gate_open:
            return
        self._gate_open = is_open
        alt = f"{self._rel_alt:.2f}" if self._rel_alt is not None else "n/a"
        self.get_logger().info(
            f"[GATE] {'OPEN' if is_open else 'CLOSED'} — alt={alt} m, "
            f"band [{self.gate_lo:.2f}, {self.gate_hi:.2f}] m"
        )
        self.pub_gate.publish(String(data=json.dumps({
            "open":     is_open,
            "alt_m":    round(self._rel_alt, 2) if self._rel_alt is not None else None,
            "band_m":   [round(self.gate_lo, 2), round(self.gate_hi, 2)],
            "latched":  self._armed_gate,
        })))

    def _on_depth(self, msg: Image):
        if self.cam_info is None:
            return

        allowed = self._gate_allows()
        self._report_gate(allowed)
        if not allowed:
            return

        # Stamp NOW, before the back-projection below. The stamp still has to be
        # this node's clock (see the v2 note in the module docstring — the Gazebo
        # image carries sim-time while the MAVROS TF is wall-clock), but taking
        # it at ARRIVAL instead of after ~76k points of numpy work removes that
        # processing time from the pose octomap transforms the cloud with. Any
        # residual latency still smears every point by v*L in world coordinates,
        # which is where stale phantom voxels come from — see STUCK_MAP_RESET_*
        # in fm_inference_base.py for the recovery path.
        now   = self.get_clock().now()       # VOXCONF: the filter wants it as a Time too
        stamp = now.to_msg()

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"imgmsg_to_cv2 failed: {e}", throttle_duration_sec=2.0)
            return

        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        h, w = depth.shape
        skip = self.skip

        v_idx, u_idx = np.mgrid[0:h:skip, 0:w:skip]
        z = depth[v_idx, u_idx]

        valid = (z >= self.min_range) & (z <= self.max_range) & np.isfinite(z)

        z_v = z[valid].astype(np.float32)
        u_v = u_idx[valid].astype(np.float32)
        v_v = v_idx[valid].astype(np.float32)

        # Back-project to 3D camera coordinates (optical: z=forward, x=right, y=down)
        x = (u_v - cx) / fx * z_v
        y = (v_v - cy) / fy * z_v

        points = np.column_stack([x, y, z_v]).astype(np.float32)

        # FIX: stamp with THIS NODE's clock, not msg.header (which is Gazebo sim-time)
        frame_id = msg.header.frame_id

        # NEW (2026-10-06, VOXCONF): hold back the points whose voxel has not
        # been seen often enough yet. Called for an EMPTY frame too — the filter
        # has to know the camera was looking. Off -> `points` is not touched.
        if self._confirm_on:
            points = self._confirm_filter(points, frame_id, now, stamp)

        # Skip empty clouds: octomap would shout "No data to copy" and there is no
        # point processing 0 points (e.g. when facing an open area).
        if len(points) == 0:
            return

        self.pub_cloud.publish(self._to_pointcloud2(points, frame_id, stamp))

    def _to_pointcloud2(self, points: np.ndarray, frame_id: str,
                        stamp) -> PointCloud2:
        msg = PointCloud2()
        # FIX TF SYNC: node clock, not the depth image's sim-time stamp. Captured
        # at image arrival by the caller (not here) — see _on_depth.
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.height     = 1
        msg.width      = len(points)
        msg.is_dense   = False
        msg.is_bigendian = False
        msg.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step   = 12 * len(points)
        raw = points.tobytes()
        # NEW (2026-10-06, VOXCONF): with the filter on, hand rclpy an
        # array.array instead of `bytes`. Given `bytes`, the generated setter
        # checks every single byte in Python: measured on this Jetson 29 ms for
        # 15k points, 85 ms for 44k and 149 ms for a full 76.8k-point frame —
        # more than the 100 ms between two frames, so the node (and with it the
        # filter's TF) could not keep up exactly when an obstacle fills the
        # picture. The array goes straight in (< 0.1 ms); the bytes on the wire
        # are the same. Left as it was when the filter is off, so that "off"
        # changes nothing at all, not even the timing.
        msg.data       = array.array("B", raw) if self._confirm_on else raw
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointcloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
