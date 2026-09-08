#!/usr/bin/env python3
"""
gps_check_node.py  (package: gps_guard)
==================================================================
PURPOSE: a standalone GPS pre-flight / stability checker.

WHY THIS PACKAGE EXISTS
    On 2026-08-27 the drone armed and took off with fix_type=0, 0
    satellites, HDOP 99.99 — and, on the ground, an EKF position that
    was jumping around by ~1.5 m while the airframe sat perfectly still.
    The controller "corrected" toward that phantom motion and the drone
    drifted sideways into a tree. See
    takeoff_land/INCIDENT_2026-08-27_gps_ekf.md.

    takeoff_land already has its own inline GPS gate. This node is a
    SEPARATE, INDEPENDENT opinion, kept in its own package so it can be
    reused (fm_deploy, future flight nodes) and managed on its own. It
    also adds one check the inline gate does not do: it watches whether
    the horizontal position estimate is actually STILL while the drone is
    on the ground. A GPS/EKF that reports a "3D fix" but whose x/y
    wanders half a metre is not stable — that is exactly the failure that
    put the drone in a tree.

RELATIONSHIP TO takeoff_land
    This node does NOT modify or gate takeoff_land. It only reads the
    telemetry takeoff_land's px4_sensor_reader already publishes and
    emits a verdict. To make a flight node refuse to ARM on a bad
    verdict, have it subscribe to /px4/gps_ready (Bool, latched) and
    block until it is True. Nothing here commands the vehicle.

WHAT IT DOES
    Subscribes (JSON from px4_sensor_reader, never MAVROS directly):
        /px4/state    — armed flag
        /px4/sensors  — gps_quality block + local_x/y/z

    Two independent things must BOTH hold, CONTINUOUSLY for stable_dur,
    before GPS is declared ready:

      1. QUALITY   fix_type >= min_fix_type (3D)
                   satellites >= min_satellites
                   HDOP <= max_hdop
                   h_acc <= max_h_acc            (only if require_h_acc)
         Missing / "unknown" values count as BAD — an unknown fix is the
         exact situation that caused the incident.

      2. STABILITY satellite count swing over the window <= max_sat_swing
                   HDOP swing over the window        <= max_hdop_swing
                   horizontal position peak-to-peak  <= max_pos_drift
                     (position check is applied ONLY while DISARMED / on
                      the ground — in flight the drone legitimately moves)

    Publishes:
        /px4/gps_ready  (std_msgs/Bool, latched)  — True only when good+stable
        /px4/gps_check  (std_msgs/String, JSON)   — full detail / reason
    and prints a one-line [GPSCHK] status at 1 Hz.

    It keeps running after takeoff, so if GPS degrades in the air
    /px4/gps_ready drops back to False and a warning is logged. It does
    NOT command the vehicle — degraded-mode action is an open question
    (AUTO.LAND relies on the same estimate); this node only reports.

USAGE
    # px4_sensor_reader (from takeoff_land or fm_deploy) must be running
    ros2 run gps_guard gps_check_node
    ros2 run gps_guard gps_check_node --ros-args -p min_satellites:=10
    ros2 launch gps_guard gps_guard.launch.py min_satellites:=10
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from std_msgs.msg import Bool, String


# Latched QoS so a late-joining flight node still gets the last
# /px4/gps_ready value without waiting for the next publish.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_FIX_NAMES = {0: "NO GPS", 1: "NO FIX", 2: "2D fix", 3: "3D fix",
              4: "DGPS", 5: "RTK-float", 6: "RTK-fixed"}


class GpsCheckNode(Node):

    def __init__(self):
        super().__init__("gps_check_node")

        # ── Quality thresholds (same meaning/defaults as takeoff_land_node) ──
        self.declare_parameter("min_fix_type",     3)      # 3 = 3D fix (GPSRAW enum)
        self.declare_parameter("min_satellites",   8)
        self.declare_parameter("max_hdop",         2.0)
        self.declare_parameter("require_h_acc",    False)   # gate on h_acc too?
        self.declare_parameter("max_h_acc",        5.0)     # m, only if require_h_acc

        # ── Stability thresholds ────────────────────────────────────────────
        self.declare_parameter("stable_dur",       5.0)     # s quality must hold
        self.declare_parameter("window_s",         5.0)     # s window for the swings
        self.declare_parameter("max_sat_swing",    4)       # sat count peak-to-peak
        self.declare_parameter("max_hdop_swing",   0.8)     # HDOP peak-to-peak
        self.declare_parameter("max_pos_drift",    0.30)    # m horizontal p2p, on ground

        self.declare_parameter("sample_hz",        10.0)    # matches px4_sensor_reader
        self.declare_parameter("status_period_s",  1.0)

        self._min_fix       = int(self.get_parameter("min_fix_type").value)
        self._min_sats      = int(self.get_parameter("min_satellites").value)
        self._max_hdop      = float(self.get_parameter("max_hdop").value)
        self._require_hacc  = bool(self.get_parameter("require_h_acc").value)
        self._max_hacc      = float(self.get_parameter("max_h_acc").value)
        self._stable_dur    = float(self.get_parameter("stable_dur").value)
        self._window_s      = float(self.get_parameter("window_s").value)
        self._max_sat_swing = int(self.get_parameter("max_sat_swing").value)
        self._max_hdop_swing = float(self.get_parameter("max_hdop_swing").value)
        self._max_pos_drift = float(self.get_parameter("max_pos_drift").value)
        self._sample_hz     = float(self.get_parameter("sample_hz").value)
        self._status_period = float(self.get_parameter("status_period_s").value)

        self._win_n = max(10, int(self._window_s * self._sample_hz))

        # ── State from the reader ──────────────────────────────────────────
        self._armed   = False
        self._gps_q   = None            # dict, or None until first message
        self._pos_xy  = None            # (x, y), or None until first pose
        self._have_pose = False

        # ── Rolling history for the swing / drift checks ──────────────────
        self._hist = deque(maxlen=self._win_n)   # each: (sats, hdop, x, y)

        # ── Streak tracking ───────────────────────────────────────────────
        self._good_since = None        # time.time() when the good streak started
        self._ready      = False       # last published /px4/gps_ready value
        self._ever_ready = False       # for the "degraded in flight" warning
        self._last_reason = "starting up"
        self._last_metrics = {}

        # ── Pub / sub ─────────────────────────────────────────────────────
        self._pub_ready = self.create_publisher(Bool, "/px4/gps_ready", LATCHED_QOS)
        self._pub_check = self.create_publisher(String, "/px4/gps_check", 10)

        self.create_subscription(String, "/px4/state",   self._cb_state,   10)
        self.create_subscription(String, "/px4/sensors", self._cb_sensors, 10)

        # Publish an explicit "not ready yet" immediately so a subscriber
        # that comes up first never sees a stale/absent value as "ready".
        self._publish_ready(False)

        period = 1.0 / self._sample_hz
        self._eval_timer   = self.create_timer(period, self._evaluate)
        self._status_timer = self.create_timer(self._status_period, self._print_status)

        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"[GPSCHK] gps_check_node ready. Need fix>={self._min_fix} (3D), "
            f"sats>={self._min_sats}, HDOP<={self._max_hdop:.1f}"
            + (f", h_acc<={self._max_hacc:.1f}m" if self._require_hacc else ""))
        self.get_logger().info(
            f"[GPSCHK] Stability: held {self._stable_dur:.0f}s, "
            f"sat swing<={self._max_sat_swing}, HDOP swing<={self._max_hdop_swing:.1f}, "
            f"ground x/y drift<={self._max_pos_drift:.2f}m over {self._window_s:.0f}s")
        self.get_logger().info("=" * 62)

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._armed = bool(d.get("armed", False))
        except Exception:
            pass

    def _cb_sensors(self, msg: String):
        try:
            d = json.loads(msg.data)
            q = d.get("gps_quality")
            if isinstance(q, dict):
                self._gps_q = q
            if "local_x" in d:
                self._pos_xy = (float(d.get("local_x", 0.0)),
                                float(d.get("local_y", 0.0)))
                self._have_pose = True
        except Exception:
            pass

    # ── Quality check (single snapshot) ──────────────────────────────────────
    def _quality_problem(self):
        """Return None when the current GPS snapshot passes the quality bar,
        else a short human-readable reason. Unknown/missing == BAD."""
        q = self._gps_q
        if not q:
            return "no gps_quality from /px4/sensors yet"

        fix = q.get("fix_type")
        if fix is None:
            return "fix_type unknown"
        if int(fix) < self._min_fix:
            return (f"fix_type={_FIX_NAMES.get(int(fix), fix)} "
                    f"(need >= {self._min_fix} = 3D fix)")

        sats = q.get("satellites")
        if sats is None:
            return "satellite count unknown"
        if int(sats) < self._min_sats:
            return f"only {int(sats)} satellites (need >= {self._min_sats})"

        hdop = q.get("hdop")
        if hdop is None:
            return "HDOP unknown"
        if float(hdop) > self._max_hdop:
            return f"HDOP {float(hdop):.2f} too high (need <= {self._max_hdop:.2f})"

        if self._require_hacc:
            hacc = q.get("h_acc_m")
            if hacc is None:
                return "h_acc unknown (require_h_acc=true)"
            if float(hacc) > self._max_hacc:
                return (f"h_acc {float(hacc):.2f}m too high "
                        f"(need <= {self._max_hacc:.2f}m)")
        return None

    # ── Stability check (over the rolling window) ────────────────────────────
    def _stability_problem(self):
        """Return (problem_or_None, metrics_dict). metrics may be partial
        while the window is still filling."""
        m = {"sat_swing": None, "hdop_swing": None, "pos_drift_m": None,
             "window": f"{len(self._hist)}/{self._win_n}"}
        if len(self._hist) < self._win_n:
            return f"collecting samples ({len(self._hist)}/{self._win_n})", m

        sats = [h[0] for h in self._hist if h[0] is not None]
        hdops = [h[1] for h in self._hist if h[1] is not None]
        xs = [h[2] for h in self._hist if h[2] is not None]
        ys = [h[3] for h in self._hist if h[3] is not None]

        if len(sats) == len(self._hist):
            m["sat_swing"] = max(sats) - min(sats)
        if len(hdops) == len(self._hist):
            m["hdop_swing"] = round(max(hdops) - min(hdops), 3)
        if xs and ys:
            m["pos_drift_m"] = round(math.hypot(max(xs) - min(xs),
                                                max(ys) - min(ys)), 3)

        if m["sat_swing"] is not None and m["sat_swing"] > self._max_sat_swing:
            return (f"satellite count swinging by {m['sat_swing']} over "
                    f"{self._window_s:.0f}s (need <= {self._max_sat_swing})"), m
        if m["hdop_swing"] is not None and m["hdop_swing"] > self._max_hdop_swing:
            return (f"HDOP swinging by {m['hdop_swing']:.2f} over "
                    f"{self._window_s:.0f}s (need <= {self._max_hdop_swing:.2f})"), m

        # Position stillness only makes sense on the ground. In flight we
        # still report the number, but it must not fail the gate.
        if not self._armed:
            if m["pos_drift_m"] is None:
                return "no local position for the ground-stillness check", m
            if m["pos_drift_m"] > self._max_pos_drift:
                return (f"position estimate wandering {m['pos_drift_m']:.2f}m "
                        f"while ON THE GROUND (need <= {self._max_pos_drift:.2f}m) "
                        f"— this is the 2026-08-27 failure"), m
        return None, m

    # ── Main evaluation (timer) ─────────────────────────────────────────────
    def _evaluate(self):
        q = self._gps_q or {}
        self._hist.append((
            None if q.get("satellites") is None else int(q["satellites"]),
            None if q.get("hdop") is None else float(q["hdop"]),
            self._pos_xy[0] if self._pos_xy else None,
            self._pos_xy[1] if self._pos_xy else None,
        ))

        qprob = self._quality_problem()
        sprob, metrics = self._stability_problem()
        now = time.time()

        if qprob is None and sprob is None:
            if self._good_since is None:
                self._good_since = now
            held = now - self._good_since
            ready = held >= self._stable_dur
            reason = ("OK" if ready
                      else f"good, holding {held:.1f}/{self._stable_dur:.0f}s")
        else:
            self._good_since = None
            held = 0.0
            ready = False
            reason = qprob or sprob

        self._last_reason = reason
        self._last_metrics = metrics

        if ready != self._ready:
            if ready:
                self._ever_ready = True
                self.get_logger().info("[GPSCHK] GPS READY — quality good and stable.")
            else:
                lvl = self.get_logger().error if self._ever_ready else self.get_logger().warn
                lvl(f"[GPSCHK] GPS NOT READY: {reason}")
                if self._ever_ready:
                    self.get_logger().error(
                        "[GPSCHK] GPS degraded after being ready. This node does "
                        "NOT command the vehicle — pilot/monitor must decide.")
        self._publish_ready(ready)
        self._publish_check(ready, reason, held, metrics)

    def _publish_ready(self, ready: bool):
        self._ready = ready
        self._pub_ready.publish(Bool(data=ready))

    def _publish_check(self, ready, reason, held, metrics):
        q = self._gps_q or {}
        payload = {
            "ready": ready,
            "reason": reason,
            "held_s": round(held, 2),
            "armed": self._armed,
            "fix_type": q.get("fix_type"),
            "satellites": q.get("satellites"),
            "hdop": q.get("hdop"),
            "h_acc_m": q.get("h_acc_m"),
            "sat_swing": metrics.get("sat_swing"),
            "hdop_swing": metrics.get("hdop_swing"),
            "pos_drift_m": metrics.get("pos_drift_m"),
            "window": metrics.get("window"),
        }
        self._pub_check.publish(String(data=json.dumps(payload)))

    # ── 1 Hz terminal line ─────────────────────────────────────────────────
    def _print_status(self):
        q = self._gps_q
        if q is None:
            self.get_logger().warn(
                "[GPSCHK] Waiting for gps_quality from px4_sensor_reader...")
            return
        fix = q.get("fix_type")
        fix_s = _FIX_NAMES.get(int(fix), str(fix)) if fix is not None else "?"
        sats = q.get("satellites")
        hdop = q.get("hdop")
        drift = self._last_metrics.get("pos_drift_m")
        tag = "READY" if self._ready else "NOT READY"
        self.get_logger().info(
            f"[GPSCHK] {tag}  fix={fix_s}  sats={'?' if sats is None else sats}  "
            f"HDOP={'?' if hdop is None else f'{float(hdop):.2f}'}  "
            f"gnd_drift={'?' if drift is None else f'{drift:.2f}m'}  "
            f"({self._last_reason})")


def main(args=None):
    rclpy.init(args=args)
    node = GpsCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[GPSCHK] Stopped by user (Ctrl-C).")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
