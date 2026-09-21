#!/usr/bin/env python3
"""
vfh_avoidance_node.py — VFH perception node for the real drone.
==================================================================
The user's `vfh_obstacle_avoidance_node.py` (2026-09-14) adapted to THIS
drone. What is different from the original file is tagged NEW-VFH:

  * Camera = Orbbec Gemini 2 (front-facing, fixed), NOT a RealSense / SIYI.
    The depth comes through fm_deploy's gemini2_depth_bridge_node:
        /camera/depth/image_raw (16UC1 mm, driver)
            -> /realsense/depth/float32  (32FC1 METERS, 640x480)
            -> this node (x1000 -> mm; the VFH core works in mm)
    The bridge fills invalid pixels with 10.0 m ("far/safe"), which is
    above d_max (5 m) and therefore ignored by the histogram — the same
    as the original NaN treatment.
  * hfov_deg default 91 (Gemini 2). When /realsense/depth/camera_info
    arrives, the FOV is recomputed from fx (2*atan(W/2fx)) so the sector
    mapping is exact.
  * /vfh/movement_direction gets three extra fields the flight node uses:
        front_min_m   closest valid depth within +-12 deg of the axis (m)
        age_ok        depth frame is fresh
        seq           frame counter
  * show_debug defaults to False (headless Jetson); publish_visual stays.
  * GROUND FILTER from the BAROMETRIC altitude (ground_filter, default on).
    The camera is level, so at low altitude the ground fills the lower
    rows of the ROI within d_max (Gemini 2 VFOV 66 deg: at 2 m the ground
    is 4.4 m away at the 85 % row) and, because the histogram is normalised
    to its own maximum, an otherwise empty scene then reads as BLOCKED
    EVERYWHERE (-> 'stop'). The flight node publishes its barometric
    altitude above the take-off point on /vfh/agl_m; every pixel whose
    implied drop below the camera, depth * (v - cy) / fy, exceeds
    agl - ground_margin_m is treated as ground (invalid). Without a fresh
    /vfh/agl_m (bench use) the filter is inactive and a warning is logged.
  * roi_top / roi_bottom expose the original 15 %-85 % vertical ROI.

Subscribe : /realsense/depth/float32     (sensor_msgs/Image 32FC1 m)
            /realsense/depth/camera_info (sensor_msgs/CameraInfo)
            /vfh/target_angle            (std_msgs/Float32 deg, 0=front, +=right)
            /vfh/agl_m                   (std_msgs/Float32 m above take-off, baro)
Publish   : /vfh/movement_direction  (std_msgs/String JSON: state, steer_deg, ...)
            /vfh/obstacles           (std_msgs/String JSON obstacle map)
            /vfh/visual              (sensor_msgs/Image bgr8 debug overlay)
            /vfh/cmd_vel             (geometry_msgs/Twist, empty — kept for
                                      compatibility with the original graph)

Coordinate convention: 0 deg = straight ahead, negative = LEFT,
positive = RIGHT. steer_deg range -40..+40.
"""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from vfh_avoidance_barometer.vfh_core import VFH

HAS_DISPLAY = bool(
    os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
)

_DEPTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)


# ---------------------------------------------------------------------------
# Debug Visualiser (unchanged from the original)
# ---------------------------------------------------------------------------

class VFHVisualiser:
    """Renders VFH state as a BGR debug image."""

    def __init__(self, width: int = 960, height: int = 540) -> None:
        self.W = width
        self.H = height

    def render(self, depth_bgr, vfh_result, n_sectors, threshold,
               info_line: str = "") -> np.ndarray:
        canvas = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        half   = self.W // 2

        if depth_bgr is not None:
            thumb = cv2.resize(depth_bgr, (half, self.H))
            canvas[:, :half] = thumb

        s        = self.H / 400.0
        vx       = half
        smoothed = np.array(vfh_result["smoothed_hist"])
        binary   = np.array(vfh_result["binary_hist"])
        n        = n_sectors
        bar_w    = max(1, (self.W - half - 20) // n)
        chart_h  = int(120 * s)
        base_y   = int(20 * s) + chart_h

        for i in range(n):
            bh      = int(smoothed[i] * chart_h)
            x       = vx + 10 + i * bar_w
            blocked = binary[i] == 1
            colour  = (50, 50, 220) if blocked else (50, 180, 80)
            cv2.rectangle(canvas, (x, base_y - bh), (x + bar_w - 1, base_y), colour, -1)

        ty = base_y - int(threshold * chart_h)
        cv2.line(canvas, (vx + 10, ty), (self.W - 10, ty), (180, 100, 220), 1)
        cv2.putText(canvas, f"thresh={threshold:.2f}", (vx + 12, ty - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.47 * s, (180, 100, 220), 1)

        sector_size = 360.0 / n
        for deg in [0, 90, 180, 270]:
            idx = int(round(deg / sector_size)) % n
            x   = vx + 10 + idx * bar_w
            cv2.putText(canvas, f"{deg}", (x, base_y + int(14 * s)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40 * s, (160, 160, 160), 1)

        cx  = vx + (self.W - vx) // 2
        cy  = base_y + int(20 * s) + int(108 * s)
        rad = int(108 * s)
        cv2.circle(canvas, (cx, cy), rad,      (60, 60, 60), 1)
        cv2.circle(canvas, (cx, cy), rad // 2, (40, 40, 40), 1)

        for i in range(n):
            ang_rad = math.radians(i * sector_size - 90)
            r_bar   = int(smoothed[i] * rad)
            x3 = int(cx + r_bar * math.cos(ang_rad))
            y3 = int(cy + r_bar * math.sin(ang_rad))
            col = (50, 50, 200) if binary[i] == 1 else (50, 160, 60)
            cv2.line(canvas, (cx, cy), (x3, y3), col, 2)

        for v in vfh_result.get("valleys", []):
            vang = math.radians(v["center_deg"] - 90)
            vx2  = int(cx + (rad + int(10 * s)) * math.cos(vang))
            vy2  = int(cy + (rad + int(10 * s)) * math.sin(vang))
            cv2.circle(canvas, (vx2, vy2), int(5 * s), (0, 255, 180), -1)

        bv = vfh_result.get("best_valley")
        if bv:
            sang = math.radians(vfh_result.get("steer_deg", bv["steering_deg"]) - 90)
            sx2  = int(cx + (rad + int(18 * s)) * math.cos(sang))
            sy2  = int(cy + (rad + int(18 * s)) * math.sin(sang))
            cv2.arrowedLine(canvas, (cx, cy), (sx2, sy2), (0, 220, 255), 2, tipLength=0.25)
        tang = math.radians(vfh_result.get("target_deg", 0.0) - 90)
        cv2.line(canvas, (cx, cy), (int(cx + rad * math.cos(tang)),
                                    int(cy + rad * math.sin(tang))), (255, 200, 0), 1)

        cv2.circle(canvas, (cx, cy), int(10 * s), (200, 200, 200), -1)
        cv2.circle(canvas, (cx, cy), int(10 * s), (255, 255, 255),  1)

        cmd   = vfh_result.get("cmd", {})
        state = cmd.get("state", "?")
        obs   = cmd.get("obstacle_direction", "none")
        bv_deg = vfh_result.get("steer_deg", float("nan")) if bv else float("nan")
        state_col = {"move": (80, 200, 80), "avoid": (50, 160, 220),
                     "stop": (50, 50, 220)}.get(state, (200, 200, 200))

        bar_h  = int(108 * s)
        info_y = self.H - bar_h
        cv2.rectangle(canvas, (0, info_y - 4), (self.W, self.H), (20, 20, 20), -1)
        cv2.putText(canvas, f"STATE: {state.upper()}", (8, info_y + int(18 * s)),
                    cv2.FONT_HERSHEY_DUPLEX, 0.68 * s, state_col, 1, cv2.LINE_AA)
        cv2.putText(canvas, f"obstacle={obs}  steer={bv_deg:.1f}deg", (8, info_y + int(40 * s)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.51 * s, (200, 200, 200), 1)
        n_blocked = int(np.sum(binary))
        n_valleys = len(vfh_result.get("valleys", []))
        cv2.putText(canvas,
                    f"blocked={n_blocked}/{n}  valleys={n_valleys}  "
                    f"target={vfh_result.get('target_deg', 0.0):.1f}deg",
                    (8, info_y + int(60 * s)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.51 * s, (150, 150, 150), 1)
        cv2.putText(canvas, info_line, (8, info_y + int(80 * s)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45 * s, (100, 100, 100), 1)
        cv2.line(canvas, (half, 0), (half, self.H), (60, 60, 60), 1)
        return canvas


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class VFHAvoidanceNode(Node):

    def __init__(self) -> None:
        super().__init__("vfh_avoidance_node")

        self.declare_parameter("n_sectors",        36)
        self.declare_parameter("threshold",        0.45)
        self.declare_parameter("smooth_window",    2)
        self.declare_parameter("d_max",            5000.0)   # mm
        self.declare_parameter("d_min",            300.0)    # mm
        self.declare_parameter("min_valley_width", 3)
        self.declare_parameter("target_angle",     0.0)
        self.declare_parameter("frame_skip",       1)
        self.declare_parameter("publish_visual",   True)
        self.declare_parameter("show_debug",       False)    # NEW-VFH: headless default
        self.declare_parameter("robot_radius",     346.0)    # mm
        self.declare_parameter("safety_dist",      500.0)    # mm
        self.declare_parameter("hfov_deg",         91.0)     # NEW-VFH: Gemini 2
        self.declare_parameter("use_camera_info",  True)     # NEW-VFH: hfov from fx
        self.declare_parameter("depth_topic",      "/realsense/depth/float32")   # NEW-VFH
        self.declare_parameter("camera_info_topic", "/realsense/depth/camera_info")
        self.declare_parameter("depth_source",     "float32")   # float32 (m) | uint16 (mm)
        self.declare_parameter("log_period_s",     1.0)
        self.declare_parameter("roi_top",          0.15)     # NEW-VFH: original ROI
        self.declare_parameter("roi_bottom",       0.85)
        self.declare_parameter("ground_filter",    True)     # NEW-VFH: see docstring
        self.declare_parameter("ground_margin_m",  0.5)      # m above the ground kept
        self.declare_parameter("agl_topic",        "/vfh/agl_m")
        self.declare_parameter("agl_stale_s",      2.0)      # older AGL -> filter off

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        n_sectors        = int(g("n_sectors"))
        threshold        = float(g("threshold"))
        smooth_window    = int(g("smooth_window"))
        d_max            = float(g("d_max"))
        d_min            = float(g("d_min"))
        min_valley_width = int(g("min_valley_width"))
        self._target_angle   = float(g("target_angle"))
        self._frame_skip     = max(1, int(g("frame_skip")))
        self._publish_visual = bool(g("publish_visual"))
        self._show_debug     = bool(g("show_debug"))
        robot_radius         = float(g("robot_radius"))
        safety_dist          = float(g("safety_dist"))
        hfov_deg             = float(g("hfov_deg"))
        self._use_cam_info   = bool(g("use_camera_info"))
        depth_topic          = str(g("depth_topic"))
        info_topic           = str(g("camera_info_topic"))
        self._depth_source   = str(g("depth_source"))
        self._log_period     = float(g("log_period_s"))
        roi_top              = float(g("roi_top"))
        roi_bottom           = float(g("roi_bottom"))
        self._ground_filter  = bool(g("ground_filter"))
        self._ground_margin  = float(g("ground_margin_m"))
        agl_topic            = str(g("agl_topic"))
        self._agl_stale_s    = float(g("agl_stale_s"))
        self._agl            = None       # (value m, monotonic time)
        self._fy = None; self._cy = None  # from camera_info (scaled)
        self._drop_col = None             # (v - cy) / fy per row, cached
        self._ground_state = None         # last logged filter state

        self._enable_display = HAS_DISPLAY and self._show_debug

        self._vfh = VFH(
            n_sectors=n_sectors, threshold=threshold, smooth_window=smooth_window,
            d_max=d_max, d_min=d_min, min_valley_width=min_valley_width,
            robot_radius=robot_radius, safety_dist=safety_dist, hfov_deg=hfov_deg,
            roi_top=roi_top, roi_bottom=roi_bottom)
        self._vis = VFHVisualiser(width=960, height=540)

        self._skip_counter = 0
        self._seq          = 0
        self._last_result  = None
        self._hfov_from_info = None
        self._t_last_frame = None
        self._proc_ms      = 0.0

        self._bridge = CvBridge()
        self.create_subscription(Image, depth_topic, self._depth_callback, _DEPTH_QOS)
        if self._use_cam_info:
            self.create_subscription(CameraInfo, info_topic, self._info_callback, _DEPTH_QOS)
        self.create_subscription(Float32, "/vfh/target_angle", self._target_callback, 10)
        self.create_subscription(Float32, agl_topic, self._agl_callback, 10)

        self._pub_cmd       = self.create_publisher(Twist,  "/vfh/cmd_vel",   _CMD_QOS)
        self._pub_obstacles = self.create_publisher(String, "/vfh/obstacles", 10)
        self._pub_debug     = self.create_publisher(String, "/vfh/movement_direction", 10)
        self._pub_visual    = self.create_publisher(Image,  "/vfh/visual",    _DEPTH_QOS)

        L = self.get_logger()
        L.info("=" * 62)
        L.info("VFH Obstacle Avoidance Node  [vfh_avoidance_barometer]")
        L.info("=" * 62)
        L.info(f"  Depth          : {depth_topic} ({self._depth_source}"
               f"{', m -> mm' if self._depth_source == 'float32' else ', mm'})")
        L.info("  Camera         : Orbbec Gemini 2 via gemini2_depth_bridge_node")
        L.info(f"  hfov           : {hfov_deg:.1f} deg"
               + (" (updated from camera_info when it arrives)" if self._use_cam_info else ""))
        L.info(f"  Drone          : 49cm x 49cm  robot_radius={robot_radius:.0f} mm  "
               f"safety_dist={safety_dist:.0f} mm")
        L.info(f"  n_sectors={n_sectors} ({360.0 / n_sectors:.0f} deg)  threshold={threshold}  "
               f"smooth={smooth_window}  min_valley_w={min_valley_width} "
               f"(= {min_valley_width * 360.0 / n_sectors:.0f} deg)")
        L.info(f"  d_min={d_min:.0f}mm  d_max={d_max:.0f}mm  "
               f"FOV sectors {int(self._vfh.fov_mask.sum())}/{n_sectors}")
        L.info(f"  ROI rows       : {roi_top:.2f}-{roi_bottom:.2f} of the image height")
        L.info(f"  Ground filter  : {'ON' if self._ground_filter else 'off'} "
               f"(baro AGL from {agl_topic}, keep {self._ground_margin:.1f} m above ground)")
        L.info("  Publishing     : /vfh/movement_direction  /vfh/obstacles  /vfh/visual")
        L.info("=" * 62)

    # ── callbacks ────────────────────────────────────────────────────────

    def _info_callback(self, msg: CameraInfo) -> None:
        """NEW-VFH: exact horizontal FOV from the (scaled) intrinsics."""
        try:
            fx = float(msg.k[0]); w = float(msg.width)
            if fx <= 0.0 or w <= 0.0:
                return
            hfov = math.degrees(2.0 * math.atan(w / (2.0 * fx)))
            fy = float(msg.k[4]); cy = float(msg.k[5])
            if fy > 0.0:
                self._fy, self._cy = fy, cy
                self._drop_col = None
        except Exception:
            return
        if self._hfov_from_info is None or abs(hfov - self._hfov_from_info) > 0.5:
            self._hfov_from_info = hfov
            self._vfh.set_hfov(hfov)
            self.get_logger().info(
                f"[VFH] hfov from camera_info: {hfov:.1f} deg "
                f"(fx={fx:.1f}, w={int(w)}) -> FOV sectors "
                f"{int(self._vfh.fov_mask.sum())}/{self._vfh.n_sectors}")

    def _target_callback(self, msg: Float32) -> None:
        self._target_angle = float(msg.data) % 360.0

    def _agl_callback(self, msg: Float32) -> None:
        self._agl = (float(msg.data), time.monotonic())

    def _apply_ground_filter(self, depth_mm: np.ndarray) -> str:
        """NEW-VFH: mark pixels that lie on (or below) the ground as invalid.
        Returns a short state string for the logs."""
        if not self._ground_filter:
            return "off"
        if self._agl is None or time.monotonic() - self._agl[1] > self._agl_stale_s:
            return "no-agl"
        h, w = depth_mm.shape
        if self._drop_col is None or self._drop_col.shape[0] != h:
            fy = self._fy
            cy = self._cy
            if fy is None or fy <= 0.0:
                # fallback: square pixels, fx from the hfov
                fy = w / (2.0 * math.tan(math.radians(self._vfh.hfov_deg) / 2.0))
            if cy is None:
                cy = h / 2.0
            self._drop_col = ((np.arange(h, dtype=np.float32) - cy) / fy)[:, None]
        limit_mm = max(self._agl[0] - self._ground_margin, 0.2) * 1000.0
        # drop below the camera = depth * (v - cy) / fy  (rows below the
        # centre are positive); ground = anything at/under agl - margin
        with np.errstate(invalid="ignore"):
            ground = (depth_mm * self._drop_col) > limit_mm
        depth_mm[ground] = np.nan
        return f"agl={self._agl[0]:.1f}m"

    def _depth_callback(self, msg: Image) -> None:
        self._skip_counter += 1
        if self._skip_counter % self._frame_skip != 0:
            return
        t_proc = time.monotonic()
        try:
            if self._depth_source == "uint16":
                raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                depth_mm = raw.astype(np.float32)
                depth_mm[(raw == 0) | (raw == 65535)] = np.nan
            else:
                raw_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                depth_mm = raw_m * 1000.0
                depth_mm[~(raw_m > 0.0)] = np.nan   # 0 / NaN / negative -> no data
        except Exception as exc:
            self.get_logger().error(f"cv_bridge error: {exc}", throttle_duration_sec=5.0)
            return

        gstate = self._apply_ground_filter(depth_mm)
        if gstate != self._ground_state:
            self._ground_state = gstate
            if gstate == "no-agl":
                self.get_logger().warn(
                    "[VFH] ground filter INACTIVE: no fresh /vfh/agl_m (flight node "
                    "not running?). Low-altitude ground may read as an obstacle.")
            else:
                self.get_logger().info(f"[VFH] ground filter: {gstate}")

        try:
            result = self._vfh.run(depth_map=depth_mm,
                                   target_deg=self._target_angle, robot_heading=0.0)
        except Exception as exc:
            self.get_logger().error(f"VFH error: {exc}", throttle_duration_sec=5.0)
            return

        self._seq += 1
        self._last_result = result
        self._t_last_frame = time.monotonic()
        self._proc_ms = (self._t_last_frame - t_proc) * 1000.0
        now = self.get_clock().now().to_msg()

        self._pub_cmd.publish(Twist())

        obstacle_groups = self._vfh.extract_obstacle_map(
            smoothed_hist=np.array(result["smoothed_hist"]),
            binary_hist=np.array(result["binary_hist"]))
        self._pub_obstacles.publish(String(data=json.dumps({
            "stamp":        f"{now.sec}.{now.nanosec:09d}",
            "n_obstacles":  len(obstacle_groups),
            "has_critical": any(g["is_critical"] for g in obstacle_groups),
            "obstacles":    obstacle_groups,
        })))

        bv = result.get("best_valley")
        fmin = result.get("front_min_mm")
        self._pub_debug.publish(String(data=json.dumps({
            "stamp":         f"{now.sec}.{now.nanosec:09d}",
            "seq":           self._seq,
            "target_deg":    result["target_deg"],
            "state":         result["cmd"]["state"],
            "obstacle":      result["cmd"]["obstacle_direction"],
            "avoid_toward":  result["cmd"]["avoid_direction"],
            "steer_deg":     result["steer_deg"],   # includes the lateral penalty
            "lateral_delta": result["lateral_delta"],
            "valley_center": round(bv["center_deg"], 1) if bv else None,
            "n_valleys":     len(result["valleys"]),
            "n_blocked":     int(np.sum(result["binary_hist"])),
            "front_min_m":   None if fmin is None else round(fmin / 1000.0, 2),
            "hfov_deg":      round(self._vfh.hfov_deg, 1),
            "ground_filter": gstate,
            "proc_ms":       round(self._proc_ms, 1),
            "depth_topic":   msg.header.frame_id,
        })))

        if self._publish_visual or self._enable_display:
            vis = self._vis.render(
                self._colorize_depth(depth_mm), result,
                self._vfh.n_sectors, self._vfh.threshold,
                info_line=(f"robot_r={self._vfh.robot_radius:.0f}mm "
                           f"d_min={self._vfh.d_min:.0f}mm d_max={self._vfh.d_max:.0f}mm "
                           f"hfov={self._vfh.hfov_deg:.0f}deg "
                           f"front_min={'-' if fmin is None else f'{fmin / 1000.0:.2f}m'} "
                           f"{self._proc_ms:.0f}ms"))
            if self._publish_visual:
                try:
                    vis_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
                    vis_msg.header.stamp    = now
                    vis_msg.header.frame_id = msg.header.frame_id
                    self._pub_visual.publish(vis_msg)
                except Exception as exc:
                    self.get_logger().error(f"Publish visual error: {exc}",
                                            throttle_duration_sec=5.0)
            if self._enable_display:
                cv2.namedWindow("VFH Obstacle Avoidance", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("VFH Obstacle Avoidance", 960, 540)
                cv2.imshow("VFH Obstacle Avoidance", vis)
                cv2.waitKey(1)

        cmd = result["cmd"]
        self.get_logger().info(
            f"[VFH] state={cmd['state']} obstacle={cmd['obstacle_direction']} "
            f"avoid={cmd['avoid_direction']} steer={result['steer_deg']:+.1f}deg "
            f"target={result['target_deg']:.1f} "
            f"front_min={'-' if fmin is None else f'{fmin / 1000.0:.2f}m'} "
            f"valleys={len(result['valleys'])} blocked={int(np.sum(result['binary_hist']))} "
            f"gnd={gstate} {self._proc_ms:.0f}ms",
            throttle_duration_sec=self._log_period)

    @staticmethod
    def _colorize_depth(depth_mm: np.ndarray) -> np.ndarray:
        D_MIN_MM = 600.0
        D_MAX_MM = 6000.0
        valid_mask = np.isfinite(depth_mm) & (depth_mm > 0)
        if not valid_mask.any():
            return np.zeros((*depth_mm.shape, 3), dtype=np.uint8)
        d = np.nan_to_num(depth_mm, nan=D_MAX_MM)
        norm    = ((d - D_MIN_MM) / (D_MAX_MM - D_MIN_MM) * 255).clip(0, 255).astype(np.uint8)
        colored = cv2.applyColorMap(255 - norm, cv2.COLORMAP_JET)
        colored[~valid_mask] = 0
        return colored

    def destroy_node(self) -> None:
        if self._enable_display:
            cv2.destroyAllWindows()
        try:
            self._pub_cmd.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VFHAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
