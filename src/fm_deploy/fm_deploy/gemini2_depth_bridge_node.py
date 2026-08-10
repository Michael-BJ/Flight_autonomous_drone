#!/usr/bin/env python3
# gemini2_depth_bridge_node.py — v1.0 — Orbbec Gemini 2 -> FM model input contract
"""
gemini2_depth_bridge_node.py — REAL camera bridge -> model input contract
==========================================================================
Real-world equivalent of `gz_depth_bridge_node.py` (simulation). In
simulation Gazebo publishes /depth_camera (32FC1, meters) and that node
just forwards it to /realsense/depth/float32. On the real drone the source
is the Orbbec driver (`orbbec_camera`), which publishes 16UC1 in
MILLIMETERS on /camera/depth/image_raw. This node reconciles the two so
the ENTIRE downstream pipeline (depth_to_pointcloud -> octomap -> ESDF,
and the FM model input) needs no changes at all.

Output (identical to sim):
    /realsense/depth/float32       sensor_msgs/Image       32FC1, METERS
    /realsense/depth/camera_info   sensor_msgs/CameraInfo  640x480 intrinsics
    /realsense/depth/z16           sensor_msgs/Image       16UC1, mm (optional)

────────────────────────────────────────────────────────────────────────────
WHY THIS NODE CAN'T JUST BE A "TOPIC REMAP"
────────────────────────────────────────────────────────────────────────────
There are three physical differences between Gazebo depth and Gemini 2
depth that, left unhandled, make the FM model see input DIFFERENT from
its training data:

1. UNIT & DTYPE. Gazebo: float32 meters. Gemini 2: uint16 millimeters.
   -> converted here (x depth_scale, default 0.001).

2. INVALID PIXELS (the most dangerous one).
   Gazebo returns +inf for "no obstacle up to the far clip".
   `_form_model_input` maps +inf -> DEPTH_NORM_MAX_M (10 m = FAR/SAFE).
   Gemini 2 returns 0 for "no return" — and 0 under the same convention
   means "obstacle TOUCHING THE LENS". In the real world, the cause of a 0
   reading is actually often something far/safe: a shiny surface, a
   window, an object beyond 10 m, a stereo dropout hole. If 0 were passed
   through as-is, the model would see a solid black fog right in front of
   its nose and panic — even though the room is empty.
   -> the `invalid_fill_m` parameter (DEFAULT 10.0 = far/safe) fills
      invalid pixels. `hole_fill_px > 0` first patches SMALL holes with the
      neighborhood median (a small hole in the middle of a wall should
      read like the wall, not 10 m), then fills the rest as far.
      ONLY set invalid_fill_m:=0.0 if you retrain the model with that
      convention.

3. RESOLUTION & FOV. The model is trained at 640x480. Gemini 2 can publish
   848x480/1280x800 etc. -> resized to 640x480 with INTER_NEAREST (NOT
   linear interpolation: averaging between a 1 m obstacle edge and an 8 m
   background produces a "ghost" reading of 4.5 m where nothing exists).
   The camera_info intrinsics are scaled along with it so the point cloud
   stays metrically correct.

Depth is also clipped to [min_valid_m, DEPTH_NORM_MAX_M=10.0] since that's
the range the model's input normalization expects (see DEPTH_NORM_MAX_M in
fm_inference_base.py / expert_planner_node.py).

────────────────────────────────────────────────────────────────────────────
USAGE
────────────────────────────────────────────────────────────────────────────
    ros2 run fm_deploy gemini2_depth_bridge_node --ros-args \
        -p input_topic:=/camera/depth/image_raw \
        -p input_info_topic:=/camera/depth/camera_info

Verify before flying (drone in hand, point it at a wall ~1.5 m away):
    ros2 topic hz   /realsense/depth/float32     # should be a stable >= 10 Hz
    ros2 topic echo /realsense/depth/stats       # p50 should be ~1.5, valid% high
"""
import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# Model input contract — MUST match fm_inference_base.py
_W = 640
_H = 480
DEPTH_NORM_MAX_M = 10.0

# Fallback intrinsics for when camera_info hasn't arrived from the driver yet.
# HFOV 91 deg = Orbbec Gemini 2 (the same value used in camera_vfh.launch.py).
_HFOV_FALLBACK_DEG = 91.0

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Gemini2DepthBridgeNode(Node):

    def __init__(self):
        super().__init__("gemini2_depth_bridge_node")

        self.declare_parameter("input_topic",      "/camera/depth/image_raw")
        self.declare_parameter("input_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("frame_id",         "camera_depth_frame")
        # 16UC1 mm -> meters. Gemini 2 default 1 mm/unit.
        self.declare_parameter("depth_scale",      0.001)
        # Pixels <= min_valid_m are considered INVALID (no-return / self-view).
        self.declare_parameter("min_valid_m",      0.15)
        # Replacement value for invalid pixels. 10.0 = "far/safe" (see docstring).
        self.declare_parameter("invalid_fill_m",   DEPTH_NORM_MAX_M)
        # Median kernel size for patching SMALL holes (0 = no patching).
        self.declare_parameter("hole_fill_px",     5)
        self.declare_parameter("publish_z16",      False)
        self.declare_parameter("stats_period_s",   2.0)

        self._in_topic   = str(self.get_parameter("input_topic").value)
        self._info_topic = str(self.get_parameter("input_info_topic").value)
        self._frame_id   = str(self.get_parameter("frame_id").value)
        self._scale      = float(self.get_parameter("depth_scale").value)
        self._min_valid  = float(self.get_parameter("min_valid_m").value)
        self._fill       = float(self.get_parameter("invalid_fill_m").value)
        self._hole_px    = int(self.get_parameter("hole_fill_px").value)
        self._pub_z16_on = bool(self.get_parameter("publish_z16").value)
        self._stats_dt   = float(self.get_parameter("stats_period_s").value)

        if self._hole_px > 0 and self._hole_px % 2 == 0:
            self._hole_px += 1   # cv2.medianBlur needs an odd kernel

        self._bridge = CvBridge()
        self._src_info = None       # original CameraInfo from the driver
        self._info_out = None       # CameraInfo scaled to 640x480
        self._n_frames = 0
        self._t_stats  = self.get_clock().now()

        self._pub_f32 = self.create_publisher(
            Image, "/realsense/depth/float32", _SENSOR_QOS)
        self._pub_info = self.create_publisher(
            CameraInfo, "/realsense/depth/camera_info", _SENSOR_QOS)
        self._pub_z16 = self.create_publisher(
            Image, "/realsense/depth/z16", _SENSOR_QOS) if self._pub_z16_on else None
        self._pub_stats = self.create_publisher(
            String, "/realsense/depth/stats", 10)

        self.create_subscription(
            CameraInfo, self._info_topic, self._on_info, _SENSOR_QOS)
        self.create_subscription(
            Image, self._in_topic, self._on_depth, _SENSOR_QOS)

        self.get_logger().info("=" * 62)
        self.get_logger().info("  Gemini2 depth bridge (real hardware)")
        self.get_logger().info("=" * 62)
        self.get_logger().info(f"  in   : {self._in_topic}")
        self.get_logger().info(f"  info : {self._info_topic}")
        self.get_logger().info(f"  out  : /realsense/depth/float32  ({_W}x{_H}, 32FC1 m)")
        self.get_logger().info(
            f"  invalid px (<= {self._min_valid:.2f} m) -> {self._fill:.1f} m "
            f"| hole fill {self._hole_px} px")
        self.get_logger().info("=" * 62)

    # ── camera_info: scaled once to 640x480 ──────────────────────────────────

    def _on_info(self, msg: CameraInfo):
        if self._src_info is not None and msg.width == self._src_info.width \
                and msg.height == self._src_info.height:
            return
        self._src_info = msg
        self._info_out = self._scale_info(msg)
        self.get_logger().info(
            f"[CAM] intrinsics {msg.width}x{msg.height} "
            f"fx={msg.k[0]:.1f} -> {_W}x{_H} fx={self._info_out.k[0]:.1f}")

    @staticmethod
    def _scale_info(src: CameraInfo) -> CameraInfo:
        """Intrinsics are scaled along with the image resize, otherwise the
        point cloud ends up metrically wrong (obstacles look wider/narrower)."""
        sx = float(_W) / float(src.width) if src.width else 1.0
        sy = float(_H) / float(src.height) if src.height else 1.0
        out = CameraInfo()
        out.width, out.height = _W, _H
        out.distortion_model = "plumb_bob"
        out.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        k = list(src.k)
        out.k = [k[0] * sx, 0.0, k[2] * sx,
                 0.0, k[4] * sy, k[5] * sy,
                 0.0, 0.0, 1.0]
        out.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        out.p = [out.k[0], 0.0, out.k[2], 0.0,
                 0.0, out.k[4], out.k[5], 0.0,
                 0.0, 0.0, 1.0, 0.0]
        return out

    def _fallback_info(self) -> CameraInfo:
        fx = _W / (2.0 * math.tan(math.radians(_HFOV_FALLBACK_DEG) / 2.0))
        info = CameraInfo()
        info.width, info.height = _W, _H
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, _W / 2.0, 0.0, fx, _H / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, _W / 2.0, 0.0, 0.0, fx, _H / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    # ── depth ────────────────────────────────────────────────────────────────

    def _on_depth(self, msg: Image):
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge: {exc}", throttle_duration_sec=5.0)
            return

        # 1. to meters (float32)
        if raw.dtype == np.uint16:
            depth = raw.astype(np.float32) * self._scale
        else:
            depth = np.asarray(raw, dtype=np.float32)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        # 2. resize to the model contract BEFORE patching (cheaper this way).
        #    INTER_NEAREST: linear interpolation would invent depth values
        #    "between" an obstacle's edge and the background — ghost obstacles.
        if depth.shape != (_H, _W):
            depth = cv2.resize(depth, (_W, _H), interpolation=cv2.INTER_NEAREST)

        invalid = depth <= self._min_valid
        n_invalid = int(invalid.sum())

        # 3. patch SMALL holes with the neighborhood median (wall value, not far)
        if self._hole_px > 0 and n_invalid:
            filled = cv2.medianBlur(depth, self._hole_px)
            small = invalid & (filled > self._min_valid)
            depth[small] = filled[small]
            invalid = depth <= self._min_valid

        # 4. remaining invalid pixels -> far/safe (see docstring, point 2)
        depth[invalid] = self._fill

        # 5. clip to the model's normalization range
        np.clip(depth, 0.0, DEPTH_NORM_MAX_M, out=depth)

        stamp = self.get_clock().now().to_msg()
        out = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        out.header.stamp    = stamp
        out.header.frame_id = self._frame_id
        self._pub_f32.publish(out)

        info = self._info_out if self._info_out is not None else self._fallback_info()
        info.header.stamp    = stamp
        info.header.frame_id = self._frame_id
        self._pub_info.publish(info)

        if self._pub_z16 is not None:
            z16 = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
            m16 = self._bridge.cv2_to_imgmsg(z16, encoding="16UC1")
            m16.header.stamp    = stamp
            m16.header.frame_id = self._frame_id
            self._pub_z16.publish(m16)

        self._n_frames += 1
        self._maybe_stats(depth, n_invalid)

    def _maybe_stats(self, depth, n_invalid):
        now = self.get_clock().now()
        dt = (now - self._t_stats).nanoseconds * 1e-9
        if dt < self._stats_dt:
            return
        self._t_stats = now
        valid = depth[(depth > self._min_valid) & (depth < DEPTH_NORM_MAX_M)]
        pct_valid = 100.0 * valid.size / float(depth.size)
        payload = {
            "hz":         round(self._n_frames / dt, 1),
            "valid_pct":  round(pct_valid, 1),
            "invalid_px": n_invalid,
            "min_m":      round(float(valid.min()), 2) if valid.size else None,
            "p50_m":      round(float(np.median(valid)), 2) if valid.size else None,
        }
        self._n_frames = 0
        self._pub_stats.publish(String(data=json.dumps(payload)))
        # Early warning: a "blind" camera (almost all pixels invalid) will
        # still produce a 10 m image that looks safe -> the drone flies
        # blind. Better to make noise in the terminal.
        if pct_valid < 20.0:
            self.get_logger().warn(
                f"[CAM] only {pct_valid:.0f}% of pixels valid — check exposure/"
                "surfaces/distance. Depth is being treated as 'far' everywhere else!",
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = Gemini2DepthBridgeNode()
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
