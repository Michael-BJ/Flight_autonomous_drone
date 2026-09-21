#!/usr/bin/env python3
"""A synthetic depth camera for the offline VFH missions.

Reads the fake PX4's TRUE pose (same process) and renders what a level,
front-facing Orbbec Gemini 2 (91 x 66 deg, 640x480) would see of a world
of vertical cylinders + the ground plane, exactly in the format
fm_deploy's gemini2_depth_bridge_node publishes: 32FC1 METERS, invalid =
10.0 m, plus the scaled CameraInfo. Also records the closest approach of
the drone to any cylinder surface (the collision metric)."""
import math

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

W, H = 640, 480
HFOV, VFOV = 91.0, 66.0
FX = W / (2.0 * math.tan(math.radians(HFOV) / 2.0))
FY = H / (2.0 * math.tan(math.radians(VFOV) / 2.0))
INVALID = 10.0

_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST, depth=1)


class DepthWorld(Node):
    def __init__(self, fake, cylinders, rate_hz=8.0, stop_after=None, noise_m=0.02):
        super().__init__("fake_depth_world")
        self.fake = fake
        self.cyl = [(float(x), float(y), float(r)) for x, y, r in cylinders]
        self.stop_after = stop_after       # seconds (fake.T) after which frames stop
        self.noise = noise_m
        self.rng = np.random.default_rng(1)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/realsense/depth/float32", _QOS)
        self.pub_info = self.create_publisher(CameraInfo, "/realsense/depth/camera_info", _QOS)
        self.n_frames = 0
        self.min_clearance = float("inf")   # drone centre to cylinder surface, airborne
        self.min_clearance_at = None
        self.tan_h = ((np.arange(W) - W / 2.0) / FX)          # + = right
        self.tan_v = ((np.arange(H) - H / 2.0) / FY)[:, None]  # + = down
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(0.1, self._clearance)

    def _clearance(self):
        f = self.fake
        if not (f.armed and f.z_true > 0.3):
            return
        for (cx, cy, r) in self.cyl:
            d = math.hypot(f.x - cx, f.y - cy) - r
            if d < self.min_clearance:
                self.min_clearance = d
                self.min_clearance_at = (round(f.T(), 1), round(f.x, 2), round(f.y, 2))

    def render(self, x, y, yaw, alt):
        # ray direction per column in the world frame (camera + = right = CW)
        ang = yaw - np.arctan(self.tan_h)
        dx, dy = np.cos(ang), np.sin(ang)
        cos_a = np.cos(np.arctan(self.tan_h))          # range -> z-depth
        rng_min = np.full(W, np.inf)
        for (cx, cy, r) in self.cyl:
            ox, oy = x - cx, y - cy
            b = ox * dx + oy * dy
            c = ox * ox + oy * oy - r * r
            disc = b * b - c
            hit = disc >= 0.0
            t = -b - np.sqrt(np.where(hit, disc, 0.0))
            ok = hit & (t > 0.05)
            rng_min = np.where(ok & (t < rng_min), t, rng_min)
        z_obst = rng_min * cos_a                                   # (W,)
        depth = np.repeat(z_obst[None, :], H, axis=0)
        with np.errstate(divide="ignore", invalid="ignore"):
            zg = np.where(self.tan_v > 1e-6, alt / self.tan_v, np.inf)   # (H,1)
        depth = np.minimum(depth, np.repeat(zg, W, axis=1))
        depth = np.where(np.isfinite(depth) & (depth < INVALID), depth, INVALID).astype(np.float32)
        if self.noise > 0.0:
            depth += self.rng.normal(0.0, self.noise, depth.shape).astype(np.float32)
        return depth

    def _tick(self):
        f = self.fake
        if self.stop_after is not None and f.T() >= self.stop_after:
            return
        depth = self.render(f.x, f.y, f.yaw, max(0.1, f.z_true + 0.1))   # camera 10 cm above skids
        stamp = self.get_clock().now().to_msg()
        msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        msg.header.stamp = stamp; msg.header.frame_id = "camera_depth_frame"
        self.pub.publish(msg)
        info = CameraInfo(); info.width, info.height = W, H
        info.k = [FX, 0.0, W / 2.0, 0.0, FY, H / 2.0, 0.0, 0.0, 1.0]
        info.header.stamp = stamp; info.header.frame_id = "camera_depth_frame"
        self.pub_info.publish(info)
        self.n_frames += 1
