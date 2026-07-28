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

Input:
  /realsense/depth/float32      sensor_msgs/Image      (32FC1, meters)
  /realsense/depth/camera_info  sensor_msgs/CameraInfo
  /mavros/local_position/pose   geometry_msgs/PoseStamped   (altitude gate)

Output:
  /realsense/depth/points       sensor_msgs/PointCloud2
  /realsense/depth/gate_status  std_msgs/String   (JSON: open, alt, band)
"""

import json

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
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
        # Jendela konvergensi EKF sebelum ground di-latch. Samakan dengan
        # ekf_pre_wait_s di fm_inference_real_node.py.
        self.declare_parameter("gate_ground_settle_s", 10.0)

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

    def _on_info(self, msg: CameraInfo):
        self.cam_info = msg

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
                    f"[GATE] ground_z tidak wajar ({med:.2f} m) -> 0.0 "
                    "(sama seperti fm_inference_real_node)")
            self.get_logger().info(
                f"[GATE] ground_z latched = {self._ground_z:.2f} m "
                f"(median dari {len(self._z_samples)} pose / "
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
        stamp = self.get_clock().now().to_msg()

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

        # Skip empty clouds: octomap would shout "No data to copy" and there is no
        # point processing 0 points (e.g. when facing an open area).
        if len(points) == 0:
            return

        # FIX: stamp with THIS NODE's clock, not msg.header (which is Gazebo sim-time)
        frame_id = msg.header.frame_id
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
        msg.data       = points.tobytes()
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
