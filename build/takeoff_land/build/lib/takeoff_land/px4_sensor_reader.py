#!/usr/bin/env python3
"""
px4_sensor_reader.py
==================================================================
Telemetry bridge: PX4 -> ROS2.
Subscribes to MAVROS topics and publishes a compact JSON snapshot:

    /px4/state    (std_msgs/String, JSON) — FCU status (connected/armed/mode)
    /px4/sensors  (std_msgs/String, JSON) — sensor snapshot + local_x/y/z, yaw

takeoff_land_node reads BOTH topics (not MAVROS directly), so there is
only ONE MAVROS subscriber in the system — avoiding QoS duplication.

Connection: Jetson Orin Nano <-> PX4 via SERIAL (e.g. /dev/ttyTHS1 or /dev/ttyACM0).

PARAMS:
    fcu_url            : FCU URL (default: auto-detect serial, fallback SITL UDP)
    auto_launch_mavros : True -> spawn MAVROS if not already running.
                         Set False when MAVROS is managed by a launch file (recommended).
"""

import glob
import json
import math
import os
import signal
import subprocess
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import (
    Imu, MagneticField, FluidPressure, Temperature, NavSatFix, BatteryState,
)
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, Altitude, StatusText, HomePosition
from std_msgs.msg import String


# ── Auto-detect FCU URL: serial if available, fallback to SITL UDP ────────────
def detect_fcu_url() -> str:
    candidates = sorted(glob.glob("/dev/ttyTHS*")) \
        + sorted(glob.glob("/dev/ttyACM*")) \
        + sorted(glob.glob("/dev/ttyUSB*"))
    if candidates:
        # Jetson UART typically 921600; USB CDC ACM 57600.
        baud = "921600" if candidates[0].startswith("/dev/ttyTHS") else "57600"
        return "{0}:{1}".format(candidates[0], baud)
    return "udp://:14540@127.0.0.1:14580"  # SITL default


def is_mavros_active() -> bool:
    for topic in ("/mavros/state", "/uas1/state"):
        try:
            result = subprocess.run(
                ["ros2", "topic", "info", topic],
                capture_output=True, text=True, timeout=3,
            )
            out = result.stdout
            if "Publisher count:" in out and "Publisher count: 0" not in out:
                return True
        except Exception:
            pass
    return False


# ── QoS ───────────────────────────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def quaternion_to_euler_deg(q) -> Tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def safe_float_str(value, fmt=".2f", unit="", fallback="N/A") -> str:
    try:
        if math.isnan(value) or math.isinf(value):
            return fallback
        return ("{0:" + fmt + "}{1}").format(value, unit)
    except (TypeError, ValueError):
        return fallback


class PX4SensorReader(Node):

    def __init__(self):
        super().__init__("px4_sensor_reader")

        self.declare_parameter("fcu_url", detect_fcu_url())
        fcu_url = self.get_parameter("fcu_url").get_parameter_value().string_value

        self.declare_parameter("auto_launch_mavros", True)
        auto_launch_mavros = self.get_parameter(
            "auto_launch_mavros").get_parameter_value().bool_value

        # -- sensor data storage --
        self.imu_data        = None  # type: Optional[Imu]
        self.mag_data        = None  # type: Optional[MagneticField]
        self.gps_data        = None  # type: Optional[NavSatFix]
        self.local_pose      = None  # type: Optional[PoseStamped]
        self.local_velocity  = None  # type: Optional[TwistStamped]
        self.altitude_data   = None  # type: Optional[Altitude]
        self.battery_status  = None  # type: Optional[BatteryState]
        self.vehicle_state   = None  # type: Optional[State]
        self.baro_pressure   = None  # type: Optional[FluidPressure]
        self.imu_temperature = None  # type: Optional[Temperature]
        self.home_position   = None  # type: Optional[HomePosition]
        self._last_px4_status = ""

        # -- MAVROS process (if auto-launch) --
        self._mavros_proc     = None  # type: Optional[subprocess.Popen]
        self._mavros_log_file = None
        if auto_launch_mavros:
            self._launch_mavros_if_needed(fcu_url)
        else:
            self.get_logger().info("auto_launch_mavros=False — MAVROS managed externally.")

        # -- JSON publishers --
        self._pub_state   = self.create_publisher(String, "/px4/state",   10)
        self._pub_sensors = self.create_publisher(String, "/px4/sensors", 10)

        # -- MAVROS subscribers --
        self.create_subscription(Imu,           "/mavros/imu/data",                      self._cb_imu,           SENSOR_QOS)
        self.create_subscription(MagneticField, "/mavros/imu/mag",                       self._cb_mag,           SENSOR_QOS)
        self.create_subscription(FluidPressure, "/mavros/imu/static_pressure",           self._cb_baro,          SENSOR_QOS)
        self.create_subscription(Temperature,   "/mavros/imu/temperature_imu",           self._cb_temperature,   SENSOR_QOS)
        self.create_subscription(NavSatFix,     "/mavros/global_position/global",        self._cb_gps,           SENSOR_QOS)
        self.create_subscription(PoseStamped,   "/mavros/local_position/pose",           self._cb_local_pose,    SENSOR_QOS)
        self.create_subscription(TwistStamped,  "/mavros/local_position/velocity_local", self._cb_local_velocity,SENSOR_QOS)
        self.create_subscription(BatteryState,  "/mavros/battery",                       self._cb_battery,       SENSOR_QOS)
        self.create_subscription(State,         "/mavros/state",                         self._cb_state,         STATE_QOS)
        self.create_subscription(Altitude,      "/mavros/altitude",                      self._cb_altitude,      SENSOR_QOS)
        self.create_subscription(HomePosition,  "/mavros/home_position/home",            self._cb_home,          STATE_QOS)
        self.create_subscription(StatusText,    "/mavros/statustext/recv",               self._cb_statustext,    SENSOR_QOS)

        # -- timers --
        self.create_timer(1.0, self._timer_log)      # terminal log at 1 Hz
        self.create_timer(0.1, self._timer_publish)  # publish /px4/* at 10 Hz

        self.get_logger().info("PX4SensorReader ready. Publishing: /px4/state  /px4/sensors")

    # -- MAVROS launch (optional) ---------------------------------------------
    def _launch_mavros_if_needed(self, fcu_url: str):
        if is_mavros_active():
            self.get_logger().info("MAVROS already running, skipping auto-launch.")
            return
        self.get_logger().info("Auto-launching MAVROS  FCU: {0}".format(fcu_url))
        log_path = os.path.expanduser("~/.ros/mavros_auto.log")
        self._mavros_log_file = open(log_path, "w")
        self._mavros_proc = subprocess.Popen(
            ["ros2", "launch", "mavros", "px4.launch", "fcu_url:={0}".format(fcu_url)],
            stdout=self._mavros_log_file, stderr=self._mavros_log_file,
            preexec_fn=os.setsid,
        )
        self.get_logger().info("MAVROS PID={0}".format(self._mavros_proc.pid))
        time.sleep(4)

    def destroy_node(self):
        if self._mavros_proc is not None and self._mavros_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._mavros_proc.pid), signal.SIGTERM)
                self._mavros_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self._mavros_proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except ProcessLookupError:
                pass
        if self._mavros_log_file is not None:
            try:
                self._mavros_log_file.close()
            except Exception:
                pass
        super().destroy_node()

    # -- sensor callbacks -----------------------------------------------------
    def _cb_imu(self, msg):            self.imu_data        = msg
    def _cb_mag(self, msg):            self.mag_data        = msg
    def _cb_baro(self, msg):           self.baro_pressure   = msg
    def _cb_temperature(self, msg):    self.imu_temperature = msg
    def _cb_gps(self, msg):            self.gps_data        = msg
    def _cb_local_pose(self, msg):     self.local_pose      = msg
    def _cb_local_velocity(self, msg): self.local_velocity  = msg
    def _cb_battery(self, msg):        self.battery_status  = msg
    def _cb_altitude(self, msg):       self.altitude_data   = msg
    def _cb_home(self, msg):           self.home_position   = msg

    def _cb_statustext(self, msg):
        self._last_px4_status = msg.text
        if msg.severity <= 4:
            self.get_logger().warn("[PX4] {0}".format(msg.text))
        else:
            self.get_logger().info("[PX4] {0}".format(msg.text))

    def _cb_state(self, msg):
        prev = self.vehicle_state
        self.vehicle_state = msg
        if prev is None or prev.connected != msg.connected:
            self.get_logger().info("MAVROS {0}".format(
                "CONNECTED" if msg.connected else "DISCONNECTED"))
        if prev is None or prev.armed != msg.armed:
            self.get_logger().info("PX4 {0}  mode={1}".format(
                "ARMED" if msg.armed else "DISARMED", msg.mode))
        if prev is None or prev.mode != msg.mode:
            self.get_logger().info("PX4 mode -> {0}".format(msg.mode))
        self._publish_state()

    # -- publishers -----------------------------------------------------------
    def _publish_state(self):
        if self.vehicle_state is None:
            return
        state_dict = {
            "connected": self.vehicle_state.connected,
            "armed":     self.vehicle_state.armed,
            "mode":      self.vehicle_state.mode,
            "guided":    self.vehicle_state.guided,
            "manual":    self.vehicle_state.manual_input,
        }
        msg = String(); msg.data = json.dumps(state_dict)
        self._pub_state.publish(msg)

    def _timer_publish(self):
        self._publish_state()
        snapshot = self.get_sensor_snapshot()
        if snapshot:
            msg = String(); msg.data = json.dumps(snapshot)
            self._pub_sensors.publish(msg)

    # -- terminal log at 1 Hz -------------------------------------------------
    def _timer_log(self):
        sep = "=" * 60
        if self.vehicle_state is None or not self.vehicle_state.connected:
            self.get_logger().warn("Waiting for MAVROS connection...")
            return
        self.get_logger().info(sep)
        self.get_logger().info("Mode: {0}  Armed: {1}  Guided: {2}".format(
            self.vehicle_state.mode, self.vehicle_state.armed, self.vehicle_state.guided))

        if self.imu_data:
            r, p, y = quaternion_to_euler_deg(self.imu_data.orientation)
            warn = "  WARNING: NOT LEVEL" if (abs(r) > 10 or abs(p) > 10) else ""
            self.get_logger().info(
                "[IMU]  Roll={0:+.1f}  Pitch={1:+.1f}  Yaw={2:+.1f} deg{3}".format(r, p, y, warn))

        if self.local_pose:
            pp = self.local_pose.pose.position
            self.get_logger().info(
                "[POS]  x={0:.3f}m  y={1:.3f}m  z={2:.3f}m  (ENU local)".format(pp.x, pp.y, pp.z))

        if self.gps_data:
            fix_map = {-1: "NO FIX", 0: "FIX", 1: "SBAS", 2: "GBAS"}
            self.get_logger().info("[GPS]  Lat={0:.7f}  Lon={1:.7f}  Alt={2:.2f}m  Fix={3}".format(
                self.gps_data.latitude, self.gps_data.longitude, self.gps_data.altitude,
                fix_map.get(self.gps_data.status.status, "?")))
        else:
            self.get_logger().warn("[GPS]  No data (indoor? -> needs VIO/optical-flow for OFFBOARD)")

        if self.local_velocity:
            v = self.local_velocity.twist.linear
            spd = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            self.get_logger().info(
                "[VEL]  vx={0:+.2f}  vy={1:+.2f}  vz={2:+.2f}  speed={3:.2f} m/s".format(v.x, v.y, v.z, spd))

        if self.altitude_data:
            self.get_logger().info("[ALT]  AMSL={0}  Rel={1}  Terrain={2}".format(
                safe_float_str(self.altitude_data.amsl,     unit="m"),
                safe_float_str(self.altitude_data.relative, unit="m"),
                safe_float_str(self.altitude_data.terrain,  unit="m")))

        if self.battery_status:
            raw = self.battery_status.percentage
            if raw < 0:
                self.get_logger().info("[BAT]  No battery data")
            else:
                pct = raw * 100.0
                warn = "  WARNING: LOW BATTERY" if pct < 20.0 else ""
                self.get_logger().info("[BAT]  {0:.2f}V  {1:.1f}%{2}".format(
                    self.battery_status.voltage, pct, warn))

        if self._last_px4_status:
            self.get_logger().info("[PX4]  {0}".format(self._last_px4_status))
        self.get_logger().info(sep)

    # -- sensor snapshot ------------------------------------------------------
    def get_sensor_snapshot(self) -> dict:
        snapshot = {
            "connected": self.vehicle_state.connected if self.vehicle_state else False,
            "mode":      self.vehicle_state.mode      if self.vehicle_state else "UNKNOWN",
            "armed":     self.vehicle_state.armed     if self.vehicle_state else False,
        }
        if self.imu_data:
            r, p, y = quaternion_to_euler_deg(self.imu_data.orientation)
            snapshot["imu"] = {"roll_deg": r, "pitch_deg": p, "yaw_deg": y}

        if self.gps_data:
            snapshot["gps"] = {
                "latitude": self.gps_data.latitude,
                "longitude": self.gps_data.longitude,
                "altitude_m": self.gps_data.altitude,
                "fix_status": self.gps_data.status.status,
            }
        if self.altitude_data:
            terrain = self.altitude_data.terrain
            snapshot["altitude"] = {
                "amsl_m": self.altitude_data.amsl,
                "relative_m": self.altitude_data.relative,
                "terrain_m": None if math.isnan(terrain) else terrain,
            }
        if self.local_velocity:
            v = self.local_velocity.twist.linear
            snapshot["velocity"] = {
                "vx": v.x, "vy": v.y, "vz": v.z,
                "speed": math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z),
            }
            snapshot["vel_x"] = v.x
            snapshot["vel_y"] = v.y
            snapshot["vel_z"] = v.z

        # local position 3D + yaw — required by takeoff_land_node
        if self.local_pose:
            pp = self.local_pose.pose.position
            snapshot["local_x"] = pp.x
            snapshot["local_y"] = pp.y
            snapshot["local_z"] = pp.z
            q = self.local_pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            snapshot["yaw"] = math.atan2(siny_cosp, cosy_cosp)  # rad

        if self.battery_status:
            raw = self.battery_status.percentage
            snapshot["battery"] = {
                "voltage_V": self.battery_status.voltage,
                "percentage_display": None if raw < 0.0 else round(raw * 100.0, 1),
            }
        return snapshot


def main(args=None):
    rclpy.init(args=args)
    node = PX4SensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
