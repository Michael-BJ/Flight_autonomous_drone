#!/usr/bin/env python3
"""
launch_common.py — pieces shared by vfh_all_barometer.launch.py and
vfh_perception.launch.py: the Orbbec Gemini 2 camera include (same
arguments/defaults as fm_deploy/fm_all.launch.py: DEPTH ONLY, publish_tf
false — see the comments there for why), fm_deploy's
gemini2_depth_bridge_node, the VFH perception node and the VFH flight
parameters. Importable from a launch file because this module is installed
with the python package.
"""
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _b(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _i(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


# ── Orbbec Gemini 2 (same as fm_deploy/fm_all.launch.py) ────────────────────

def camera_arguments():
    return [
        # DEPTH ONLY: color/IR on USB 2.0 starve the depth stream (silently).
        DeclareLaunchArgument("enable_color",       default_value="false"),
        DeclareLaunchArgument("enable_ir",          default_value="false"),
        DeclareLaunchArgument("enable_point_cloud", default_value="false"),
        DeclareLaunchArgument("depth_registration", default_value="false"),
        DeclareLaunchArgument("depth_width",  default_value="0"),
        DeclareLaunchArgument("depth_height", default_value="0"),
        DeclareLaunchArgument("depth_fps",    default_value="0"),
        # TF CONFLICT — keep false (fm_deploy README §5).
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="false"),
        DeclareLaunchArgument("enable_accel", default_value="false"),
        DeclareLaunchArgument("enable_gyro",  default_value="false"),
        # gemini2_depth_bridge_node (fm_deploy)
        DeclareLaunchArgument("depth_topic",      default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("depth_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("depth_scale",      default_value="0.001"),
        DeclareLaunchArgument(
            "invalid_fill_m", default_value="10.0",
            description="Depth given to invalid pixels (no return). 10 m = far/safe, "
                        "i.e. ignored by VFH (> d_max)."),
        DeclareLaunchArgument("hole_fill_px", default_value="5"),
    ]


def camera_include():
    share = get_package_share_directory("orbbec_camera")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{share}/launch/gemini2.launch.py"),
        launch_arguments={
            "enable_color":       LaunchConfiguration("enable_color"),
            "enable_ir":          LaunchConfiguration("enable_ir"),
            "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
            "depth_registration": LaunchConfiguration("depth_registration"),
            "depth_width":        LaunchConfiguration("depth_width"),
            "depth_height":       LaunchConfiguration("depth_height"),
            "depth_fps":          LaunchConfiguration("depth_fps"),
            "publish_tf":         LaunchConfiguration("publish_tf"),
            "enable_sync_output_accel_gyro":
                LaunchConfiguration("enable_sync_output_accel_gyro"),
            "enable_accel":       LaunchConfiguration("enable_accel"),
            "enable_gyro":        LaunchConfiguration("enable_gyro"),
        }.items(),
    )


def depth_bridge_node():
    return Node(
        package="fm_deploy",
        executable="gemini2_depth_bridge_node",
        name="gemini2_depth_bridge_node",
        output="screen",
        parameters=[{
            "input_topic":      LaunchConfiguration("depth_topic"),
            "input_info_topic": LaunchConfiguration("depth_info_topic"),
            "frame_id":         "camera_depth_frame",
            "depth_scale":      _f("depth_scale"),
            "invalid_fill_m":   _f("invalid_fill_m"),
            "hole_fill_px":     _i("hole_fill_px"),
        }],
    )


# ── VFH perception node ─────────────────────────────────────────────────────

def vfh_arguments():
    return [
        DeclareLaunchArgument("n_sectors",        default_value="36",
                              description="Angular sectors over 360 deg (36 = 10 deg each)."),
        DeclareLaunchArgument("threshold",        default_value="0.45",
                              description="Normalised density above which a sector is blocked."),
        DeclareLaunchArgument("smooth_window",    default_value="2"),
        DeclareLaunchArgument("d_max",            default_value="5000.0",
                              description="mm; obstacles farther than this are ignored."),
        DeclareLaunchArgument("d_min",            default_value="300.0",
                              description="mm; readings closer than this are ignored (noise/self)."),
        DeclareLaunchArgument("min_valley_width", default_value="3",
                              description="Free sectors needed for a passable gap (3 = 30 deg)."),
        DeclareLaunchArgument("robot_radius",     default_value="346.0",
                              description="mm; 49x49 cm frame -> half diagonal."),
        DeclareLaunchArgument("safety_dist",      default_value="500.0",
                              description="mm; extra clearance added to robot_radius."),
        DeclareLaunchArgument("hfov_deg",         default_value="91.0",
                              description="Gemini 2 horizontal FOV; replaced by camera_info."),
        DeclareLaunchArgument("use_camera_info",  default_value="true"),
        DeclareLaunchArgument("frame_skip",       default_value="1"),
        DeclareLaunchArgument("publish_visual",   default_value="true",
                              description="Publish /vfh/visual (bgr8 overlay) for rqt_image_view."),
        DeclareLaunchArgument("show_debug",       default_value="false",
                              description="Open a local OpenCV window (needs a display)."),
        DeclareLaunchArgument("roi_top",          default_value="0.15",
                              description="Top of the vertical ROI (fraction of the image height)."),
        DeclareLaunchArgument("roi_bottom",       default_value="0.85"),
        DeclareLaunchArgument("ground_filter",    default_value="true",
                              description="Drop pixels on the ground using the barometric AGL "
                                          "from the flight node (/vfh/agl_m)."),
        DeclareLaunchArgument("ground_margin_m",  default_value="0.5",
                              description="m; anything lower than agl - this is ground."),
    ]


def vfh_node():
    return Node(
        package="vfh_avoidance_barometer",
        executable="vfh_avoidance_node",
        name="vfh_avoidance_node",
        output="screen",
        parameters=[{
            "n_sectors":        _i("n_sectors"),
            "threshold":        _f("threshold"),
            "smooth_window":    _i("smooth_window"),
            "d_max":            _f("d_max"),
            "d_min":            _f("d_min"),
            "min_valley_width": _i("min_valley_width"),
            "robot_radius":     _f("robot_radius"),
            "safety_dist":      _f("safety_dist"),
            "hfov_deg":         _f("hfov_deg"),
            "use_camera_info":  _b("use_camera_info"),
            "frame_skip":       _i("frame_skip"),
            "publish_visual":   _b("publish_visual"),
            "show_debug":       _b("show_debug"),
            "roi_top":          _f("roi_top"),
            "roi_bottom":       _f("roi_bottom"),
            "ground_filter":    _b("ground_filter"),
            "ground_margin_m":  _f("ground_margin_m"),
            "depth_topic":      "/realsense/depth/float32",
            "camera_info_topic": "/realsense/depth/camera_info",
            "depth_source":     "float32",
        }],
    )


# ── VFH flight node (vfh_flight_baro_node) ──────────────────────────────────

def vfh_flight_arguments():
    return [
        DeclareLaunchArgument(
            "goal_dist", default_value="3.0",
            description="Goal = this many metres ahead of the launch point along the "
                        "heading the nose points at when the mission starts (max 30)."),
        DeclareLaunchArgument("goal_tol", default_value="0.5",
                              description="m; the drone within this of the goal = reached."),
        DeclareLaunchArgument("yaw_rate_max_deg", default_value="30.0",
                              description="deg/s; yaw setpoint slew rate toward the VFH heading."),
        DeclareLaunchArgument("move_heading_tol_deg", default_value="25.0",
                              description="The XY setpoint only advances while the drone's heading "
                                          "is within this of the yaw setpoint (camera looks ahead)."),
        DeclareLaunchArgument("max_lead", default_value="0.6",
                              description="m; the setpoint stops advancing when it leads the drone "
                                          "by more than this (must be <= max_pos_error)."),
        DeclareLaunchArgument("stop_dist", default_value="1.2",
                              description="m; closest valid depth within +-12 deg of the axis below "
                                          "this -> XY frozen (hard brake), yaw still turns."),
        DeclareLaunchArgument("vfh_stale_s", default_value="1.0",
                              description="s; no /vfh/movement_direction for this long -> XY frozen."),
        DeclareLaunchArgument("vfh_stale_abort_s", default_value="5.0",
                              description="s; no VFH message for this long -> leg ends, land here."),
        DeclareLaunchArgument("blocked_timeout_s", default_value="15.0",
                              description="s; no progress toward the goal for this long (blocked, "
                                          "oscillating, stuck) -> land here."),
        DeclareLaunchArgument("goal_fov_half_deg", default_value="80.0",
                              description="deg; goal farther off the nose than this is outside the "
                                          "camera view -> XY frozen, turn toward it first."),
        DeclareLaunchArgument("mission_timeout_s", default_value="120.0",
                              description="s; leg not finished by then -> land here."),
        DeclareLaunchArgument("geofence_margin", default_value="2.0",
                              description="m; farther than goal_dist + this from the launch point "
                                          "-> sanity abort (AUTO.LAND)."),
        DeclareLaunchArgument("require_vfh", default_value="true",
                              description="Refuse to ARM until the VFH stream is alive."),
        DeclareLaunchArgument("vfh_min_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("vfh_wait_timeout_s", default_value="60.0"),
    ]


def vfh_flight_parameters():
    return {
        "goal_dist":            _f("goal_dist"),
        "goal_tol":             _f("goal_tol"),
        "yaw_rate_max_deg":     _f("yaw_rate_max_deg"),
        "move_heading_tol_deg": _f("move_heading_tol_deg"),
        "max_lead":             _f("max_lead"),
        "stop_dist":            _f("stop_dist"),
        "vfh_stale_s":          _f("vfh_stale_s"),
        "vfh_stale_abort_s":    _f("vfh_stale_abort_s"),
        "blocked_timeout_s":    _f("blocked_timeout_s"),
        "goal_fov_half_deg":    _f("goal_fov_half_deg"),
        "mission_timeout_s":    _f("mission_timeout_s"),
        "geofence_margin":      _f("geofence_margin"),
        "require_vfh":          _b("require_vfh"),
        "vfh_min_rate_hz":      _f("vfh_min_rate_hz"),
        "vfh_wait_timeout_s":   _f("vfh_wait_timeout_s"),
    }
