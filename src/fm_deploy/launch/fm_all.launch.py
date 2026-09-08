#!/usr/bin/env python3
"""
fm_all.launch.py
=================
One terminal for everything: MAVROS + Orbbec Gemini 2 camera driver +
the whole fm_deploy stack (bridge, pointcloud, octomap, FM inference).
Does not run RViz — use `rviz2` in a separate terminal if needed.

Combined equivalent of:
    T1  ros2 launch fm_deploy mavros_only.launch.py fcu_url:=...
    T2  ros2 launch orbbec_camera gemini2.launch.py
    T3  ros2 launch fm_deploy fm_real.launch.py model_path:=... dry_run:=true

Start order is enforced via TimerAction (MAVROS -> camera -> fm stack) so
logs don't collide and `_kill_stale` in mavros_only.launch.py has time to
finish before other nodes come up. This isn't a hard ROS requirement
(topics/TF are already late-binding — a node that starts first just waits
for data), it just keeps the log readable and matches the documented order.

Only the parameters that get changed most often (README §5) are exposed
here. Other fm_real.launch.py parameters (planner guards, octomap
resolution, etc.) still use their original defaults — run fm_real.launch.py
directly if you need to tune those.

Example (model_path now has a .onnx default, may be omitted):
    ros2 launch fm_deploy fm_all.launch.py \\
        fcu_url:=/dev/ttyACM0:57600 \\
        dry_run:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_FM_DEPLOY_SHARE = get_package_share_directory("fm_deploy")
_ORBBEC_SHARE = get_package_share_directory("orbbec_camera")

# Default model_path: .onnx, not .pth. This Jetson's GPU still fails with
# CUBLAS_STATUS_ALLOC_FAILED (a JetPack 6.2/CUDA 12.6 driver bug, no fix
# as of 2026-08-10 — see the onnxruntime-gpu-jetson memory), so .pth
# ALWAYS falls back to .onnx-CPU via the failed-GPU-attempt path (wastes
# ~2s + confusing error logs). Point straight at .onnx here so the node
# skips that GPU attempt and goes straight to CPUExecutionProvider — same
# end result, faster start, cleaner logs. K stays locked to 8 on this
# backend either way (see fm_inference_node.py), so nothing is lost by
# defaulting here. Override model_path:=...pth on the command line once
# the GPU is fixed and you want to re-test it.
_DEFAULT_MODEL_PATH = os.path.expanduser(
    "~/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037.onnx")


def generate_launch_description():
    args = [
        # ── Communication ────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "fcu_url", default_value="/dev/ttyACM0:57600",
            description="FCU URL — USB: /dev/ttyACM0:57600 | Jetson UART: /dev/ttyTHS1:921600"),

        # ── Model ────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "model_path", default_value=_DEFAULT_MODEL_PATH,
            description="FM checkpoint path (.onnx or .pth). Defaults to "
                        ".onnx — see the _DEFAULT_MODEL_PATH comment above."),
        DeclareLaunchArgument("K", default_value="8"),
        DeclareLaunchArgument(
            "onnx_fallback_on_oom", default_value="true",
            description="true = if .pth fails to load due to CUDA "
                        "out-of-memory, automatically switch to .onnx (K "
                        "forced to 8) instead of crashing the node."),
        DeclareLaunchArgument("onnx_fallback_path", default_value=""),

        # ── Mission ──────────────────────────────────────────────────────────
        DeclareLaunchArgument("goal_dist",  default_value="5.0"),
        DeclareLaunchArgument("goal_lat",   default_value="0.0"),
        DeclareLaunchArgument("target_alt", default_value="1.5"),
        DeclareLaunchArgument("v_max",      default_value="0.5"),
        DeclareLaunchArgument("mission_timeout_s", default_value="180.0"),
        DeclareLaunchArgument(
            "hover_settle_s", default_value="5.0",
            description="Seconds hovering over home after reaching cruise "
                        "altitude, before the octomap is wiped and the FM "
                        "loop starts."),

        # ── Terminal monitoring ──────────────────────────────────────────────
        DeclareLaunchArgument("status_period_s",     default_value="2.0"),
        DeclareLaunchArgument("color_output",        default_value="true"),
        DeclareLaunchArgument("warn_replan_overrun", default_value="true"),

        # ── Safety parameters that used to be LOCKED here ────────────────────
        # These are declared in fm_real.launch.py but were not forwarded, so
        # passing them to fm_all did NOTHING — ros2 launch ignores an
        # undeclared argument silently, with no error and no warning. That is
        # the worst possible failure mode for a safety switch: the operator
        # believes a guard is on when it is not. Every default below is copied
        # verbatim from fm_real.launch.py, so behaviour is unchanged unless
        # you explicitly pass one.
        DeclareLaunchArgument(
            "use_safety_guards", default_value="false",
            description="TRUE = escape maneuvers ON (guard fires -> the drone "
                        "flies a 0.4 m/s escape). FALSE = guards still fire "
                        "but only invalidate + hover. FALSE is the 'fair' "
                        "ablation mode that matches simulation, so keep it "
                        "false if real-flight numbers must stay comparable."),
        DeclareLaunchArgument(
            "use_lookahead_guard", default_value="true",
            description="Keeps guards 1 & 2 alive while use_safety_guards is "
                        "false. If BOTH are false there is NO guard at all."),
        DeclareLaunchArgument("guard_clearance",    default_value="0.60"),
        DeclareLaunchArgument("hard_clearance",     default_value="0.55"),
        DeclareLaunchArgument("collision_cost_tol", default_value="20.0"),
        DeclareLaunchArgument("max_plan_cost",      default_value="50.0"),
        DeclareLaunchArgument("planning_time_ahead", default_value="0.3"),
        DeclareLaunchArgument("speed_margin_k",     default_value="0.25"),
        DeclareLaunchArgument("use_speed_limit",    default_value="true"),
        DeclareLaunchArgument("blind_abort_s",      default_value="10.0"),
        DeclareLaunchArgument("stuck_abort_s",      default_value="60.0"),
        DeclareLaunchArgument("depth_max_lag",      default_value="0.5"),
        DeclareLaunchArgument(
            "min_battery_pct", default_value="0.0",
            description="0 = battery percentage check DISABLED."),
        DeclareLaunchArgument("rc_override_enabled",      default_value="true"),
        DeclareLaunchArgument("verify_rc_override_param", default_value="true"),
        DeclareLaunchArgument(
            "write_px4_params", default_value="false",
            description="Must be TRUE for px4_vel_cap to be applied at all. "
                        "While false, PX4 keeps its stock MPC_XY_VEL_MAX "
                        "(12 m/s) no matter what px4_vel_cap says."),
        DeclareLaunchArgument("px4_vel_cap",        default_value="2.0"),
        DeclareLaunchArgument("descent_speed",      default_value="0.3"),
        DeclareLaunchArgument("land_handoff_alt",   default_value="0.25"),
        DeclareLaunchArgument("auto_land_mode",     default_value="true"),
        DeclareLaunchArgument("min_fix_type",       default_value="3"),
        DeclareLaunchArgument("gps_wait_timeout",   default_value="120.0"),
        DeclareLaunchArgument("gps_stable_dur",     default_value="5.0"),
        DeclareLaunchArgument("ekf_window_s",       default_value="5.0"),
        DeclareLaunchArgument("max_ground_drift",   default_value="0.20"),

        # ── Safety ───────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="DEFAULT TRUE. Only set false after a clean dry run."),
        DeclareLaunchArgument(
            "safe_dis", default_value="0.8",
            description="Planner's soft preference (m from drone CENTER). "
                        "The planner prefers paths >= this value, but it's "
                        "not a hard limit (see guard_clearance/hard_clearance "
                        "for that). TEMPORARILY raised from the code default "
                        "0.65 -> 0.8 m per the 2026-07-28 test request."),
        DeclareLaunchArgument("fence_fwd",     default_value="10.0"),
        DeclareLaunchArgument("fence_back",    default_value="3.0"),
        DeclareLaunchArgument("fence_lat",     default_value="4.0"),
        DeclareLaunchArgument("max_home_dist", default_value="12.0"),
        DeclareLaunchArgument(
            "max_alt_error", default_value="0.5",
            description="Max z deviation from cruise_z before ABORT (m)."),
        DeclareLaunchArgument("min_battery_v", default_value="0.0"),
        # ── GPS quality pre-arm gate ─────────────────────────────────────────
        # Refuses to ARM until GPS is genuinely usable. Skipped automatically
        # in dry_run (stage 1 runs on a bench indoors). Added after the
        # 2026-08-27 incident — see _wait_gps_quality() in
        # fm_inference_real_node.py for the full story.
        DeclareLaunchArgument(
            "require_gps", default_value="true",
            description="false ONLY for indoor flight with a healthy "
                        "VIO/optical-flow source feeding PX4 local position."),
        DeclareLaunchArgument("min_satellites", default_value="8"),
        DeclareLaunchArgument("max_hdop",       default_value="2.0"),
        # Max |local-frame z| accepted while ON THE GROUND. A steady but
        # far-from-zero z means the estimate is broken, not stable — on
        # 2026-08-27 a std-only check accepted 5.46 m for a grounded drone.
        DeclareLaunchArgument("max_ground_z",   default_value="1.0"),
        DeclareLaunchArgument(
            "ekf_pre_wait_s", default_value="10.0",
            description="Initial GPS/EKF convergence delay before the "
                        "ground_z std-check (same as takeoff_land_node.py)."),

        # ── Camera mount on the drone body — MEASURE IT YOURSELF (see README §4/§6) ──
        DeclareLaunchArgument("cam_x", default_value="0.10"),
        DeclareLaunchArgument("cam_y", default_value="0.00"),
        DeclareLaunchArgument("cam_z", default_value="-0.05"),

        # ── Camera stream — DEPTH ONLY ───────────────────────────────────────
        # The pipeline's only input is depth (/camera/depth/image_raw);
        # no node uses color or IR. If all three are enabled (gemini2.launch.py
        # default) on a USB 2.0 port, color 1280x720@30 MJPG + IR 1280x800@10
        # eat the bandwidth first and the DEPTH stream FAILS TO START — the
        # driver gives no error, it just stays silent. The result: octomap
        # never fills in, ESDF never becomes ready, and fm_inference_real_node
        # stalls at "Timeout depth/ESDF" after 90s.
        # Verification: depth-only -> /camera/depth/image_raw ~8 Hz.
        # Only re-enable these for visual debugging, and preferably on a
        # USB 3.0 port.
        DeclareLaunchArgument("enable_color",       default_value="false"),
        DeclareLaunchArgument("enable_ir",          default_value="false"),
        # Point cloud is built by depth_to_pointcloud_node, not the driver.
        DeclareLaunchArgument("enable_point_cloud", default_value="false"),
        # Without color, depth->color alignment is pointless. Disabling it
        # also keeps depth in its own sensor frame, consistent with the
        # base_link -> camera_depth_frame TF this stack uses.
        DeclareLaunchArgument("depth_registration", default_value="false"),
        # 0 = let the driver choose (verified 1280x800@10 Y16). Lower this
        # if depth latency over USB 2.0 is still too high.
        DeclareLaunchArgument("depth_width",  default_value="0"),
        DeclareLaunchArgument("depth_height", default_value="0"),
        DeclareLaunchArgument("depth_fps",    default_value="0"),
        # TF CONFLICT — DO NOT enable. The Orbbec driver defaults to
        # publish_tf:=true and publishes `camera_link -> camera_depth_frame`
        # on /tf_static. fm_real.launch.py also publishes
        # `base_link -> camera_depth_frame` on /tf_static. One child frame
        # with TWO parents: since both are latched, whichever parent wins in
        # each subscriber's buffer depends on message arrival order — not
        # deterministic. If camera_link wins, the odom->camera_depth_frame
        # lookup fails, octomap drops every cloud, /projected_map never
        # publishes, and fm_inference_real_node stalls at "Timeout
        # depth/ESDF" (the symptom in README §5).
        # Verified 2026-07-28: publish_tf:=false -> camera /tf_static is
        # empty, depth stays at 9.2 Hz.
        DeclareLaunchArgument("publish_tf", default_value="false"),
        # Accel/gyro already default to false in gemini2.launch.py, BUT
        # enable_sync_output_accel_gyro defaults to true and still publishes
        # /camera/accel/imu_info, /camera/gyro/imu_info and
        # /camera/gyro_accel/sample. No node in this stack uses them — the
        # pipeline only needs depth.
        DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="false"),
        DeclareLaunchArgument("enable_accel", default_value="false"),
        DeclareLaunchArgument("enable_gyro",  default_value="false"),

        # ── Depth-processing altitude gate (see fm_real.launch.py) ───────────
        DeclareLaunchArgument("gate_enabled",    default_value="true"),
        DeclareLaunchArgument("gate_alt_margin", default_value="0.5"),
    ]

    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{_FM_DEPLOY_SHARE}/launch/mavros_only.launch.py"),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
        }.items(),
    )

    camera = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{_ORBBEC_SHARE}/launch/gemini2.launch.py"),
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
            ),
        ],
    )

    fm_stack = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{_FM_DEPLOY_SHARE}/launch/fm_real.launch.py"),
                launch_arguments={
                    "model_path":        LaunchConfiguration("model_path"),
                    "K":                 LaunchConfiguration("K"),
                    "onnx_fallback_on_oom": LaunchConfiguration("onnx_fallback_on_oom"),
                    "onnx_fallback_path": LaunchConfiguration("onnx_fallback_path"),
                    "goal_dist":         LaunchConfiguration("goal_dist"),
                    "goal_lat":          LaunchConfiguration("goal_lat"),
                    "target_alt":        LaunchConfiguration("target_alt"),
                    "v_max":             LaunchConfiguration("v_max"),
                    "mission_timeout_s": LaunchConfiguration("mission_timeout_s"),
                    "hover_settle_s":    LaunchConfiguration("hover_settle_s"),
                    "status_period_s":   LaunchConfiguration("status_period_s"),
                    "color_output":      LaunchConfiguration("color_output"),
                    "warn_replan_overrun": LaunchConfiguration("warn_replan_overrun"),
                    "dry_run":           LaunchConfiguration("dry_run"),
                    "safe_dis":          LaunchConfiguration("safe_dis"),
                    "fence_fwd":         LaunchConfiguration("fence_fwd"),
                    "fence_back":        LaunchConfiguration("fence_back"),
                    "fence_lat":         LaunchConfiguration("fence_lat"),
                    "max_home_dist":     LaunchConfiguration("max_home_dist"),
                    "max_alt_error":     LaunchConfiguration("max_alt_error"),
                    "min_battery_v":     LaunchConfiguration("min_battery_v"),
                    "require_gps":       LaunchConfiguration("require_gps"),
                    "min_satellites":    LaunchConfiguration("min_satellites"),
                    "max_hdop":          LaunchConfiguration("max_hdop"),
                    "max_ground_z":      LaunchConfiguration("max_ground_z"),
                    "ekf_pre_wait_s":    LaunchConfiguration("ekf_pre_wait_s"),
                    "cam_x":             LaunchConfiguration("cam_x"),
                    "cam_y":             LaunchConfiguration("cam_y"),
                    "cam_z":             LaunchConfiguration("cam_z"),
                    "gate_enabled":      LaunchConfiguration("gate_enabled"),
                    "gate_alt_margin":   LaunchConfiguration("gate_alt_margin"),
                    # ── safety parameters that used to be locked here ────────
                    "use_safety_guards":   LaunchConfiguration("use_safety_guards"),
                    "use_lookahead_guard": LaunchConfiguration("use_lookahead_guard"),
                    "guard_clearance":     LaunchConfiguration("guard_clearance"),
                    "hard_clearance":      LaunchConfiguration("hard_clearance"),
                    "collision_cost_tol":  LaunchConfiguration("collision_cost_tol"),
                    "max_plan_cost":       LaunchConfiguration("max_plan_cost"),
                    "planning_time_ahead": LaunchConfiguration("planning_time_ahead"),
                    "speed_margin_k":      LaunchConfiguration("speed_margin_k"),
                    "use_speed_limit":     LaunchConfiguration("use_speed_limit"),
                    "blind_abort_s":       LaunchConfiguration("blind_abort_s"),
                    "stuck_abort_s":       LaunchConfiguration("stuck_abort_s"),
                    "depth_max_lag":       LaunchConfiguration("depth_max_lag"),
                    "min_battery_pct":     LaunchConfiguration("min_battery_pct"),
                    "rc_override_enabled": LaunchConfiguration("rc_override_enabled"),
                    "verify_rc_override_param":
                        LaunchConfiguration("verify_rc_override_param"),
                    "write_px4_params":    LaunchConfiguration("write_px4_params"),
                    "px4_vel_cap":         LaunchConfiguration("px4_vel_cap"),
                    "descent_speed":       LaunchConfiguration("descent_speed"),
                    "land_handoff_alt":    LaunchConfiguration("land_handoff_alt"),
                    "auto_land_mode":      LaunchConfiguration("auto_land_mode"),
                    "min_fix_type":        LaunchConfiguration("min_fix_type"),
                    "gps_wait_timeout":    LaunchConfiguration("gps_wait_timeout"),
                    "gps_stable_dur":      LaunchConfiguration("gps_stable_dur"),
                    "ekf_window_s":        LaunchConfiguration("ekf_window_s"),
                    "max_ground_drift":    LaunchConfiguration("max_ground_drift"),
                }.items(),
            ),
        ],
    )

    return LaunchDescription(args + [mavros, camera, fm_stack])
