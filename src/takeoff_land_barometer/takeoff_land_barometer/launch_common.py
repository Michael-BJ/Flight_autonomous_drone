#!/usr/bin/env python3
"""
launch_common.py — pieces shared by the takeoff_land_barometer and
forward_move_barometer launch files: the argument table (takeoff_land's
names + the BARO ones), the parameter dict for the flight node, and the
MAVROS + px4_sensor_reader start-up (same as takeoff_land.launch.py).
Importable from a launch file because this module is installed with the
python package.
"""
import os
import subprocess

from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_MAVROS_CFG = "/opt/ros/humble/share/mavros/launch/px4_config.yaml"
_LOCAL_PLG  = "/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml"

# Every flight node of this workspace: two setpoint streams must never run
# at the same time.
_FLIGHT_NODES = ("takeoff_land_node", "takeoff_land_baro_node",
                 "forward_move_node", "forward_move_baro_node",
                 "hold_position_node",
                 "fm_inference_real_node", "fm_inference_baro_node",
                 "fm_inference_recovery_node")


def _kill_stale(context):
    import time, glob
    try:
        subprocess.run(["pkill", "-9", "-f", "mavros_node"], capture_output=True)
        subprocess.run(["pkill", "-9", "-f", "px4_sensor_reader"], capture_output=True)
        for n in _FLIGHT_NODES:
            subprocess.run(["pkill", "-9", "-f", n], capture_output=True)
    except Exception:
        pass
    time.sleep(2.0)
    for shm in glob.glob("/dev/shm/fastrtps_*"):
        try:
            os.remove(shm)
        except Exception:
            pass
    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
    time.sleep(1.0)
    subprocess.run(["ros2", "daemon", "start"], capture_output=True)
    time.sleep(1.0)
    return []


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _b(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _i(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def takeoff_land_arguments(max_ground_z="1000.0", max_ground_drift="0.5"):
    """The takeoff_land.launch.py arguments (same names) + the BARO ones.
    Shared with forward_move_barometer.launch.py."""
    return [
        DeclareLaunchArgument(
            "fcu_url", default_value="/dev/ttyACM0:57600",
            description="FCU URL — USB: /dev/ttyACM0:57600 | Jetson UART: /dev/ttyTHS1:921600"),
        DeclareLaunchArgument("target_alt",     default_value="1.2"),
        DeclareLaunchArgument("hover_time",     default_value="8.0"),
        DeclareLaunchArgument("cmd_hz",         default_value="50"),
        DeclareLaunchArgument("descent_speed",  default_value="0.3"),
        DeclareLaunchArgument("auto_land_mode", default_value="true"),
        DeclareLaunchArgument("land_handoff_alt",    default_value="0.25"),
        DeclareLaunchArgument("rc_override_enabled", default_value="true"),
        DeclareLaunchArgument(
            "verify_rc_override_param", default_value="true",
            description="Pre-flight check of PX4 COM_RC_OVERRIDE. Set false only "
                        "for bench tests without a battery/RC bound."),
        DeclareLaunchArgument(
            "require_rc_offboard", default_value="true",
            description="Refuse to ARM unless the RC mode switch is already "
                        "in the OFFBOARD slot. Set false only for bench "
                        "tests with no RC bound."),
        DeclareLaunchArgument("max_pos_error",  default_value="2.0"),
        DeclareLaunchArgument("max_vz",         default_value="3.0"),
        DeclareLaunchArgument(
            "max_alt_error",  default_value="1.5",
            description="Max BAROMETRIC altitude deviation from the target (m) "
                        "during hover/forward -> AUTO.LAND."),
        DeclareLaunchArgument(
            "require_gps", default_value="true",
            description="Refuse to ARM until GPS is genuinely usable (x/y still "
                        "come from the GPS). Set false ONLY indoors with VIO/flow."),
        DeclareLaunchArgument("min_fix_type",     default_value="3"),
        DeclareLaunchArgument("min_satellites",   default_value="8"),
        DeclareLaunchArgument("max_hdop",         default_value="2.0"),
        DeclareLaunchArgument("gps_wait_timeout", default_value="120.0"),
        DeclareLaunchArgument("gps_stable_dur",   default_value="5.0"),
        # ── EKF criteria on the ground (BARO defaults, see the docstring) ────
        DeclareLaunchArgument(
            "max_ground_z", default_value=max_ground_z,
            description="BARO: max |EKF z| on the ground. Irrelevant to the "
                        "barometric loop (setpoints are relative to the live "
                        "EKF z), so effectively disabled. takeoff_land: 1.0."),
        DeclareLaunchArgument("ekf_window_s",     default_value="5.0"),
        DeclareLaunchArgument(
            "max_ground_drift", default_value=max_ground_drift,
            description="BARO: max EKF z peak-to-peak over ekf_window_s on the "
                        "ground. Slow drift is compensated by the barometric "
                        "loop, so relaxed from takeoff_land's 0.2 m."),
        DeclareLaunchArgument(
            "ekf_gate_std", default_value="0.15",
            description="BARO: max EKF z std over ekf_window_s on the ground. "
                        "PX4 still flies on its own vertical velocity, which "
                        "this loop cannot fix — a violently jittering EKF is "
                        "still refused. takeoff_land uses 0.08."),
        # ── Barometric altitude hold (see baro_altitude.py) ──────────────────
        DeclareLaunchArgument(
            "baro_gain", default_value="0.0",   # NEW (2026-09-15, GAIN0): was 0.7
            description="0 = no barometric loop (setpoint z = target, altitude = "
                        "PX4 EKF, which already uses the barometer). >0: "
                        "z_sp = z_ekf + gain*(alt_target - alt_baro) (0.7 = old)."),
        DeclareLaunchArgument(
            "baro_tau_s", default_value="1.0",
            description="Low-pass time constant on the raw barometer (~0.085 m "
                        "RMS white noise at 10 Hz measured 2026-09-13 -> ~3 cm)."),
        DeclareLaunchArgument(
            "baro_ground_effect_alt", default_value="1.0",
            description="Below this altitude the barometer is NOT used "
                        "(propeller downwash); the EKF z dead-reckons from the "
                        "last trusted barometric fix, i.e. the first/last metre "
                        "behave like takeoff_land."),
        DeclareLaunchArgument("baro_blend_s",        default_value="2.0"),
        DeclareLaunchArgument("baro_glitch_m",       default_value="1.5"),
        DeclareLaunchArgument("baro_stale_s",        default_value="1.0"),
        DeclareLaunchArgument(
            "baro_stale_abort_s", default_value="5.0",
            description="No barometer sample for this long during the mission "
                        "-> AUTO.LAND (0 = never)."),
        DeclareLaunchArgument("baro_max_cmd_offset", default_value="1.5"),
        DeclareLaunchArgument(
            "baro_gate_std", default_value="0.08",
            description="Pre-flight: max std of the FILTERED barometric altitude "
                        "over ekf_window_s (measured on the ground: ~0.02)."),
        DeclareLaunchArgument(
            "baro_gate_drift", default_value="0.30",
            description="Pre-flight: max peak-to-peak of the filtered barometric "
                        "altitude over ekf_window_s."),
        DeclareLaunchArgument("baro_min_rate_hz",    default_value="5.0"),
        # ── Terminal monitoring ─────────────────────────────────────────────
        DeclareLaunchArgument(
            "status_period_s", default_value="2.0",
            description="How often the [T+mm:ss] PHASE status line is printed."),
        DeclareLaunchArgument(
            "color_output", default_value="true",
            description="ANSI colour. Set false when piping the log to a file."),
    ]


def takeoff_land_parameters():
    """Parameter dict for the flight node (same names as takeoff_land)."""
    return {
        "target_alt":          _f("target_alt"),
        "hover_time":          _f("hover_time"),
        "cmd_hz":              _i("cmd_hz"),
        "descent_speed":       _f("descent_speed"),
        "auto_land_mode":      _b("auto_land_mode"),
        "land_handoff_alt":    _f("land_handoff_alt"),
        "rc_override_enabled": _b("rc_override_enabled"),
        "verify_rc_override_param": _b("verify_rc_override_param"),
        "require_rc_offboard": _b("require_rc_offboard"),
        "max_pos_error":       _f("max_pos_error"),
        "max_vz":              _f("max_vz"),
        "max_alt_error":       _f("max_alt_error"),
        "require_gps":         _b("require_gps"),
        "min_fix_type":        _i("min_fix_type"),
        "min_satellites":      _i("min_satellites"),
        "max_hdop":            _f("max_hdop"),
        "gps_wait_timeout":    _f("gps_wait_timeout"),
        "gps_stable_dur":      _f("gps_stable_dur"),
        "max_ground_z":        _f("max_ground_z"),
        "ekf_window_s":        _f("ekf_window_s"),
        "max_ground_drift":    _f("max_ground_drift"),
        "ekf_gate_std":        _f("ekf_gate_std"),
        "baro_gain":           _f("baro_gain"),
        "baro_tau_s":          _f("baro_tau_s"),
        "baro_ground_effect_alt": _f("baro_ground_effect_alt"),
        "baro_blend_s":        _f("baro_blend_s"),
        "baro_glitch_m":       _f("baro_glitch_m"),
        "baro_stale_s":        _f("baro_stale_s"),
        "baro_stale_abort_s":  _f("baro_stale_abort_s"),
        "baro_max_cmd_offset": _f("baro_max_cmd_offset"),
        "baro_gate_std":       _f("baro_gate_std"),
        "baro_gate_drift":     _f("baro_gate_drift"),
        "baro_min_rate_hz":    _f("baro_min_rate_hz"),
        "status_period_s":     _f("status_period_s"),
        "color_output":        _b("color_output"),
        "use_ekf_stable":      True,
    }


def mavros_and_reader():
    """MAVROS + px4_sensor_reader, as in takeoff_land.launch.py."""
    fcu_url = LaunchConfiguration("fcu_url")
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        output="screen",
        parameters=[
            {"fcu_url":             fcu_url},
            {"gcs_url":             ""},
            {"target_system_id":    1},
            {"target_component_id": 1},
            {"fcu_protocol":        "v2.0"},
            _MAVROS_CFG,
            _LOCAL_PLG,
        ],
    )
    reader = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="takeoff_land",
                executable="px4_sensor_reader",
                name="px4_sensor_reader",
                output="screen",
                parameters=[{"fcu_url": fcu_url, "auto_launch_mavros": False}],
            ),
        ],
    )
    return [OpaqueFunction(function=_kill_stale), mavros_node, reader]


