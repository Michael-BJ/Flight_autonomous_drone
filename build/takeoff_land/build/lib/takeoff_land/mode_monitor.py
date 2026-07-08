#!/usr/bin/env python3
"""
mode_monitor.py
===============
Real-time PX4 mode and status monitor.
Displays mode changes, arm/disarm events, and RC override detection.

USAGE:
    ros2 run takeoff_land mode_monitor
"""
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ANSI colors
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
BLUE   = "\033[94m"
CYAN   = "\033[96m"
WHITE  = "\033[97m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"

_MODE_COLOR = {
    "OFFBOARD":   GREEN,
    "AUTO.LAND":  CYAN,
    "AUTO.RTL":   CYAN,
    "AUTO.LOITER": CYAN,
    "AUTO.MISSION": CYAN,
    "POSCTL":     BLUE,
    "STABILIZED": BLUE,
    "ALTCTL":     BLUE,
    "MANUAL":     YELLOW,
    "ACRO":       YELLOW,
}


class ModeMonitor(Node):

    def __init__(self):
        super().__init__("mode_monitor")

        self._mode      = ""
        self._armed     = False
        self._connected = False
        self._pos_z     = 0.0
        self._vel_z     = 0.0
        self._alt_rel   = 0.0
        self._gps_fix   = ""
        self._bat_pct   = None

        self.create_subscription(String, "/px4/state",   self._cb_state,   10)
        self.create_subscription(String, "/px4/sensors", self._cb_sensors, 10)
        self.create_timer(1.0, self._tick)

        print(f"\n{BOLD}{'='*62}{RESET}")
        print(f"{BOLD}  PX4 MODE MONITOR  —  waiting for data...{RESET}")
        print(f"{BOLD}{'='*62}{RESET}")
        print(f"{DIM}Events (mode change / arm / RC override) appear on new lines.{RESET}")
        print(f"{DIM}Status line updates every 1 second.{RESET}\n")

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _ts():
        return datetime.now().strftime("%H:%M:%S")

    @staticmethod
    def _mode_str(mode: str) -> str:
        c = _MODE_COLOR.get(mode, WHITE)
        return f"{c}{BOLD}{mode}{RESET}"

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            prev_mode  = self._mode
            prev_armed = self._armed

            self._connected = d.get("connected", False)
            self._armed     = d.get("armed",     False)
            self._mode      = d.get("mode",      "")

            # Mode change event — print on new line so it stays visible
            if self._mode != prev_mode and prev_mode != "":
                print(f"\n[{self._ts()}] MODE  {self._mode_str(prev_mode)} "
                      f"-> {self._mode_str(self._mode)}")

                # RC override warning
                if (prev_mode == "OFFBOARD" and
                        self._mode not in ("OFFBOARD", "AUTO.LAND", "")):
                    print(f"{RED}{BOLD}          *** RC OVERRIDE DETECTED ***{RESET}")
                    print(f"{RED}          Setpoints from mission node stopped.{RESET}")
                    print(f"{RED}          RC pilot has full control of drone.{RESET}")

            # Arm / disarm event
            if self._armed != prev_armed:
                if self._armed:
                    print(f"\n[{self._ts()}] {RED}{BOLD}>>> ARMED <<<{RESET}")
                else:
                    print(f"\n[{self._ts()}] {GREEN}{BOLD}>>> DISARMED <<<{RESET}")
        except Exception:
            pass

    def _cb_sensors(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._pos_z  = float(d.get("local_z",  0.0))
            self._vel_z  = float(d.get("vel_z",    0.0))
            self._alt_rel = float(d.get("alt_rel",  0.0))
            self._gps_fix = str(d.get("gps_fix",   ""))
            bat = d.get("battery_pct")
            self._bat_pct = float(bat) if bat is not None else None
        except Exception:
            pass

    # ── periodic status line ──────────────────────────────────────────────────

    def _tick(self):
        if not self._connected:
            print(f"\r[{self._ts()}] {RED}FCU: DISCONNECTED{RESET} "
                  f"— waiting for MAVROS and px4_sensor_reader...", end="", flush=True)
            return

        conn_c = GREEN if self._connected else RED
        arm_c  = RED   if self._armed     else GREEN
        arm_s  = "ARMED   " if self._armed else "DISARMED"

        bat_s = ""
        if self._bat_pct is not None:
            bc = GREEN if self._bat_pct > 40 else (YELLOW if self._bat_pct > 20 else RED)
            bat_s = f"  BAT:{bc}{self._bat_pct:.0f}%{RESET}"

        gps_c = GREEN if "FIX" in self._gps_fix else RED
        gps_s = f"{gps_c}{self._gps_fix}{RESET}" if self._gps_fix else f"{RED}NO GPS{RESET}"

        print(
            f"\r[{self._ts()}]  "
            f"Mode: {self._mode_str(self._mode):30s}  "
            f"{arm_c}{BOLD}{arm_s}{RESET}  "
            f"z={self._pos_z:+.2f}m  vz={self._vel_z:+.2f}m/s  "
            f"Rel:{self._alt_rel:+.2f}m  "
            f"GPS:{gps_s}"
            f"{bat_s}   ",
            end="", flush=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ModeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n\n{BOLD}Monitor stopped.{RESET}\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
