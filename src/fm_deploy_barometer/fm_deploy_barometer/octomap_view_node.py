#!/usr/bin/env python3
"""
octomap_view_node.py — NEW (2026-09-21, MAPVIEW)
================================================

Terminal (ASCII/ANSI) top-down view of the map the planner actually uses,
so the environment can be watched over a plain SSH session with no GUI,
no X11 forwarding and no port forwarding.

It renders /projected_map — the 2D OccupancyGrid that octomap_server
projects out of the octree and that esdf_ros2.py reads — in the `odom`
frame, with the drone, home, goal and the flown trail drawn on top.

LISTENS ONLY. It publishes nothing, offers no service, never opens
/dev/ttyACM0 and never touches the mission. Killing it cannot affect a
flight in progress.

    /projected_map        nav_msgs/OccupancyGrid  the map
    /px4/sensors          std_msgs/String (JSON)  local_x/y/z, yaw, battery
    /px4/state            std_msgs/String (JSON)  connected/armed/mode

Output modes (`view_mode`):
    scroll      print a fresh block each refresh, no cursor control. Safe
                when stdout is shared with other nodes' logs — this is
                what fm_all_barometer.launch.py uses.
    fullscreen  clear the screen and redraw in place. For running the
                node alone in its own SSH terminal:
                    ros2 run fm_deploy_barometer octomap_view_node \
                        --ros-args -p view_mode:=fullscreen

Axes: ENU, same as the ESDF and the markers. Up the screen = +y (north),
right = +x (east). Character cells are about twice as tall as they are
wide, so the scale is chosen to keep the picture roughly proportional.
"""
import json
import math
import shutil
import sys
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String

# ── ANSI ────────────────────────────────────────────────────────────────────
_RESET = "\033[0m"
_C = {
    "occ":    "\033[1;31m",   # bright red
    "free":   "\033[2;37m",   # dim white
    "unk":    "",
    "drone":  "\033[1;96m",   # bright cyan
    "home":   "\033[1;33m",   # yellow
    "goal":   "\033[1;35m",   # magenta
    "trail":  "\033[34m",     # blue
    "frame":  "\033[2;37m",
    "head":   "\033[1;37m",
    "warn":   "\033[1;33m",
}
_CLEAR = "\033[2J\033[H"

# 8-way heading glyphs, index = round(yaw / 45 deg) starting at east (+x).
_ARROW_UNI = ["→", "↗", "↑", "↖",
              "←", "↙", "↓", "↘"]
_ARROW_ASC = [">", "/", "^", "\\", "<", "/", "v", "\\"]


class OctomapViewNode(Node):
    """Draws /projected_map in the terminal. Read-only diagnostic node."""

    def __init__(self):
        super().__init__("octomap_view_node")

        # Every view_* parameter is declared with dynamic typing: on the
        # command line `-p view_hz:=2` arrives as an int and
        # `-p view_color:=false` as a bool, and a fixed declared type would
        # abort the node on the type mismatch. Values are coerced below.
        def _dyn(name, default, doc):
            self.declare_parameter(
                name, default,
                ParameterDescriptor(dynamic_typing=True, description=doc))
            return self.get_parameter(name).value

        p_hz    = _dyn("view_hz",    1.0,       "redraw rate (Hz)")
        p_mode  = _dyn("view_mode",  "scroll",  "scroll | fullscreen")
        p_cols  = _dyn("view_cols",  0,         "map width in columns, 0=auto")
        p_rows  = _dyn("view_rows",  0,         "map height in rows, 0=auto")
        # 0 = fit the whole published map; > 0 = +/- this many metres around
        # the drone (a moving window, useful once the map gets large).
        p_range = _dyn("view_range", 0.0,       "0=whole map, >0=+/-N m window")
        p_color = _dyn("view_color", "auto",    "auto | true | false")
        p_ascii = _dyn("view_ascii", False,     "true = no unicode arrows")
        # Drawn exactly the way fm_inference_real_node._lock_home() computes
        # it: home + goal_dist along the nose + goal_lat to the left. Pass the
        # same values the mission was launched with, or leave 0 for no marker.
        _dyn("goal_dist",  0.0,  "goal distance ahead of home (m), 0 = no marker")
        _dyn("goal_lat",   0.0,  "goal offset to the left of home (m)")
        _dyn("trail_step", 0.30, "metres between trail crumbs")
        _dyn("trail_max",  400,  "maximum number of trail crumbs")
        self.declare_parameter("map_topic",     "/projected_map")
        self.declare_parameter("sensors_topic", "/px4/sensors")
        self.declare_parameter("state_topic",   "/px4/state")

        g = self.get_parameter
        self._hz      = max(0.1, min(10.0, float(p_hz)))
        mode          = str(p_mode).strip().lower()
        self._full    = (mode == "fullscreen")
        self._cols_p  = int(float(p_cols))
        self._rows_p  = int(float(p_rows))
        self._range   = max(0.0, float(p_range))
        self._ascii   = bool(p_ascii)
        self._goal_d  = float(g("goal_dist").value)
        self._goal_l  = float(g("goal_lat").value)
        self._t_step  = max(0.05, float(g("trail_step").value))
        self._t_max   = max(0, int(float(g("trail_max").value)))

        if isinstance(p_color, bool):
            self._color = p_color
        else:
            color = str(p_color).strip().lower()
            if color in ("true", "1", "yes", "on"):
                self._color = True
            elif color in ("false", "0", "no", "off"):
                self._color = False
            else:                       # "auto": colour only on a real TTY
                self._color = sys.stdout.isatty()

        self._arrows = _ARROW_ASC if self._ascii else _ARROW_UNI

        # ── shared state (written by callbacks, read by the render timer) ──
        self._lock  = threading.Lock()
        self._map   = None          # (array HxW int8, res, ox, oy)
        self._map_t = 0.0
        self._map_n = 0
        self._pos   = None          # (x, y, z)
        self._yaw   = 0.0
        self._bat   = None
        self._pos_t = 0.0
        self._armed = False
        self._mode  = ""
        self._conn  = False
        self._home  = None          # latched on the disarmed -> armed edge
        self._home_yaw = 0.0
        self._goal  = None
        self._trail = []

        self.create_subscription(
            OccupancyGrid, str(g("map_topic").value), self._cb_map, 10)
        self.create_subscription(
            String, str(g("sensors_topic").value), self._cb_sensors, 10)
        self.create_subscription(
            String, str(g("state_topic").value), self._cb_state, 10)

        self.create_timer(1.0 / self._hz, self._render)

        self.get_logger().info(
            "[MAP] octomap terminal view — mode={0} {1:.1f} Hz  color={2}  "
            "goal={3:.1f}m/{4:+.1f}m  (listens only, publishes nothing)"
            .format(mode, self._hz, self._color, self._goal_d, self._goal_l))

    # ── callbacks ───────────────────────────────────────────────────────────

    def _cb_map(self, msg: OccupancyGrid):
        try:
            h, w = msg.info.height, msg.info.width
            if h <= 0 or w <= 0:
                return
            a = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
            with self._lock:
                self._map = (a, float(msg.info.resolution),
                             float(msg.info.origin.position.x),
                             float(msg.info.origin.position.y))
                self._map_t = time.time()
                self._map_n += 1
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn("[MAP] bad OccupancyGrid: {0}".format(exc))

    def _cb_sensors(self, msg: String):
        try:
            d = json.loads(msg.data)
            x = float(d.get("local_x", 0.0))
            y = float(d.get("local_y", 0.0))
            z = float(d.get("local_z", 0.0))
            yaw = float(d.get("yaw", 0.0))
            bat = d.get("battery_pct", None)
        except Exception:                                         # noqa: BLE001
            return
        with self._lock:
            self._pos = (x, y, z)
            self._yaw = yaw
            self._bat = bat
            self._pos_t = time.time()
            if self._t_max and (not self._trail or
                                math.hypot(x - self._trail[-1][0],
                                           y - self._trail[-1][1])
                                >= self._t_step):
                self._trail.append((x, y))
                while len(self._trail) > self._t_max:
                    del self._trail[0]

    def _cb_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            armed = bool(d.get("armed", False))
            mode  = str(d.get("mode", ""))
            conn  = bool(d.get("connected", False))
        except Exception:                                         # noqa: BLE001
            return
        with self._lock:
            # Latch home on the disarmed -> armed edge: that is when
            # fm_inference_real_node._lock_home() fixes its own home frame.
            if armed and not self._armed and self._pos is not None:
                self._home = (self._pos[0], self._pos[1])
                self._home_yaw = self._yaw
                self._trail = [self._home]
                self._goal = None
                if self._goal_d != 0.0 or self._goal_l != 0.0:
                    c, s = math.cos(self._yaw), math.sin(self._yaw)
                    self._goal = (self._home[0] + c * self._goal_d
                                  - s * self._goal_l,
                                  self._home[1] + s * self._goal_d
                                  + c * self._goal_l)
            self._armed, self._mode, self._conn = armed, mode, conn

    # ── rendering ───────────────────────────────────────────────────────────

    def _paint(self, key, text):
        if not self._color or not _C.get(key):
            return text
        return _C[key] + text + _RESET

    def _term(self):
        """Character grid available for the map body."""
        cols, rows = self._cols_p, self._rows_p
        if cols <= 0 or rows <= 0:
            try:
                size = shutil.get_terminal_size((100, 32))
            except Exception:                                     # noqa: BLE001
                size = (100, 32)
            if cols <= 0:
                cols = size[0]
            if rows <= 0:
                # header (2) + border (2) + legend (1) + breathing room
                rows = size[1] - 7
        return max(20, min(240, cols - 2)), max(6, min(120, rows))

    def _render(self):
        with self._lock:
            mp     = self._map
            map_t  = self._map_t
            map_n  = self._map_n
            pos    = self._pos
            yaw    = self._yaw
            bat    = self._bat
            pos_t  = self._pos_t
            armed  = self._armed
            mode   = self._mode
            conn   = self._conn
            home   = self._home
            goal   = self._goal
            trail  = list(self._trail)

        now = time.time()
        out = []
        if self._full:
            out.append(_CLEAR)

        if mp is None:
            waiting = ("[MAP] waiting for the map on /projected_map — "
                       "octomap_server publishes once the camera has "
                       "produced a cloud")
            out.append(self._paint("warn", waiting))
            out.append(self._status_line(pos, yaw, bat, pos_t, armed, mode,
                                         conn, now))
            self._emit(out)
            return

        a, res, ox, oy = mp
        h, w = a.shape
        cols, rows = self._term()

        # ── viewport in world metres ───────────────────────────────────────
        if self._range > 0.0 and pos is not None:
            x0, x1 = pos[0] - self._range, pos[0] + self._range
            y0, y1 = pos[1] - self._range, pos[1] + self._range
        else:
            x0, x1 = ox, ox + w * res
            y0, y1 = oy, oy + h * res
        span_x = max(res, x1 - x0)
        span_y = max(res, y1 - y0)

        # A character cell is ~2x taller than wide: keep metres-per-row at
        # twice the metres-per-column so the picture is not squashed.
        m_col = max(span_x / cols, span_y / (2.0 * rows), res * 0.5)
        m_row = 2.0 * m_col
        n_col = max(1, min(cols, int(math.ceil(span_x / m_col))))
        n_row = max(1, min(rows, int(math.ceil(span_y / m_row))))

        # ── aggregate the grid into the character cells ────────────────────
        xc = ox + (np.arange(w) + 0.5) * res
        yc = oy + (np.arange(h) + 0.5) * res
        c_idx = np.floor((xc - x0) / m_col).astype(np.int64)
        r_idx = np.floor((yc - y0) / m_row).astype(np.int64)
        c_ok  = np.where((c_idx >= 0) & (c_idx < n_col))[0]
        r_ok  = np.where((r_idx >= 0) & (r_idx < n_row))[0]

        occ_n  = np.zeros(n_row * n_col, dtype=np.int64)
        free_n = np.zeros(n_row * n_col, dtype=np.int64)
        if c_ok.size and r_ok.size:
            sub  = a[np.ix_(r_ok, c_ok)]
            flat = (r_idx[r_ok][:, None] * n_col + c_idx[c_ok][None, :]).ravel()
            v    = sub.ravel()
            occ_n  = np.bincount(flat[v >= 50], minlength=n_row * n_col)
            free_n = np.bincount(flat[(v >= 0) & (v < 50)],
                                 minlength=n_row * n_col)
        occ_n  = occ_n.reshape(n_row, n_col)
        free_n = free_n.reshape(n_row, n_col)

        base = np.where(occ_n > 0, "#", np.where(free_n > 0, ".", " "))
        kind = np.where(occ_n > 0, "occ", np.where(free_n > 0, "free", "unk"))

        # ── overlays (later ones win) ──────────────────────────────────────
        def cell(px, py):
            c = int(math.floor((px - x0) / m_col))
            r = int(math.floor((py - y0) / m_row))
            if 0 <= c < n_col and 0 <= r < n_row:
                return r, c
            return None

        for tx, ty in trail:
            rc = cell(tx, ty)
            if rc:
                base[rc], kind[rc] = "o", "trail"
        if home:
            rc = cell(*home)
            if rc:
                base[rc], kind[rc] = "H", "home"
        if goal:
            rc = cell(*goal)
            if rc:
                base[rc], kind[rc] = "G", "goal"
        drone_off = True
        if pos is not None:
            rc = cell(pos[0], pos[1])
            if rc:
                idx = int(round(math.degrees(yaw) / 45.0)) % 8
                base[rc], kind[rc] = self._arrows[idx], "drone"
                drone_off = False

        # ── assemble (row 0 is the SOUTH edge, so print top-down reversed) ──
        out.append(self._status_line(pos, yaw, bat, pos_t, armed, mode,
                                     conn, now))
        out.append(self._paint("head",
                   "      map {0}x{1} @ {2:.2f}m  age {3:.1f}s  #{4}  "
                   "view {5:.1f}x{6:.1f}m  scale {7:.2f}m/col  occ {8}"
                   .format(w, h, res, now - map_t, map_n,
                           n_col * m_col, n_row * m_row, m_col,
                           int(np.count_nonzero(a >= 50)))))

        bar = self._paint("frame", "+" + "-" * n_col + "+")
        # Append the compass AFTER the painted bar — slicing into it would cut
        # the ANSI reset in half once view_color is on.
        out.append(bar + self._paint("frame", "   N ^ +y (east = right)"))
        for r in range(n_row - 1, -1, -1):
            line = [self._paint("frame", "|")]
            for c in range(n_col):
                line.append(self._paint(kind[r, c], str(base[r, c])))
            line.append(self._paint("frame", "|"))
            out.append("".join(line))
        out.append(bar)
        legend = ("  {0} occupied  {1} free  ' ' unknown   {2} drone  "
                  "{3} home  {4} goal  {5} trail"
                  .format(self._paint("occ", "#"), self._paint("free", "."),
                          self._paint("drone", self._arrows[0]),
                          self._paint("home", "H"),
                          self._paint("goal", "G"),
                          self._paint("trail", "o")))
        if drone_off and pos is not None:
            legend += self._paint("warn", "   [drone outside view]")
        out.append(legend)
        self._emit(out)

    def _status_line(self, pos, yaw, bat, pos_t, armed, mode, conn, now):
        clk = time.strftime("%H:%M:%S")
        if pos is None:
            body = "no /px4/sensors yet"
        else:
            age = now - pos_t
            body = ("pos=({0:+.2f},{1:+.2f}) z={2:+.2f}m yaw={3:+.0f}deg"
                    .format(pos[0], pos[1], pos[2], math.degrees(yaw)))
            if age > 1.0:
                body += self._paint("warn", " STALE {0:.1f}s".format(age))
        state = "{0}{1}".format(mode or "?", " ARMED" if armed else "")
        if not conn:
            state = self._paint("warn", "NO FCU")
        if bat is not None:
            body += "  bat={0:.0f}%".format(bat)
        return self._paint("head",
                           "[MAP] {0}  {1}  {2}".format(clk, state, body))

    def _emit(self, lines):
        sys.stdout.write(("" if self._full else "\n") + "\n".join(lines) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = OctomapViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
