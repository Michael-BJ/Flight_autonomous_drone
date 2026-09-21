#!/usr/bin/env python3
"""
offline_test_mapview.py — NEW (2026-09-21, MAPVIEW)

Offline checks for octomap_view_node. Does NOT start MAVROS, does NOT arm
anything and does NOT spin: it builds the node, hands the callbacks
synthetic messages and inspects what _render() writes to stdout.

    /usr/bin/python3 src/fm_deploy_barometer/test/offline_test_mapview.py
"""
import io
import math
import os
import re
import sys
from contextlib import redirect_stdout

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from fm_deploy_barometer.octomap_view_node import OctomapViewNode  # noqa: E402

import json  # noqa: E402

PASS = FAIL = 0
ANSI = re.compile(r"\033\[[0-9;]*m")


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS  {0}".format(name))
    else:
        FAIL += 1
        print("  FAIL  {0}  {1}".format(name, detail))


def make_grid(res=0.15, w=80, h=60, ox=-6.0, oy=-4.5):
    """Free everywhere, a wall of occupied cells, an unknown corner."""
    a = np.zeros((h, w), dtype=np.int8)
    a[:, :] = 0
    a[45:49, 30:70] = 100          # wall to the north (y = +2.3..+2.9 m)
    a[0:10, 0:10] = -1             # unknown patch, south-west corner
    g = OccupancyGrid()
    g.info.resolution = res
    g.info.width = w
    g.info.height = h
    g.info.origin.position.x = ox
    g.info.origin.position.y = oy
    g.data = [int(v) for v in a.ravel()]
    return g


def sensors(x, y, z=2.0, yaw=0.0, bat=80.0):
    m = String()
    m.data = json.dumps({"local_x": x, "local_y": y, "local_z": z,
                         "yaw": yaw, "battery_pct": bat})
    return m


def state(armed, mode="OFFBOARD", conn=True):
    m = String()
    m.data = json.dumps({"armed": armed, "mode": mode, "connected": conn})
    return m


def render(node):
    buf = io.StringIO()
    with redirect_stdout(buf):
        node._render()
    return buf.getvalue()


def body_rows(text):
    """The map body lines (between the +---+ borders), ANSI stripped."""
    plain = [ANSI.sub("", ln) for ln in text.splitlines()]
    idx = [i for i, ln in enumerate(plain) if ln.startswith("+-")]
    if len(idx) < 2:
        return []
    return [ln for ln in plain[idx[0] + 1:idx[-1]] if ln.startswith("|")]


def new_node(**over):
    n = OctomapViewNode()
    n._color = False
    n._cols_p, n._rows_p = 60, 14
    for k, v in over.items():
        setattr(n, k, v)
    return n


def main():
    rclpy.init()
    print("== octomap_view_node offline tests (no MAVROS, no arming) ==")

    # ── 1. no map yet ──────────────────────────────────────────────────────
    n = new_node()
    out = render(n)
    check("1 waiting message before any map", "waiting for the map" in out, out)
    check("1 no crash without pose", "no /px4/sensors" in out, out)

    # ── 2. map + drone + home + goal ───────────────────────────────────────
    n = new_node()
    n._goal_d, n._goal_l = 4.0, 0.0
    n._cb_map(make_grid())
    n._cb_sensors(sensors(0.0, 0.0, 2.0, math.radians(90)))
    n._cb_state(state(False))
    n._cb_state(state(True))              # disarmed -> armed latches home
    n._cb_sensors(sensors(0.5, 1.0, 2.0, math.radians(90)))
    out = render(n)
    rows = body_rows(out)
    plain = ANSI.sub("", out)
    check("2 body drawn", len(rows) >= 5, "rows={0}".format(len(rows)))
    check("2 rows within view_rows", len(rows) <= 14, str(len(rows)))
    check("2 all rows same width",
          len({len(r) for r in rows}) == 1, str({len(r) for r in rows}))
    check("2 row width within view_cols",
          len(rows[0]) - 2 <= 60, str(len(rows[0])))
    check("2 occupied cells shown", any("#" in r for r in rows))
    check("2 free cells shown", any("." in r for r in rows))
    check("2 unknown cells shown", any("  " in r for r in rows))
    check("2 home marker", any("H" in r for r in rows))
    check("2 goal marker", any("G" in r for r in rows))
    check("2 drone arrow (north)", any("↑" in r for r in rows), plain)
    check("2 status line", "[MAP]" in plain and "OFFBOARD ARMED" in plain)
    check("2 battery shown", "bat=80%" in plain, plain)
    check("2 legend", "occupied" in plain and "trail" in plain)
    check("2 no stale warning on fresh pose", "STALE" not in plain, plain)

    # ── 3. geometry: north wall is drawn above the drone ───────────────────
    wall_rows = [i for i, r in enumerate(rows) if "#" in r]
    drone_rows = [i for i, r in enumerate(rows) if "↑" in r]
    check("3 wall north of the drone (printed above it)",
          wall_rows and drone_rows and max(wall_rows) < min(drone_rows),
          "wall={0} drone={1}".format(wall_rows, drone_rows))

    # Same wall, drone moved north of it -> must now print below the drone.
    n._cb_sensors(sensors(0.5, 4.0, 2.0, math.radians(90)))
    rows2 = body_rows(render(n))
    w2 = [i for i, r in enumerate(rows2) if "#" in r]
    d2 = [i for i, r in enumerate(rows2) if "\u2191" in r]
    check("3 wall south of the drone (printed below it)",
          w2 and d2 and min(w2) > max(d2), "wall={0} drone={1}".format(w2, d2))

    # East/west: a pillar east of the drone must land in a right-hand column.
    n3 = new_node()
    g3 = make_grid()
    a3 = np.asarray(g3.data, dtype=np.int8).reshape(60, 80).copy()
    a3[:, :] = 0
    a3[28:32, 60:64] = 100          # x = +3.0..+3.6 m
    g3.data = [int(v) for v in a3.ravel()]
    n3._cb_map(g3)
    n3._cb_sensors(sensors(0.0, 0.0, 2.0, 0.0))
    rows3 = body_rows(render(n3))
    occ_cols = [c for r in rows3 for c, ch in enumerate(r) if ch == "#"]
    dr_cols = [c for r in rows3 for c, ch in enumerate(r) if ch == "\u2192"]
    check("3 pillar east of the drone (printed right of it)",
          occ_cols and dr_cols and min(occ_cols) > max(dr_cols),
          "occ={0} drone={1}".format(occ_cols[:4], dr_cols))

    # ── 4. heading glyphs ──────────────────────────────────────────────────
    for deg, arrow, label in ((0, "→", "east"), (90, "↑", "north"),
                              (180, "←", "west"), (-90, "↓", "south"),
                              (45, "↗", "north-east")):
        n._cb_sensors(sensors(0.0, 0.0, 2.0, math.radians(deg)))
        check("4 heading {0}".format(label),
              any(arrow in r for r in body_rows(render(n))), str(deg))

    # ── 5. trail crumbs ────────────────────────────────────────────────────
    n = new_node()
    n._cb_map(make_grid())
    n._cb_state(state(False))
    for i in range(30):
        n._cb_sensors(sensors(0.0, i * 0.1, 2.0, math.radians(90)))
    check("5 crumb every trail_step (0.3 m)",
          abs(len(n._trail) - 10) <= 1, str(len(n._trail)))
    check("5 trail drawn", any("o" in r for r in body_rows(render(n))))
    n._t_max = 5
    for i in range(60):
        n._cb_sensors(sensors(i * 0.5, 0.0))
    check("5 trail capped at trail_max", len(n._trail) <= 5, str(len(n._trail)))

    # ── 6. moving window (view_range) ──────────────────────────────────────
    n = new_node()
    n._range = 2.0
    n._cb_map(make_grid())
    n._cb_sensors(sensors(0.0, 0.0, 2.0, 0.0))
    out = ANSI.sub("", render(n))
    m = re.search(r"view ([0-9.]+)x([0-9.]+)m", out)
    check("6 window sized from view_range", m is not None, out)
    if m:
        check("6 window ~4x4 m", abs(float(m.group(1)) - 4.0) < 0.6
              and abs(float(m.group(2)) - 4.0) < 0.6, m.group(0))
    check("6 drone inside the window",
          "[drone outside view]" not in out, out)

    # ── 7. drone outside the map ───────────────────────────────────────────
    n = new_node()
    n._cb_map(make_grid())
    n._cb_sensors(sensors(500.0, 500.0))
    out = ANSI.sub("", render(n))
    check("7 flagged, still renders",
          "[drone outside view]" in out and body_rows(render(n)), out)

    # ── 8. stale pose warning ──────────────────────────────────────────────
    n = new_node()
    n._cb_map(make_grid())
    n._cb_sensors(sensors(0.0, 0.0))
    n._pos_t -= 5.0
    check("8 STALE shown", "STALE" in ANSI.sub("", render(n)))

    # ── 9. output modes ────────────────────────────────────────────────────
    n = new_node()
    n._cb_map(make_grid())
    n._cb_sensors(sensors(0.0, 0.0))
    scroll = render(n)
    check("9 scroll mode has no cursor control",
          "\033[2J" not in scroll and scroll.startswith("\n"))
    n._full = True
    check("9 fullscreen clears the screen", "\033[2J" in render(n))

    # ── 10. colour ─────────────────────────────────────────────────────────
    n = new_node()
    n._cb_map(make_grid())
    n._cb_sensors(sensors(0.0, 0.0))
    check("10 no ANSI when view_color false", "\033[" not in render(n))
    plainmap = render(n)
    n._color = True
    coloured = render(n)
    check("10 ANSI when view_color true", "\033[1;31m" in coloured)
    # Stripping the ANSI out of the coloured render must give back exactly the
    # uncoloured one (minus the clock): catches any slicing into an escape.
    def scrub(t):
        return [re.sub(r"\d\d:\d\d:\d\d", "", ln)
                for ln in ANSI.sub("", t).splitlines()]
    check("10 colour adds nothing but escapes",
          scrub(coloured) == scrub(plainmap),
          "\n".join(a + "  !=  " + b for a, b in
                    zip(scrub(coloured), scrub(plainmap)) if a != b))
    check("10 no broken escape leaks as text",
          not re.search(r"\d+m[-+|]", ANSI.sub("", coloured)),
          ANSI.sub("", coloured)[:200])
    check("10 compass label kept on the border",
          any("N ^ +y" in ln for ln in ANSI.sub("", coloured).splitlines()))

    # ── 11. ascii fallback ─────────────────────────────────────────────────
    n = new_node()
    n._ascii = True
    from fm_deploy_barometer.octomap_view_node import _ARROW_ASC
    n._arrows = _ARROW_ASC
    n._cb_map(make_grid())
    n._cb_sensors(sensors(0.0, 0.0, 2.0, 0.0))
    out = render(n)
    check("11 ascii arrow, no unicode",
          any(">" in r for r in body_rows(out))
          and all(ord(c) < 128 for c in out), "non-ascii present")

    # ── 12. robustness ─────────────────────────────────────────────────────
    n = new_node()
    bad = OccupancyGrid()               # 0x0
    n._cb_map(bad)
    check("12 empty grid ignored", n._map is None)
    junk = String(); junk.data = "not json"
    n._cb_sensors(junk); n._cb_state(junk)
    check("12 junk JSON ignored", n._pos is None and n._mode == "")
    n._cb_map(make_grid(res=0.15, w=400, h=400, ox=-30.0, oy=-30.0))
    n._cb_sensors(sensors(0.0, 0.0))
    big = body_rows(render(n))
    check("12 big map still fits the terminal",
          len(big) <= 14 and len(big[0]) - 2 <= 60,
          "{0}x{1}".format(len(big), len(big[0]) - 2))

    # ── 13. read-only: no publisher, no service client ─────────────────────
    n = new_node()
    pubs = [t for t, _ in n.get_publisher_names_and_types_by_node(
        n.get_name(), n.get_namespace())]
    talk = [t for t in pubs if not t.startswith(("/rosout", "/parameter_events"))]
    check("13 publishes nothing but rosout", not talk, str(talk))
    check("13 no service clients for arming/param",
          not hasattr(n, "_arming_client") and not hasattr(n, "_param_client"))

    # ── 14. parameters as they really arrive from the command line ────────
    # `-p view_hz:=2` is an int and `-p view_color:=false` a bool; a fixed
    # declared type would abort the node before it drew anything. Run a real
    # process so the ros-args parser is the one under test.
    import subprocess
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    snippet = (
        "import sys, rclpy;"
        "sys.path.insert(0, {0!r});"
        "from fm_deploy_barometer.octomap_view_node import OctomapViewNode;"
        "rclpy.init(args=sys.argv[1:]);"
        "n = OctomapViewNode();"
        "print('RESULT', n._color, n._hz, n._cols_p, n._goal_d, n._full)"
    ).format(src)
    for label, cli, want in (
        ("int for a float param",
         ["-p", "view_hz:=2", "-p", "goal_dist:=5"], "2.0 0 5.0"),
        ("bool for view_color",
         ["-p", "view_color:=false"], "False"),
        ("string 'true' for view_color",
         ["-p", "view_color:=true"], "True"),
        ("float for an int param",
         ["-p", "view_cols:=40.0"], "40"),
        ("fullscreen mode",
         ["-p", "view_mode:=fullscreen"], "True"),
    ):
        r = subprocess.run(
            [sys.executable, "-c", snippet, "--ros-args"] + cli,
            capture_output=True, text=True, timeout=60)
        line = [ln for ln in r.stdout.splitlines() if ln.startswith("RESULT")]
        check("14 CLI {0}".format(label),
              r.returncode == 0 and line and all(w in line[0] for w in want.split()),
              "rc={0} out={1} err={2}".format(r.returncode, r.stdout.strip(),
                                              r.stderr.strip()[-300:]))

    rclpy.shutdown()
    print("\n== {0}/{1} passed =={2}".format(
        PASS, PASS + FAIL, "" if not FAIL else "  ({0} FAILED)".format(FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
