#!/usr/bin/env python3
"""vfh_core.VFH must produce EXACTLY the user's original VFH numbers, and the
steering signs must be what the flight node assumes.

ORIGINAL below = the VFH class from the user's vfh_obstacle_avoidance_node.py
(2026-09-14), verbatim (only the class name differs)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from vfh_avoidance_barometer.vfh_core import VFH   # noqa: E402


class OriginalVFH:
    def __init__(self, n_sectors=36, threshold=0.45, smooth_window=2, d_max=5000.0,
                 d_min=300.0, min_valley_width=3, robot_radius=346.0,
                 safety_dist=500.0, hfov_deg=88.0):
        self.n_sectors = n_sectors; self.threshold = threshold
        self.smooth_window = smooth_window; self.d_max = d_max; self.d_min = d_min
        self.min_valley_width = min_valley_width; self.robot_radius = robot_radius
        self.safety_dist = safety_dist; self.hfov_deg = hfov_deg
        self.sector_size_deg = 360.0 / n_sectors
        half = hfov_deg / 2.0
        self.fov_mask = np.zeros(n_sectors, dtype=bool)
        for k in range(n_sectors):
            angle = k * self.sector_size_deg
            if angle > 180.0:
                angle -= 360.0
            if abs(angle) <= half:
                self.fov_mask[k] = True

    def build_polar_histogram(self, depth_map):
        h, w = depth_map.shape
        hist = np.zeros(self.n_sectors, dtype=np.float32)
        hfov_rad = math.radians(self.hfov_deg)
        row_start = int(h * 0.15); row_end = int(h * 0.85)
        col_indices = np.arange(w, dtype=np.float32)
        angles_deg = np.degrees((col_indices / w - 0.5) * hfov_rad)
        col_sectors = (np.round(angles_deg / self.sector_size_deg).astype(int) % self.n_sectors)
        depth_roi = depth_map[row_start:row_end, :]
        valid_mask = (np.isfinite(depth_roi) & (depth_roi > self.d_min) & (depth_roi < self.d_max))
        weight = np.where(valid_mask, (1.0 - depth_roi / self.d_max) ** 2, 0.0).astype(np.float32)
        enlarge_r = self.robot_radius + self.safety_dist
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = np.where(valid_mask, enlarge_r / depth_roi, 0.0)
        enlarge_sec = np.round(np.degrees(np.arcsin(np.clip(ratio, 0.0, 1.0))) / self.sector_size_deg).astype(int)
        for col in range(w):
            k = col_sectors[col]; col_w = weight[:, col]; col_m = valid_mask[:, col]
            total_w = col_w.sum()
            if total_w == 0.0:
                continue
            max_e = int(enlarge_sec[:, col][col_m].max()) if col_m.any() else 0
            for dk in range(-max_e, max_e + 1):
                tk = (k + dk) % self.n_sectors
                if not self.fov_mask[tk]:
                    continue
                fade = 1.0 - abs(dk) / (max_e + 1) if max_e > 0 else 1.0
                hist[tk] += total_w * fade
        hist[~self.fov_mask] = 0.0
        mx = hist.max()
        if mx > 0:
            hist /= mx
        return hist

    def smooth_histogram(self, hist):
        w = self.smooth_window
        if w == 0:
            s = hist.copy(); s[~self.fov_mask] = 0.0; return s
        smoothed = np.zeros_like(hist); n = len(hist)
        for i in range(n):
            if not self.fov_mask[i]:
                continue
            total = 0.0; count = 0
            for d in range(-w, w + 1):
                j = (i + d) % n
                if self.fov_mask[j]:
                    total += hist[j]; count += 1
            smoothed[i] = total / count if count > 0 else 0.0
        return smoothed

    def to_binary(self, smoothed):
        binary = (smoothed >= self.threshold).astype(np.int8)
        binary[~self.fov_mask] = 0
        return binary

    def find_valleys(self, binary):
        n = len(binary); valleys = []
        valid_ordered = sorted([k for k in range(n) if self.fov_mask[k]],
                               key=lambda k: (k * self.sector_size_deg if k * self.sector_size_deg <= 180
                                              else k * self.sector_size_deg - 360))
        run_start_pos = None
        for pos, k in enumerate(valid_ordered):
            if binary[k] == 0:
                if run_start_pos is None:
                    run_start_pos = pos
            else:
                if run_start_pos is not None:
                    if pos - run_start_pos >= self.min_valley_width:
                        valleys.append(self._make_valley(valid_ordered, run_start_pos, pos - 1))
                    run_start_pos = None
        if run_start_pos is not None:
            if len(valid_ordered) - run_start_pos >= self.min_valley_width:
                valleys.append(self._make_valley(valid_ordered, run_start_pos, len(valid_ordered) - 1))
        return valleys

    def _make_valley(self, valid_ordered, start_pos, end_pos):
        def sa(k):
            a = k * self.sector_size_deg
            return a - 360.0 if a > 180.0 else a
        mid_k = valid_ordered[round((start_pos + end_pos) / 2.0)]
        return {"start_idx": valid_ordered[start_pos], "end_idx": valid_ordered[end_pos],
                "width": end_pos - start_pos + 1, "center_deg": round(sa(mid_k), 1)}

    @staticmethod
    def _angle_diff(a, b):
        diff = (a - b) % 360.0
        return diff - 360.0 if diff > 180.0 else diff

    def select_best_valley(self, valleys, target_deg, smoothed_hist, robot_heading=0.0):
        if not valleys:
            return None
        best = None; best_score = float("inf"); best_cand = 0.0
        for v in valleys:
            margin = max(1, self.min_valley_width // 2)
            half_clamp = max(0.0, (v["width"] * self.sector_size_deg / 2) - margin * self.sector_size_deg)
            raw_diff = self._angle_diff(target_deg, v["center_deg"])
            clamped = max(-half_clamp, min(half_clamp, raw_diff))
            candidate = v["center_deg"] + clamped
            angle_err = abs(self._angle_diff(target_deg, candidate))
            score = angle_err - v["width"] * 0.5
            if score < best_score:
                best_score = score; best = dict(v); best_cand = candidate
        if best is None:
            return None
        best["steering_deg"] = round(best_cand, 1)
        return best

    def compute_cmd(self, smoothed_hist, binary_hist, best_valley):
        peak_k = -1; peak_val = 0.0
        for k in range(self.n_sectors):
            if self.fov_mask[k] and smoothed_hist[k] >= self.threshold:
                if smoothed_hist[k] > peak_val:
                    peak_val = smoothed_hist[k]; peak_k = k
        if peak_k == -1:
            obs_dir = "none"
        else:
            pa = peak_k * self.sector_size_deg
            if pa > 180.0: pa -= 360.0
            obs_dir = self._angle_to_label(pa % 360.0)
        ld = rd = 0.0
        for k in range(self.n_sectors):
            if not self.fov_mask[k]: continue
            a = k * self.sector_size_deg
            if a > 180.0: a -= 360.0
            if a < 0: ld += float(smoothed_hist[k])
            elif a > 0: rd += float(smoothed_hist[k])
        if ld == 0.0 and rd == 0.0: avoid_dir = "none"
        elif ld < rd: avoid_dir = "left"
        else: avoid_dir = "right"
        state = "stop" if best_valley is None else ("avoid" if obs_dir != "none" else "move")
        return {"state": state, "obstacle_direction": obs_dir, "avoid_direction": avoid_dir}

    def lateral_proximity_penalty(self, depth_map, steer_deg):
        h, w = depth_map.shape
        row_s = int(h * 0.30); row_e = int(h * 0.70)
        col_edge = max(1, int(w * 0.10))
        danger = self.robot_radius + self.safety_dist + 200.0
        delta = 0.0
        for zone, sign in [(depth_map[row_s:row_e, :col_edge], 1.0), (depth_map[row_s:row_e, w - col_edge:], -1.0)]:
            v = zone[(np.isfinite(zone)) & (zone > self.d_min) & (zone < self.d_max)]
            if v.size > 0:
                mn = float(v.min())
                if mn < danger:
                    delta += sign * (1.0 - mn / danger) * 20.0
        return delta

    def run(self, depth_map, target_deg=0.0, robot_heading=0.0):
        raw_hist = self.build_polar_histogram(depth_map)
        smoothed_hist = self.smooth_histogram(raw_hist)
        binary_hist = self.to_binary(smoothed_hist)
        valleys = self.find_valleys(binary_hist)
        best_valley = self.select_best_valley(valleys, target_deg, smoothed_hist, robot_heading)
        cmd = self.compute_cmd(smoothed_hist, binary_hist, best_valley)
        steer_deg = best_valley.get("steering_deg", 0.0) if best_valley else 0.0
        lat_delta = self.lateral_proximity_penalty(depth_map, steer_deg)
        if abs(lat_delta) > 0.5:
            steer_deg = float(np.clip(steer_deg + lat_delta, -40.0, 40.0))
        return {"raw_hist": raw_hist.tolist(), "smoothed_hist": smoothed_hist.tolist(),
                "binary_hist": binary_hist.tolist(), "valleys": valleys, "best_valley": best_valley,
                "cmd": cmd, "target_deg": target_deg, "steer_deg": round(steer_deg, 1),
                "lateral_delta": round(lat_delta, 1)}

    _DIRECTION_LABELS = [(0, 22, "front"), (23, 67, "front-right"), (68, 112, "right"),
                         (113, 157, "rear-right"), (158, 202, "rear"), (203, 247, "rear-left"),
                         (248, 292, "left"), (293, 337, "front-left"), (338, 360, "front")]

    def _angle_to_label(self, deg):
        deg = deg % 360.0
        for lo, hi, label in self._DIRECTION_LABELS:
            if lo <= deg <= hi:
                return label
        return "front"

    def extract_obstacle_map(self, smoothed_hist, binary_hist):
        n = len(binary_hist); groups = []
        extended_bin = np.concatenate([binary_hist, binary_hist])
        extended_smo = np.concatenate([smoothed_hist, smoothed_hist])
        run_start = None
        for i in range(2 * n):
            if extended_bin[i] == 1:
                if run_start is None:
                    run_start = i
            else:
                if run_start is not None:
                    indices = [j % n for j in range(run_start, i)]
                    seen_idx = set(); unique_idx = []
                    for idx in indices:
                        if idx not in seen_idx:
                            seen_idx.add(idx); unique_idx.append(idx)
                    if unique_idx:
                        densities = [float(extended_smo[j]) for j in range(run_start, i)]
                        peak_density = max(densities); mean_density = sum(densities) / len(densities)
                        start_deg = (run_start % n) * self.sector_size_deg
                        end_deg = ((i - 1) % n) * self.sector_size_deg
                        center_deg = ((run_start + (i - run_start) / 2) % n) * self.sector_size_deg
                        width_deg = len(unique_idx) * self.sector_size_deg
                        is_critical = (0 in unique_idx)
                        groups.append({"angle_start_deg": round(start_deg, 1), "angle_end_deg": round(end_deg, 1),
                                       "angle_center_deg": round(center_deg, 1), "width_deg": round(width_deg, 1),
                                       "peak_density": round(peak_density, 3), "mean_density": round(mean_density, 3),
                                       "sector_indices": unique_idx, "relative_pos": self._angle_to_label(center_deg),
                                       "is_critical": is_critical})
                    run_start = None
        if run_start is not None:
            indices = [j % n for j in range(run_start, 2 * n)]
            seen_idx = set(); unique_idx = []
            for idx in indices:
                if idx not in seen_idx:
                    seen_idx.add(idx); unique_idx.append(idx)
            if unique_idx:
                densities = [float(extended_smo[j]) for j in range(run_start, 2 * n)]
                peak_density = max(densities); mean_density = sum(densities) / len(densities)
                start_deg = (run_start % n) * self.sector_size_deg
                end_deg = ((2 * n - 1) % n) * self.sector_size_deg
                center_deg = ((run_start + (2 * n - run_start) / 2) % n) * self.sector_size_deg
                width_deg = len(unique_idx) * self.sector_size_deg
                is_critical = (0 in unique_idx)
                groups.append({"angle_start_deg": round(start_deg, 1), "angle_end_deg": round(end_deg, 1),
                               "angle_center_deg": round(center_deg, 1), "width_deg": round(width_deg, 1),
                               "peak_density": round(peak_density, 3), "mean_density": round(mean_density, 3),
                               "sector_indices": unique_idx, "relative_pos": self._angle_to_label(center_deg),
                               "is_critical": is_critical})
        seen_keys = set(); unique_groups = []
        for g in groups:
            key = tuple(sorted(g["sector_indices"]))
            if key not in seen_keys:
                seen_keys.add(key); unique_groups.append(g)
        unique_groups.sort(key=lambda g: g["peak_density"], reverse=True)
        return unique_groups


# ── synthetic depth scenes (mm) ────────────────────────────────────────────
H, W, HFOV, VFOV = 480, 640, 91.0, 66.0          # Orbbec Gemini 2 depth FOV
FX = W / (2.0 * math.tan(math.radians(HFOV) / 2.0))
FY = H / (2.0 * math.tan(math.radians(VFOV) / 2.0))


def render_scene(poles, alt_m=2.0, ground=True, invalid_mm=10000.0, noise=None, rng=None):
    """Depth (mm) seen from the origin looking +x, poles = [(x, y_left, r)] in m.
    Camera convention: +y = left of the image centre = LEFT columns."""
    u = np.arange(W, dtype=np.float64)
    v = np.arange(H, dtype=np.float64)
    tan_h = (u - W / 2.0) / FX          # + = right
    tan_v = (v - H / 2.0) / FY          # + = down
    depth = np.full((H, W), invalid_mm, dtype=np.float64)
    # ground plane at -alt
    if ground:
        with np.errstate(divide="ignore"):
            zg = np.where(tan_v > 1e-6, alt_m / tan_v, np.inf)
        depth = np.minimum(depth, np.repeat(zg[:, None], W, axis=1) * 1000.0)
    # vertical cylinders: ray in the horizontal plane (dx=1, dy=-tan_h)
    for (px, py, r) in poles:
        dx = np.ones_like(tan_h); dy = -tan_h
        nrm = np.sqrt(dx * dx + dy * dy); dx /= nrm; dy /= nrm
        b = -(px * dx + py * dy)
        c = px * px + py * py - r * r
        disc = b * b - c
        hit = disc >= 0
        t = -b - np.sqrt(np.where(hit, disc, 0.0))
        ok = hit & (t > 0)
        z = np.where(ok, t * dx, np.inf)           # z-depth along the axis
        depth = np.minimum(depth, np.repeat(z[None, :], H, axis=0) * 1000.0)
    depth = depth.astype(np.float32)
    if noise is not None:
        depth = depth + rng.normal(0.0, noise, depth.shape).astype(np.float32)
    return depth


def main():
    rng = np.random.default_rng(3)
    fails = 0
    n = 0

    def check(name, cond, info=""):
        nonlocal fails, n
        n += 1
        print(f"  [{'OK ' if cond else 'FAIL'}] {name} {info}")
        if not cond:
            fails += 1

    print("== 1. vfh_core == original (bitwise on the same input) ==")
    a = VFH(hfov_deg=HFOV); b = OriginalVFH(hfov_deg=HFOV)
    render = render_scene
    scenes = {
        "open": render([]),
        "pole_front": render([(2.5, 0.0, 0.3)]),
        "pole_left": render([(2.5, 0.8, 0.3)]),
        "pole_right": render([(2.5, -0.8, 0.3)]),
        "two_poles": render([(2.0, 0.9, 0.25), (3.0, -0.7, 0.25)]),
        "wall": render([(2.0, y, 0.35) for y in np.arange(-3.0, 3.01, 0.3)]),
        "close_edge": render([(1.0, 1.1, 0.3)]),
        "noisy": render([(2.5, 0.4, 0.3)], noise=30.0, rng=rng),
        "nan_holes": None,
        "random": None,
    }
    d = render([(2.5, 0.2, 0.3)]); m = rng.random(d.shape) < 0.3; d[m] = np.nan
    scenes["nan_holes"] = d
    scenes["random"] = (rng.random((H, W)) * 8000.0).astype(np.float32)
    for name, depth in scenes.items():
        for tgt in (0.0, 20.0, 340.0):
            ra = a.run(depth.copy(), tgt); rb = b.run(depth.copy(), tgt)
            same = (np.array_equal(ra["raw_hist"], rb["raw_hist"])
                    and np.array_equal(ra["smoothed_hist"], rb["smoothed_hist"])
                    and ra["binary_hist"] == rb["binary_hist"]
                    and ra["valleys"] == rb["valleys"] and ra["best_valley"] == rb["best_valley"]
                    and ra["cmd"] == rb["cmd"] and ra["steer_deg"] == rb["steer_deg"]
                    and ra["lateral_delta"] == rb["lateral_delta"])
            oa = a.extract_obstacle_map(np.array(ra["smoothed_hist"]), np.array(ra["binary_hist"]))
            ob = b.extract_obstacle_map(np.array(rb["smoothed_hist"]), np.array(rb["binary_hist"]))
            check(f"{name} target={tgt}", same and oa == ob,
                  f"state={ra['cmd']['state']} steer={ra['steer_deg']} valleys={len(ra['valleys'])}")
    # random binary histograms for extract_obstacle_map
    for i in range(30):
        bh = (rng.random(36) < 0.4).astype(np.int8); bh[~a.fov_mask] = 0
        sh = rng.random(36).astype(np.float32) * bh
        check(f"obstacle_map random {i}", a.extract_obstacle_map(sh, bh) == b.extract_obstacle_map(sh, bh))

    print("== 2. steering semantics the flight node relies on (ground filtered) ==")
    r = a.run(render_scene([], ground=True), 0.0)
    print(f"     (info) UNFILTERED ground at 2.0 m (ROI 15-85 %): state={r['cmd']['state']} "
          f"blocked={sum(r['binary_hist'])} front_min={r['front_min_mm']} mm "
          "-> this is why the node has the barometric ground filter")
    def render(poles, **kw):          # noqa: E306  (section 2: ground removed, as the node does)
        kw.setdefault("ground", False)
        return render_scene(poles, **kw)
    r = a.run(render([]), 0.0)
    check("open field -> move, steer 0", r["cmd"]["state"] == "move" and r["steer_deg"] == 0.0, str(r["cmd"]))
    check("open field front_min None", r["front_min_mm"] is None, str(r["front_min_mm"]))
    r = a.run(render([(2.5, 0.8, 0.3)]), 0.0)
    check("pole LEFT -> steer > 0 (turn right)", r["steer_deg"] > 0, f"steer={r['steer_deg']} {r['cmd']}")
    r = a.run(render([(2.5, -0.8, 0.3)]), 0.0)
    check("pole RIGHT -> steer < 0 (turn left)", r["steer_deg"] < 0, f"steer={r['steer_deg']} {r['cmd']}")
    r = a.run(render([(2.5, 0.0, 0.3)]), 0.0)
    check("pole FRONT -> avoid, steer != 0", r["cmd"]["state"] == "avoid" and r["steer_deg"] != 0,
          f"steer={r['steer_deg']} {r['cmd']}")
    check("pole FRONT front_min ~2.2 m", r["front_min_mm"] is not None and 2100 <= r["front_min_mm"] <= 2300,
          str(r["front_min_mm"]))
    r = a.run(render([(2.0, y, 0.35) for y in np.arange(-3.0, 3.01, 0.3)]), 0.0)
    check("WALL across -> stop", r["cmd"]["state"] == "stop", str(r["cmd"]))
    r = a.run(render([]), 30.0)
    check("open, target +30 (right) -> steer ~ +30", 20 <= r["steer_deg"] <= 40, str(r["steer_deg"]))
    r = a.run(render([]), 330.0)
    check("open, target 330 (= -30 left) -> steer ~ -30", -40 <= r["steer_deg"] <= -20, str(r["steer_deg"]))
    # ROI parameter: a narrower band keeps the ground (unfiltered) out at 2 m
    a3 = VFH(hfov_deg=HFOV, roi_top=0.35, roi_bottom=0.65)
    r = a3.run(render_scene([], ground=True), 0.0)
    check("roi 0.35-0.65: unfiltered ground at 2 m -> move", r["cmd"]["state"] == "move", str(r["cmd"]))
    r = a3.run(render_scene([(2.5, 0.8, 0.3)], ground=True), 0.0)
    check("roi 0.35-0.65: pole LEFT with ground -> steer > 0", r["steer_deg"] > 0, str(r["steer_deg"]))
    # camera_info hfov update
    a2 = VFH(hfov_deg=60.0); n60 = int(a2.fov_mask.sum()); a2.set_hfov(91.0)
    check("set_hfov rebuilds the FOV mask", n60 == 7 and int(a2.fov_mask.sum()) == 9, f"{n60} -> {int(a2.fov_mask.sum())}")
    # timing
    import time
    d = render([(2.0, 0.9, 0.25), (3.0, -0.7, 0.25)])
    t0 = time.monotonic()
    for _ in range(10):
        a.run(d, 0.0)
    ms = (time.monotonic() - t0) / 10 * 1000.0
    t0 = time.monotonic()
    for _ in range(3):
        b.run(d, 0.0)
    ms_b = (time.monotonic() - t0) / 3 * 1000.0
    print(f"     (info) VFH.run 640x480: vfh_core {ms:.1f} ms, original {ms_b:.1f} ms")
    check("VFH.run < 150 ms on this Jetson", ms < 150.0, f"{ms:.1f} ms")

    print(f"\n{n - fails}/{n} passed")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
