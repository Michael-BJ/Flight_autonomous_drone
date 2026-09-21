#!/usr/bin/env python3
"""
vfh_core.py — VFH (Vector Field Histogram) obstacle avoidance, pure NumPy.
==========================================================================
This is the VFH class the user supplied (2026-09-14), kept as written so the
algorithm behaves exactly like the original `vfh_obstacle_avoidance_node.py`.
Only additions (tagged NEW-VFH): `front_min()` — the closest valid depth in
the central column band, which the flight node uses as a HARD BRAKE that
does not depend on the histogram threshold; `set_hfov()`; and the vertical
ROI as parameters (roi_top/roi_bottom, defaults = the original 15 %-85 %).

All distances are MILLIMETERS. Coordinates: 0 = straight ahead,
positive = RIGHT (clockwise from the top), negative = LEFT.
Drone: 49 cm x 49 cm -> robot_radius 346 mm (half the diagonal).
"""
import math

import numpy as np


class VFH:
    """
    Vector Field Histogram obstacle avoidance (classic).

    Pipeline:
      1. build_polar_histogram  — POD(k) with robot enlargement
      2. smooth_histogram       — moving average
      3. to_binary              — single threshold
      4. find_valleys           — free-sector run >= min_valley_width
      5. select_best_valley     — score = angle_err - width*0.5
      6. compute_cmd            — state: move/avoid/stop
      7. lateral_proximity_penalty — FOV edge correction
    """

    def __init__(
        self,
        n_sectors:        int   = 36,
        threshold:        float = 0.45,
        smooth_window:    int   = 2,
        d_max:            float = 5000.0,
        d_min:            float = 300.0,
        min_valley_width: int   = 3,
        robot_radius:     float = 346.0,
        safety_dist:      float = 500.0,
        hfov_deg:         float = 88.0,
        roi_top:          float = 0.15,   # NEW-VFH: vertical ROI (fractions of h)
        roi_bottom:       float = 0.85,   #          original values 15 %-85 %
    ) -> None:
        self.roi_top          = float(roi_top)
        self.roi_bottom       = float(roi_bottom)
        self.n_sectors        = n_sectors
        self.threshold        = threshold
        self.smooth_window    = smooth_window
        self.d_max            = d_max
        self.d_min            = d_min
        self.min_valley_width = min_valley_width
        self.robot_radius     = robot_radius
        self.safety_dist      = safety_dist
        self.hfov_deg         = hfov_deg
        self.sector_size_deg  = 360.0 / n_sectors
        self.set_hfov(hfov_deg)

    def set_hfov(self, hfov_deg: float) -> None:
        """NEW-VFH: (re)build the FOV mask; the node calls this once the real
        camera intrinsics arrive."""
        self.hfov_deg = float(hfov_deg)
        n_sectors = self.n_sectors
        half = self.hfov_deg / 2.0
        self.fov_mask = np.zeros(n_sectors, dtype=bool)
        for k in range(n_sectors):
            angle = k * self.sector_size_deg
            if angle > 180.0:
                angle -= 360.0
            if abs(angle) <= half:
                self.fov_mask[k] = True

        valid_indices = np.where(self.fov_mask)[0]
        self.fov_sector_right = int(valid_indices[-1])
        left_valid = [k for k in range(n_sectors)
                      if self.fov_mask[k] and k > n_sectors // 2]
        self.fov_sector_left = int(left_valid[0]) if left_valid else 0

    # ------------------------------------------------------------------ #
    # Step 1 — Polar histogram with robot enlargement                    #
    # ------------------------------------------------------------------ #

    def build_polar_histogram(self, depth_map: np.ndarray) -> np.ndarray:
        """
        POD(k) = sum[(1 - d/d_max)^2] per sector, with enlargement:
            arcsin((robot_radius + safety_dist) / d) sectors to the left/right.
        Vertical ROI 15%-85% so tree trunks are read.
        """
        h, w      = depth_map.shape
        hist      = np.zeros(self.n_sectors, dtype=np.float32)
        hfov_rad  = math.radians(self.hfov_deg)
        row_start = int(h * self.roi_top)      # NEW-VFH: was 0.15 / 0.85 fixed
        row_end   = int(h * self.roi_bottom)

        col_indices = np.arange(w, dtype=np.float32)
        angles_deg  = np.degrees((col_indices / w - 0.5) * hfov_rad)
        col_sectors = (
            np.round(angles_deg / self.sector_size_deg).astype(int) % self.n_sectors
        )

        depth_roi  = depth_map[row_start:row_end, :]
        valid_mask = (
            np.isfinite(depth_roi)
            & (depth_roi > self.d_min)
            & (depth_roi < self.d_max)
        )
        weight = np.where(
            valid_mask,
            (1.0 - depth_roi / self.d_max) ** 2,
            0.0,
        ).astype(np.float32)

        enlarge_r = self.robot_radius + self.safety_dist
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = np.where(valid_mask, enlarge_r / depth_roi, 0.0)
        enlarge_sec = np.round(
            np.degrees(np.arcsin(np.clip(ratio, 0.0, 1.0))) / self.sector_size_deg
        ).astype(int)

        # NEW-VFH: the loop only visits columns that contain valid pixels
        # (weights are >= 0, so a zero axis-sum means an all-zero column).
        # The per-column sum itself is the original expression, so the
        # numbers are bit-identical to the user's loop.
        max_e_col = np.where(valid_mask, enlarge_sec, 0).max(axis=0)
        for col in np.nonzero(weight.sum(axis=0) > 0.0)[0]:
            k       = int(col_sectors[col])
            total_w = weight[:, col].sum()
            max_e   = int(max_e_col[col])
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

    # ------------------------------------------------------------------ #
    # Step 2 — Smooth histogram                                           #
    # ------------------------------------------------------------------ #

    def smooth_histogram(self, hist: np.ndarray) -> np.ndarray:
        w = self.smooth_window
        if w == 0:
            s = hist.copy(); s[~self.fov_mask] = 0.0; return s
        smoothed = np.zeros_like(hist)
        n = len(hist)
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

    # ------------------------------------------------------------------ #
    # Step 3 — Binary histogram (single threshold)                        #
    # ------------------------------------------------------------------ #

    def to_binary(self, smoothed: np.ndarray) -> np.ndarray:
        """1 = blocked (>= threshold), 0 = free. Blind sectors always 0."""
        binary = (smoothed >= self.threshold).astype(np.int8)
        binary[~self.fov_mask] = 0
        return binary

    # ------------------------------------------------------------------ #
    # Step 4 — Valley detection                                           #
    # ------------------------------------------------------------------ #

    def find_valleys(self, binary: np.ndarray) -> list:
        """Find contiguous free (0) runs >= min_valley_width within the FOV."""
        n       = len(binary)
        valleys = []
        valid_ordered = sorted(
            [k for k in range(n) if self.fov_mask[k]],
            key=lambda k: (k * self.sector_size_deg
                           if k * self.sector_size_deg <= 180
                           else k * self.sector_size_deg - 360)
        )

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
        return {
            "start_idx":  valid_ordered[start_pos],
            "end_idx":    valid_ordered[end_pos],
            "width":      end_pos - start_pos + 1,
            "center_deg": round(sa(mid_k), 1),
        }

    # ------------------------------------------------------------------ #
    # Step 5 — Select best valley                                         #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        diff = (a - b) % 360.0
        return diff - 360.0 if diff > 180.0 else diff

    def select_best_valley(
        self,
        valleys:       list,
        target_deg:    float,
        smoothed_hist: np.ndarray,
        robot_heading: float = 0.0,
    ):
        """
        Classic VFH: score = angle_err - width * 0.5
        The valley with the smallest score is selected.
        """
        if not valleys:
            return None

        best = None; best_score = float("inf"); best_cand = 0.0
        for v in valleys:
            margin      = max(1, self.min_valley_width // 2)
            half_clamp  = max(0.0, (v["width"] * self.sector_size_deg / 2)
                                   - margin * self.sector_size_deg)
            raw_diff    = self._angle_diff(target_deg, v["center_deg"])
            clamped     = max(-half_clamp, min(half_clamp, raw_diff))
            candidate   = v["center_deg"] + clamped

            angle_err = abs(self._angle_diff(target_deg, candidate))
            score     = angle_err - v["width"] * 0.5
            if score < best_score:
                best_score = score; best = dict(v); best_cand = candidate

        if best is None:
            return None
        best["steering_deg"] = round(best_cand, 1)
        return best

    # ------------------------------------------------------------------ #
    # Step 6 — Compute command                                            #
    # ------------------------------------------------------------------ #

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
        elif ld < rd:                avoid_dir = "left"
        else:                        avoid_dir = "right"

        state = "stop" if best_valley is None else ("avoid" if obs_dir != "none" else "move")
        return {"state": state, "obstacle_direction": obs_dir, "avoid_direction": avoid_dir}

    def lateral_proximity_penalty(self, depth_map: np.ndarray, steer_deg: float) -> float:
        """Correct steering by ±20° if an obstacle is very close at the FOV edge."""
        h, w     = depth_map.shape
        row_s    = int(h * 0.30); row_e = int(h * 0.70)
        col_edge = max(1, int(w * 0.10))
        danger   = self.robot_radius + self.safety_dist + 200.0
        delta    = 0.0
        for zone, sign in [(depth_map[row_s:row_e, :col_edge], 1.0),
                           (depth_map[row_s:row_e, w - col_edge:], -1.0)]:
            v = zone[(np.isfinite(zone)) & (zone > self.d_min) & (zone < self.d_max)]
            if v.size > 0:
                mn = float(v.min())
                if mn < danger:
                    delta += sign * (1.0 - mn / danger) * 20.0
        return delta

    # NEW-VFH ------------------------------------------------------------- #
    def front_min(self, depth_map: np.ndarray, half_angle_deg: float = 12.0):
        """Closest valid depth (mm) in the columns within +-half_angle_deg of
        the optical axis, rows 15-85 %. None when nothing valid is there.
        A hard brake for the flight node, independent of the histogram."""
        h, w = depth_map.shape
        frac = min(0.5, half_angle_deg / max(1e-6, self.hfov_deg))
        c0 = int(w * (0.5 - frac)); c1 = max(c0 + 1, int(w * (0.5 + frac)))
        zone = depth_map[int(h * self.roi_top):int(h * self.roi_bottom), c0:c1]
        v = zone[np.isfinite(zone) & (zone > self.d_min) & (zone < self.d_max)]
        return float(v.min()) if v.size > 0 else None

    # ------------------------------------------------------------------ #
    # Full VFH pipeline                                                   #
    # ------------------------------------------------------------------ #

    def run(self, depth_map: np.ndarray, target_deg: float = 0.0,
            robot_heading: float = 0.0) -> dict:
        """Classic VFH pipeline: PHD -> Smooth -> Binary -> Valleys -> select -> cmd."""
        raw_hist      = self.build_polar_histogram(depth_map)
        smoothed_hist = self.smooth_histogram(raw_hist)
        binary_hist   = self.to_binary(smoothed_hist)
        valleys       = self.find_valleys(binary_hist)
        best_valley   = self.select_best_valley(valleys, target_deg, smoothed_hist, robot_heading)
        cmd           = self.compute_cmd(smoothed_hist, binary_hist, best_valley)

        steer_deg = best_valley.get("steering_deg", 0.0) if best_valley else 0.0
        lat_delta = self.lateral_proximity_penalty(depth_map, steer_deg)
        if abs(lat_delta) > 0.5:
            steer_deg = float(np.clip(steer_deg + lat_delta, -40.0, 40.0))

        fmin = self.front_min(depth_map)   # NEW-VFH

        return {
            "raw_hist":      raw_hist.tolist(),
            "smoothed_hist": smoothed_hist.tolist(),
            "binary_hist":   binary_hist.tolist(),
            "valleys":       valleys,
            "best_valley":   best_valley,
            "cmd":           cmd,
            "target_deg":    target_deg,
            "steer_deg":     round(steer_deg, 1),
            "lateral_delta": round(lat_delta, 1),
            "front_min_mm":  None if fmin is None else round(fmin, 0),   # NEW-VFH
        }

    _DIRECTION_LABELS = [
        (  0,  22, "front"),
        ( 23,  67, "front-right"),
        ( 68, 112, "right"),
        (113, 157, "rear-right"),
        (158, 202, "rear"),
        (203, 247, "rear-left"),
        (248, 292, "left"),
        (293, 337, "front-left"),
        (338, 360, "front"),
    ]

    def _angle_to_label(self, deg: float) -> str:
        deg = deg % 360.0
        for lo, hi, label in self._DIRECTION_LABELS:
            if lo <= deg <= hi:
                return label
        return "front"

    def extract_obstacle_map(
        self,
        smoothed_hist: np.ndarray,
        binary_hist:   np.ndarray,
    ) -> list:
        n      = len(binary_hist)
        groups = []

        extended_bin  = np.concatenate([binary_hist,  binary_hist])
        extended_smo  = np.concatenate([smoothed_hist, smoothed_hist])

        def _group(run_start, run_end):
            indices = [j % n for j in range(run_start, run_end)]
            seen_idx = set()
            unique_idx = []
            for idx in indices:
                if idx not in seen_idx:
                    seen_idx.add(idx)
                    unique_idx.append(idx)
            if not unique_idx:
                return None
            densities    = [float(extended_smo[j]) for j in range(run_start, run_end)]
            peak_density = max(densities)
            mean_density = sum(densities) / len(densities)
            start_deg  = (run_start % n) * self.sector_size_deg
            end_deg    = ((run_end - 1) % n) * self.sector_size_deg
            center_deg = ((run_start + (run_end - run_start) / 2) % n) * self.sector_size_deg
            width_deg  = len(unique_idx) * self.sector_size_deg
            return {
                "angle_start_deg":  round(start_deg,  1),
                "angle_end_deg":    round(end_deg,    1),
                "angle_center_deg": round(center_deg, 1),
                "width_deg":        round(width_deg,  1),
                "peak_density":     round(peak_density, 3),
                "mean_density":     round(mean_density, 3),
                "sector_indices":   unique_idx,
                "relative_pos":     self._angle_to_label(center_deg),
                "is_critical":      (0 in unique_idx),
            }

        run_start = None
        for i in range(2 * n):
            if extended_bin[i] == 1:
                if run_start is None:
                    run_start = i
            else:
                if run_start is not None:
                    g = _group(run_start, i)
                    if g is not None:
                        groups.append(g)
                    run_start = None
        if run_start is not None:
            g = _group(run_start, 2 * n)
            if g is not None:
                groups.append(g)

        seen_keys = set()
        unique_groups = []
        for g in groups:
            key = tuple(sorted(g["sector_indices"]))
            if key not in seen_keys:
                seen_keys.add(key)
                unique_groups.append(g)

        unique_groups.sort(key=lambda g: g["peak_density"], reverse=True)
        return unique_groups
