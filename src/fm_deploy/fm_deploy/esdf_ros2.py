#!/usr/bin/env python3
"""
esdf_ros2.py
============
ESDF (Euclidean Signed Distance Field) for ROS2, built from the OccupancyGrid
published by the OctoMap server.

This is a direct port of the original NEO-Planner esdf.py (ROS1) to ROS2,
with adjustments for integration with expert_planner_node.

How it works:
  1. OctoMap server subscribes to /pointcloud/map (from publish_pcd_node)
  2. OctoMap server publishes /projected_map (2D OccupancyGrid)
  3. ESDF subscribes to /projected_map and computes the distance field
  4. The expert planner uses the ESDF for collision cost in the MINCO optimizer

Difference from the old esdf_builder.py:
  - esdf_builder.py  : loads from a .npz file, static, never updates
  - esdf_ros2.py     : subscribes to /projected_map, updates in real time
                       because OctoMap keeps receiving new point clouds

API (same as the original esdf.py to stay compatible with expert_planner):
  esdf.get_edt_dis(pos)   -> float  : distance to the nearest obstacle (meters)
  esdf.get_edt_grad(pos)  -> [gx,gy]: distance gradient
  esdf.has_collision(pos) -> bool   : True if too close to an obstacle
  esdf.is_ready()         -> bool   : True if a map has been received
"""

import threading

import numpy as np
from scipy import ndimage

import rclpy
from nav_msgs.msg import OccupancyGrid

# Default safe distance (meters) — same as the original repo
SAFE_DIS = 0.5


class ESDF:
    """
    ESDF from an OccupancyGrid (projected_map from the OctoMap server).

    Used by MinJerkPlanner to compute the collision cost and its gradient during
    trajectory optimization.

    Thread-safe: the OccupancyGrid callback and the optimizer queries run on
    different threads.
    """

    def __init__(self, inflation_radius: float = 0.25, arena_bounds=None):
        # Map data — filled by occupancy_map_cb
        self.map_resolution = 0.1
        self.map_width      = 0
        self.map_height     = 0
        self.map_origin     = None    # geometry_msgs/Point

        # VIRTUAL ARENA WALL (2026-07-07, optional — only the expert uses
        # it; the inference node is unchanged). (x_min, x_max, y_min,
        # y_max) world: cells OUTSIDE the bounds are treated as occupied
        # before the EDT. Reason: the octomap grid spans the full pcd
        # extent (y ±15) while the arena is only y ±6; without this wall
        # the MINCO collision cost is zero outside the arena, so the
        # optimizer/escape is free to push the trajectory out of bounds
        # (repeated out-of-bounds + an explosion in run_20260707_132401
        # even though the A* seed was already bounded). With the virtual
        # wall: MINCO steers away from the edge, _check_trajectory_safe
        # rejects out-of-arena labels, and escape direction automatically
        # returns inward. Inflation also applies to the virtual wall
        # (effectively ~0.3 m inward).
        self.arena_bounds = (tuple(float(b) for b in arena_bounds)
                             if arena_bounds else None)

        # FIX COLLISION #2: inflate obstacles by the drone body radius.
        # The original ESDF treats the drone as a POINT — distance is computed
        # from the drone center to the obstacle cell. The x500 drone is ~0.5m in
        # diameter + propellers, so a 0.2-0.4m clearance from the drone CENTER
        # means the body is already colliding. With inflation, all queries
        # (get_edt_dis / has_collision / escape / forward-guard) automatically
        # measure the distance from the drone BODY to the obstacle.
        self.inflation_radius = float(inflation_radius)

        self.occupancy_2d   = None    # np.ndarray (height x width), 0=free 1=occ (RAW)
        self.unknown_2d     = None    # np.ndarray, 1 = never observed (see cb)
        self.esdf_map       = None    # np.ndarray (height x width), distance in meters
        self.esdf_grad_x    = None    # np.ndarray (height x width)
        self.esdf_grad_y    = None    # np.ndarray (height x width)

        self._lock          = threading.Lock()
        self._received      = False   # flag: whether a map has been received yet

    def is_ready(self) -> bool:
        """Return True if a map has been received and the ESDF has been built."""
        return self._received

    def occupancy_map_cb(self, map_msg: OccupancyGrid):
        """
        Callback for /projected_map from the OctoMap server.

        Called every time the OctoMap server publishes a new OccupancyGrid.
        Each call rebuilds the ESDF from scratch.

        Flow:
          OccupancyGrid (1D int8, -1/0/100)
            -> binary occupancy (0=free, 1=occ)
            -> ESDF via scipy distance_transform_edt
            -> ESDF gradient via np.gradient
        """
        with self._lock:
            self.map_resolution = map_msg.info.resolution
            self.map_width      = map_msg.info.width
            self.map_height     = map_msg.info.height
            self.map_origin     = map_msg.info.origin.position

            # Convert the flat 1D data to a 2D binary grid
            # OccupancyGrid values: 100=occupied, 0=free, -1=unknown
            # Treat unknown (-1) as free (0) per the original repo
            raw = np.array(map_msg.data, dtype=np.int8)
            binary = (raw == 100).astype(np.uint8)
            self.occupancy_2d = binary.reshape(
                self.map_height, self.map_width)
            # 2026-07-27: keep the UNOBSERVED cells too. The occupancy above
            # folds unknown (-1) into "free", which is right for the MINCO cost
            # (you cannot penalise what has not been seen, or the drone never
            # advances) but WRONG for the reactive guards: a wall the octomap
            # has not inserted yet reads as open space, and the planner happily
            # commits to it. MEASURED (gauntlet_train2, 2026-07-27): every
            # replan up to impact succeeded with a clean cost (4.3, 4.6) while
            # the drone flew straight into the G2 wall at 1.26 m/s, perfectly
            # stable (z=1.94, vz=0.00); the collision guard only fired 0.6 s
            # AFTER contact. See is_unobserved() for how the guards use this.
            self.unknown_2d = (raw == -1).astype(np.uint8).reshape(
                self.map_height, self.map_width)

            # DINDING VIRTUAL ARENA: tandai sel di luar batas sebagai occupied
            # (pusat sel = origin + (i+0.5)*res, konsisten get_edt_dis).
            if self.arena_bounds is not None:
                bx0, bx1, by0, by1 = self.arena_bounds
                res = self.map_resolution
                xc = (self.map_origin.x + (np.arange(self.map_width) + 0.5)
                      * res)
                yc = (self.map_origin.y + (np.arange(self.map_height) + 0.5)
                      * res)
                out_col = (xc < bx0) | (xc > bx1)
                out_row = (yc < by0) | (yc > by1)
                self.occupancy_2d = self.occupancy_2d.copy()
                self.occupancy_2d[out_row, :] = 1
                self.occupancy_2d[:, out_col] = 1

            # FIX COLLISION #2: inflate obstacles by the drone radius before EDT.
            # Binary dilation with a circular kernel of radius = inflation_radius.
            occ_for_edt = self.occupancy_2d
            n_cells = int(np.ceil(self.inflation_radius / self.map_resolution))
            if n_cells > 0:
                yy, xx = np.ogrid[-n_cells:n_cells + 1, -n_cells:n_cells + 1]
                kernel = (xx * xx + yy * yy) <= n_cells * n_cells
                occ_for_edt = ndimage.binary_dilation(
                    self.occupancy_2d, structure=kernel).astype(np.uint8)

            # Build ESDF: distance_transform_edt computes the distance to the
            # nearest obstacle for every free cell
            dist_in_cells = ndimage.distance_transform_edt(
                1 - occ_for_edt)
            self.esdf_map = dist_in_cells * self.map_resolution

            # ESDF gradient (for the MINCO collision gradient)
            # FIX COLLISION #1 (FATAL BUG): np.gradient WITHOUT a spacing argument
            # produces a per-CELL gradient, not per-METER. At 0.2m resolution the
            # gradient becomes 5x too small (0.15m -> 6.7x). As a result the
            # collision repulsion force in the MINCO optimizer is much weaker than
            # its cost -> the trajectory is left hugging/penetrating obstacles.
            # Spacing = map_resolution makes |∇d| ≈ 1 (correct for a distance field).
            self.esdf_grad_y, self.esdf_grad_x = np.gradient(
                self.esdf_map, self.map_resolution)

            if not self._received:
                self._received = True

    def _world_to_index(self, pos):
        """
        Convert world coordinates (x, y) to array indices (row, col).

        OccupancyGrid origin = bottom-left corner of the grid.
        row = (y - origin_y) / resolution  -> Y direction = row direction
        col = (x - origin_x) / resolution  -> X direction = col direction
        """
        x, y = pos[0], pos[1]
        row = int((y - self.map_origin.y) / self.map_resolution)
        col = int((x - self.map_origin.x) / self.map_resolution)
        return row, col

    def _in_bounds(self, row: int, col: int) -> bool:
        return (0 <= row < self.map_height and
                0 <= col < self.map_width)

    def get_edt_dis(self, pos) -> float:
        """
        Query the distance to the nearest obstacle at position (x, y).

        Args:
            pos: [x, y] in meters, world frame

        Returns:
            float: distance in meters
                   10000.0 if the position is outside the map bounds (considered safe)
                   0.0 if the position is inside an obstacle
        """
        with self._lock:
            if not self._received:
                return 10000.0
            row, col = self._world_to_index(pos)
            if not self._in_bounds(row, col):
                return 10000.0
            return float(self.esdf_map[row, col])

    def is_unobserved(self, pos) -> bool:
        """True if (x, y) has NOT been observed yet — either outside the mapped
        grid entirely, or a cell the octomap still marks unknown (-1).

        Deliberately NOT folded into get_edt_dis(): that stays optimistic
        (10000 = safe) so the MINCO cost can still plan into unseen space,
        which is what lets the drone advance at all. This is for the reactive
        guards, which must refuse to FLY into space the drone has not actually
        looked at yet. Two measured failures motivate it (2026-07-27,
        gauntlet_train2): (1) the drone flew into the G2 wall while every
        replan reported a clean cost, because the wall was still unknown =
        "free"; (2) after the resulting contact threw it out of the mapped
        region, get_edt_dis returned 10000 for 60+ s, so nothing stopped it
        flying blind through more walls."""
        with self._lock:
            if not self._received:
                return True
            row, col = self._world_to_index(pos)
            if not self._in_bounds(row, col):
                return True
            if self.unknown_2d is None:
                return False
            return bool(self.unknown_2d[row, col])

    def get_edt_grad(self, pos) -> list:
        """
        Query the ESDF gradient at position (x, y).

        Returns:
            [gx, gy]: gradient in meters/meter
                      [0, 0] if out of bounds
        """
        with self._lock:
            if not self._received:
                return [0.0, 0.0]
            row, col = self._world_to_index(pos)
            if not self._in_bounds(row, col):
                return [0.0, 0.0]
            return [
                float(self.esdf_grad_x[row, col]),
                float(self.esdf_grad_y[row, col]),
            ]

    def has_collision(self, pos) -> bool:
        """
        Check whether a position is too close to an obstacle.

        Returns:
            True if the distance to the obstacle < SAFE_DIS
        """
        return self.get_edt_dis(pos) < SAFE_DIS

    def is_occupied(self, pos) -> bool:
        """Check whether the cell at position (x, y) is occupied by an obstacle."""
        with self._lock:
            if not self._received:
                return False
            row, col = self._world_to_index(pos)
            if not self._in_bounds(row, col):
                return False
            return bool(self.occupancy_2d[row, col])
