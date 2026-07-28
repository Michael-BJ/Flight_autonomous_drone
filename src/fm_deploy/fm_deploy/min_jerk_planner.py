#!/usr/bin/env python3
"""
min_jerk_planner.py
===================
Direct port of the original NEO-Planner expert_planner.py + traj_utils.py
(ROS1) to pure Python compatible with ROS2.

No changes to the algorithm — only:
  1. Removed the rospy dependency (not used here)
  2. Merged TrajUtils and MinJerkPlanner into a single file
  3. Added explanatory docstrings for each key function

FIX (vs the previous version):
  - batch_generate_init_variables: the sinusoidal bump now uses the INTERIOR
    points (linspace(0, pi, n+2)[1:-1]) so for n=2 it produces a real offset.
    Previously linspace(0, pi, n) included the endpoints so sin = 0 and the
    three candidates (straight/left/right) collapsed to all-straight — the
    Section II-D multimodality was lost.
  - Lateral direction comments (right/left) fixed to match the rotation.

How MINCO (Minimum Control effort) works:
  - Trajectory = M piece polynomial p(t), degree 5 (minimum jerk)
  - Parameterization: Q (intermediate waypoints) + T (time per piece)
  - Optimization: minimize J = w_e*effort + w_t*time + w_f*feasibility + w_c*collision
  - Solver: L-BFGS-B via scipy.optimize.minimize
  - Collision cost uses the ESDF from esdf_ros2.py

References:
  - Wang et al., "Geometrically Constrained Trajectory Optimization
    for Multicopters", IEEE T-RO 2022 (MINCO paper)
  - NEO-Planner IROS'25
"""

import math

import numpy as np
import scipy.optimize as opt


# ── Default config ────────────────────────────────────────────────────────────

class PlannerConfig:
    """
    MinJerkPlanner configuration.

    Default values follow Table III of the NEO-Planner paper:
      M=3, v_max=1.0, T_min=0.5, T_max=5.0
      weights = [1, 1, 1, 10000]
    """
    def __init__(self):
        self.v_max              = 1.0     # m/s maximum speed
        self.T_min              = 0.5     # minimum seconds per piece
        self.T_max              = 5.0     # maximum seconds per piece
        # 2026-07-23: lowered 0.5 -> 0.35 at the user's request (easier
        # navigation in tight gaps). With the real ESDF inflation 0.30m,
        # 0.35 + 0.30 = 0.65m total from the center of mass at rest (down from
        # 0.80m); the expert's K_SPEED_MARGIN raises it to ~0.90m at cruise
        # (down from 1.05m). safe_dis (0.35) still > hard_dis (0.30), so the
        # two-tier soft-cubic zone [0.30, 0.35] stays non-empty (only 0.05m
        # wide — thinner than before, but not collapsed like the 2026-07-21
        # 0.20 experiment which put safe_dis below hard_dis entirely).
        self.safe_dis           = 0.35    # meters, safe distance to obstacle (SOFT margin)
        # Two-tier potential: beyond safe_dis cost=0 (free/smooth motion); the
        # [hard_dis, safe_dis] zone = a cubic soft margin that may be entered when
        # passing through tight gaps (navigation stays agile); below hard_dis =
        # a heavily-weighted quadratic hard barrier -> practically never penetrated.
        self.hard_dis           = 0.30    # m hard barrier (≈ body radius 0.25 + buffer)
        self.w_hard_rel         = 20.0    # relative weight of the hard barrier vs collision
        self.delta_t            = 0.1     # seconds, sampling interval
        self.weights            = [1.0, 1.0, 1.0, 10000.0]
        # [w_energy, w_time, w_feasibility, w_collision]
        self.init_wpts_mode     = "fixed" # "fixed" or "adaptive"
        self.init_seg_len       = 2.0     # meters per segment (adaptive mode)
        self.init_wpts_num      = 2       # number of intermediate waypoints (fixed mode)
        self.init_T             = 2.0     # initial seconds per piece
        self.collision_cost_tol = 10.0    # collision cost tolerance before it is deemed a failure
        self.opt_tol            = 1e-4    # optimizer convergence tolerance


# ── TrajUtils ─────────────────────────────────────────────────────────────────

class TrajUtils:
    """
    Utility functions to evaluate a trajectory from polynomial coefficients.
    Direct port of the original traj_utils.py.
    """

    def get_coeffs(self, int_wpts, ts):
        """
        Compute the MINCO polynomial coefficients from waypoints and time allocation.

        Input:
            int_wpts: (D, M-1) array of intermediate waypoints (column-major)
            ts:       (M,) array of per-piece durations

        Stores:
            self.A      : (2*M*s, 2*M*s) constraint matrix
            self.coeffs : (2*M*s, D)     polynomial coefficients
        """
        int_wpts = int_wpts.T   # (M-1, D)
        T1 = ts
        T2 = ts ** 2
        T3 = ts ** 3
        T4 = ts ** 4
        T5 = ts ** 5

        size = 2 * self.M * self.s
        A = np.zeros((size, size))
        b = np.zeros((size, self.D))
        b[0:self.s, :]  = self.head_state
        b[-self.s:, :]  = self.tail_state

        # Boundary conditions of the first piece
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 2.0

        # Continuity conditions at the junction between pieces
        for i in range(self.M - 1):
            r = 6 * i + 3

            # Position at the end of piece i = waypoint position q_i
            A[r+0, 6*i:6*i+6] = [1, T1[i], T2[i], T3[i], T4[i], T5[i]]

            # Position at the start of piece i+1 = waypoint position q_i
            A[r+1, 6*i:6*i+6]   = [1, T1[i], T2[i], T3[i], T4[i], T5[i]]
            A[r+1, 6*i+6]       = -1.0

            # Velocity continuity
            A[r+2, 6*i+1:6*i+6] = [1, 2*T1[i], 3*T2[i], 4*T3[i], 5*T4[i]]
            A[r+2, 6*i+7]       = -1.0

            # Acceleration continuity
            A[r+3, 6*i+2:6*i+6] = [2, 6*T1[i], 12*T2[i], 20*T3[i]]
            A[r+3, 6*i+8]       = -2.0

            # Jerk continuity
            A[r+4, 6*i+3:6*i+6] = [6, 24*T1[i], 60*T2[i]]
            A[r+4, 6*i+9]       = -6.0

            # Snap continuity
            A[r+5, 6*i+4:6*i+6] = [24, 120*T1[i]]
            A[r+5, 6*i+10]      = -24.0

            # b: intermediate waypoint position
            b[r] = int_wpts[i]

        # Boundary conditions of the last piece
        A[-3, -6:] = [1, T1[-1], T2[-1], T3[-1], T4[-1], T5[-1]]
        A[-2, -5:] = [1, 2*T1[-1], 3*T2[-1], 4*T3[-1], 5*T4[-1]]
        A[-1, -4:] = [2, 6*T1[-1], 12*T2[-1], 20*T3[-1]]

        self.A      = A
        self.coeffs = np.linalg.solve(A, b)

    def _locate_piece(self, t):
        """Find the piece index active at time t."""
        t = min(t, sum(self.ts))
        piece_idx = 0
        while sum(self.ts[:piece_idx + 1]) < t and piece_idx < self.M - 1:
            piece_idx += 1
        return piece_idx

    def get_pos(self, t):
        t = min(t, sum(self.ts))
        i = self._locate_piece(t)
        T = t - sum(self.ts[:i])
        c = self.coeffs[2*self.s*i : 2*self.s*(i+1), :]
        b = np.array([1, T, T**2, T**3, T**4, T**5])
        return c.T @ b

    def get_vel(self, t):
        t = min(t, sum(self.ts))
        i = self._locate_piece(t)
        T = t - sum(self.ts[:i])
        c = self.coeffs[2*self.s*i : 2*self.s*(i+1), :]
        b = np.array([0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4])
        return c.T @ b

    def get_acc(self, t):
        t = min(t, sum(self.ts))
        i = self._locate_piece(t)
        T = t - sum(self.ts[:i])
        c = self.coeffs[2*self.s*i : 2*self.s*(i+1), :]
        b = np.array([0, 0, 2, 6*T, 12*T**2, 20*T**3])
        return c.T @ b

    def get_full_state_cmd(self, hz: int = 300) -> np.ndarray:
        """
        Evaluate the trajectory at every time step and return the state command
        array to be tracked by the controller.

        Returns:
            state_cmd: (N, 3, D) array
                       axis 1 -> [pos, vel, acc]
                       axis 2 -> dimension (x, y)
        """
        self.get_coeffs(self.int_wpts, self.ts)
        total_time = sum(self.ts)
        t_samples  = np.arange(0, total_time, 1.0 / hz)
        N          = len(t_samples)
        state_cmd  = np.zeros((N, 3, self.D))
        for k, t in enumerate(t_samples):
            state_cmd[k, 0] = self.get_pos(t)
            state_cmd[k, 1] = self.get_vel(t)
            state_cmd[k, 2] = self.get_acc(t)
        return state_cmd

    def get_pos_array(self) -> np.ndarray:
        self.get_coeffs(self.int_wpts, self.ts)
        t_samples = np.arange(0, sum(self.ts), 0.1)
        return np.array([self.get_pos(t) for t in t_samples])


# ── MinJerkPlanner ────────────────────────────────────────────────────────────

class MinJerkPlanner(TrajUtils):
    """
    Minimum-jerk trajectory planner using the MINCO parameterization.

    Port of the original NEO-Planner expert_planner.py.

    Usage:
        planner = MinJerkPlanner(config)
        planner.batch_plan(esdf, head_state, tail_state)
        # The result is stored in planner.int_wpts, planner.ts
        state_cmd = planner.get_full_state_cmd(hz=300)
    """

    def __init__(self, config: PlannerConfig = None):
        super().__init__()
        if config is None:
            config = PlannerConfig()

        self.s = 3   # integrator order (jerk = order 3)
        self.D = 2   # dimension (2D planar: x, y)
        self.M = config.init_wpts_num + 1  # number of pieces = waypoints + 1

        self.v_max              = config.v_max
        self.T_min              = config.T_min
        self.T_max              = config.T_max
        self.safe_dis           = config.safe_dis
        self.hard_dis           = config.hard_dis
        self.w_hard_rel         = config.w_hard_rel
        self.collision_cost_tol = config.collision_cost_tol
        self.weights            = np.array(config.weights, dtype=np.float64)
        self.delta_t            = config.delta_t
        self.opt_tol            = config.opt_tol
        self.init_wpts_mode     = config.init_wpts_mode
        self.init_seg_len       = config.init_seg_len
        self.init_wpts_num      = int(config.init_wpts_num)
        self.init_T             = config.init_T
        self.batch_num          = 3   # 3 candidates: straight, right, left

        # Counters for metrics
        self.iter_num         = 0
        self.opt_running_times = 0

        # Pre-compute the beta matrix for sampling
        self._precompute_beta()

        # State after planning (filled by plan/batch_plan)
        self.int_wpts    = None   # (D, M-1) intermediate waypoints
        self.ts          = None   # (M,) time allocation
        self.tau         = None   # (M,) proxy variable for ts
        self.coeffs      = None   # (2*M*s, D) polynomial coefficients
        self.costs       = None   # (4,) [energy, time, feasibility, collision]
        self.final_cost  = None
        self.head_state  = None
        self.tail_state  = None
        self.map         = None

    def _precompute_beta(self):
        """
        Pre-compute the beta(t) matrix for all time steps.
        Speeds up cost and gradient evaluation during optimization.
        """
        t_array     = np.arange(0, self.T_max, self.delta_t)
        beta_full   = np.zeros((len(t_array), 4, 6))
        for idx, t in enumerate(t_array):
            beta_full[idx] = [
                [1,   t,    t**2,    t**3,    t**4,    t**5],
                [0,   1,   2*t,    3*t**2,  4*t**3,  5*t**4],
                [0,   0,   2,      6*t,    12*t**2, 20*t**3],
                [0,   0,   0,      6,      24*t,    60*t**2],
            ]
        self.beta_full = beta_full

    # ── Proxy variable T ↔ tau ─────────────────────────────────────────────

    def map_T2tau(self, ts: np.ndarray) -> np.ndarray:
        """T -> tau via inverse sigmoid. Ensures T > T_min."""
        tau = np.zeros(self.M)
        for i in range(self.M):
            ratio    = (ts[i] - self.T_min) / (self.T_max - self.T_min)
            ratio    = np.clip(ratio, 1e-6, 1 - 1e-6)
            tau[i]   = -math.log(1.0 / ratio - 1.0)
        return tau

    def map_tau2T(self, tau: np.ndarray) -> np.ndarray:
        """tau -> T via sigmoid."""
        ts = np.zeros(self.M)
        for i in range(self.M):
            ts[i] = (self.T_max - self.T_min) / (1.0 + math.exp(-tau[i])) + self.T_min
        return ts

    def get_grad_T2tau(self, grad_T: np.ndarray) -> np.ndarray:
        """Chain rule: d_cost/d_tau = d_cost/d_T * d_T/d_tau."""
        grad_tau = np.zeros(self.M)
        for i in range(self.M):
            grad_tau[i] = (grad_T[i]
                           * (self.T_max - self.T_min)
                           * math.exp(-self.tau[i])
                           / (1 + math.exp(-self.tau[i])) ** 2)
        return grad_tau

    # ── Cost functions ────────────────────────────────────────────────────

    def reset_cost(self):
        self.costs = np.zeros(len(self.weights))

    def reset_grad_CT(self):
        self.grad_C = np.zeros((2 * self.M * self.s, self.D))
        self.grad_T = np.zeros(self.M)

    def add_energy_cost(self):
        """Control effort cost = integral of jerk²."""
        for i in range(self.M):
            c = self.coeffs[2*self.s*i : 2*self.s*(i+1), :]
            T = self.ts[i]
            Q = np.array([
                [0, 0, 0,       0,        0,         0],
                [0, 0, 0,       0,        0,         0],
                [0, 0, 0,       0,        0,         0],
                [0, 0, 0, 36*T,   72*T**2,  120*T**3],
                [0, 0, 0, 72*T**2,192*T**3, 360*T**4],
                [0, 0, 0,120*T**3,360*T**4, 720*T**5],
            ])
            self.costs[0] += np.trace(c.T @ Q @ c)

    def add_energy_grad_CT(self):
        for i in range(self.M):
            c = self.coeffs[2*self.s*i : 2*self.s*(i+1), :]
            T = self.ts[i]
            beta3 = np.array([[0, 0, 0, 6, 24*T, 60*T**2]]).T
            Q = np.array([
                [0, 0, 0,       0,        0,         0],
                [0, 0, 0,       0,        0,         0],
                [0, 0, 0,       0,        0,         0],
                [0, 0, 0, 36*T,   72*T**2,  120*T**3],
                [0, 0, 0, 72*T**2,192*T**3, 360*T**4],
                [0, 0, 0,120*T**3,360*T**4, 720*T**5],
            ])
            self.grad_C[2*self.s*i : 2*self.s*(i+1), :] += (
                self.weights[0] * 2 * Q @ c)
            for d in range(self.D):
                cd = c[:, d]
                self.grad_T[i] += self.weights[0] * float(cd @ beta3) ** 2

    def add_time_cost(self):
        self.costs[1] += float(np.sum(self.ts))

    def add_time_grad_CT(self):
        self.grad_T += self.weights[1] * np.ones(self.M)

    def add_sampled_cost(self):
        """
        Feasibility + collision cost via dense sampling.

        For every point sampled along the trajectory:
          - Check whether the velocity exceeds v_max
          - Check whether the position is too close to an obstacle (via ESDF)
        """
        for i in range(self.M):
            c          = self.coeffs[2*self.s*i : 2*self.s*(i+1), :]
            sample_num = int(self.ts[i] / self.delta_t)

            for j in range(sample_num):
                beta = self.beta_full[j]
                pos  = c.T @ beta[0]   # (D,)
                vel  = c.T @ beta[1]   # (D,)
                omg  = 0.5 if j in (0, sample_num - 1) else 1.0

                # Feasibility: penalty if |vel| > v_max
                violate_vel = float(vel @ vel) - self.v_max ** 2
                if violate_vel > 0.0:
                    self.costs[2] += omg * self.delta_t * violate_vel ** 3

                # Collision: two-tier potential (see PlannerConfig.hard_dis)
                pos_2d   = pos[:2]
                edt_dis  = self.map.get_edt_dis(pos_2d)
                # Soft tier: comfort margin, may be entered (cubic, gentle)
                violate  = self.safe_dis - edt_dis
                if violate > 0.0:
                    self.costs[3] += omg * self.delta_t * violate ** 3
                # Hard tier: drone-body barrier (quadratic + large weight)
                violate_h = self.hard_dis - edt_dis
                if violate_h > 0.0:
                    self.costs[3] += (
                        self.w_hard_rel * omg * self.delta_t * violate_h ** 2)

    def add_sampled_grad_CT(self):
        for i in range(self.M):
            c          = self.coeffs[2*self.s*i : 2*self.s*(i+1), :]
            sample_num = int(self.ts[i] / self.delta_t)

            for j in range(sample_num):
                beta = self.beta_full[j]
                pos  = c.T @ beta[0]
                vel  = c.T @ beta[1]
                omg  = 0.5 if j in (0, sample_num - 1) else 1.0

                # Feasibility gradient
                violate_vel = float(vel @ vel) - self.v_max ** 2
                if violate_vel > 0.0:
                    grad_v2c = 2.0 * np.outer(beta[1], vel)
                    grad_v2t = 2.0 * float(beta[2] @ c @ vel)
                    gKv      = 3.0 * self.delta_t * omg * violate_vel ** 2

                    self.grad_C[2*self.s*i : 2*self.s*(i+1), :] += (
                        self.weights[2] * gKv * grad_v2c)
                    self.grad_T[i] += self.weights[2] * (
                        omg * violate_vel**3 / sample_num
                        + gKv * grad_v2t * j / sample_num)

                # Collision gradient (two-tier potential)
                pos_2d  = pos[:2]
                edt_dis = self.map.get_edt_dis(pos_2d)
                violate   = self.safe_dis - edt_dis
                violate_h = self.hard_dis - edt_dis
                if violate > 0.0 or violate_h > 0.0:
                    edt_grad = self.map.get_edt_grad(pos_2d)   # [gx, gy]

                    # position grad to C grad (shared by both tiers)
                    edt_g_2d = np.zeros(self.D)
                    edt_g_2d[:2] = edt_grad
                    grad_p2c = -np.outer(beta[0], edt_g_2d)
                    grad_p2t = float(-np.array(edt_grad) @ vel[:2])

                    # Soft tier (cubic)
                    if violate > 0.0:
                        gKp = 3.0 * self.delta_t * omg * violate ** 2
                        self.grad_C[2*self.s*i : 2*self.s*(i+1), :] += (
                            self.weights[3] * gKp * grad_p2c)
                        self.grad_T[i] += self.weights[3] * (
                            omg * violate**3 / sample_num
                            + gKp * grad_p2t * j / sample_num)

                    # Hard tier (quadratic + large weight)
                    if violate_h > 0.0:
                        gKh = (self.w_hard_rel * 2.0
                               * self.delta_t * omg * violate_h)
                        self.grad_C[2*self.s*i : 2*self.s*(i+1), :] += (
                            self.weights[3] * gKh * grad_p2c)
                        self.grad_T[i] += self.weights[3] * (
                            self.w_hard_rel * omg * violate_h**2 / sample_num
                            + gKh * grad_p2t * j / sample_num)

    # ── Propagate gradient to Q and tau ───────────────────────────────────

    def propagate_grad_q_tau(self):
        """
        Chain rule from grad_C and grad_T to grad_Q and grad_tau.
        Per eq (3-89) and (3-93) in Wang's thesis.
        """
        grad_q   = np.zeros((self.D, self.M - 1))
        grad_T   = np.zeros(self.M)
        G        = np.linalg.solve(self.A.T, self.grad_C)

        # grad w.r.t. Q: take from the rows corresponding to the waypoints
        for i in range(self.M - 1):
            grad_q[:, i] = G[6*i + 3, :]

        # grad w.r.t. T
        T = None
        for i in range(self.M - 1):
            T   = self.ts[i]
            Gi  = G[2*i*self.s + self.s : 2*(i+1)*self.s + self.s, :].T
            dET = np.array([
                [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
                [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
                [0, 0, 2,   6*T,   12*T**2, 20*T**3],
                [0, 0, 0,   6,     24*T,    60*T**2],
                [0, 0, 0,   0,     24,     120*T],
                [0, 0, 0,   0,     0,      120],
            ])
            ci         = self.coeffs[2*i*self.s : 2*(i+1)*self.s, :]
            grad_T[i]  = self.grad_T[i] - np.trace(Gi @ dET @ ci)

        # Last piece
        T   = self.ts[-1]
        Gi  = G[-self.s:, :].T
        dET = np.array([
            [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
            [0, 0, 2,   6*T,   12*T**2, 20*T**3],
            [0, 0, 0,   6,     24*T,    60*T**2],
        ])
        ci          = self.coeffs[-2*self.s:, :]
        grad_T[-1]  = self.grad_T[-1] - np.trace(Gi @ dET @ ci)

        grad_tau = self.get_grad_T2tau(grad_T)
        return grad_q, grad_tau

    # ── Cost and gradient for L-BFGS-B ──────────────────────────────────────

    def get_cost(self, x: np.ndarray) -> float:
        self.int_wpts = x[:self.D*(self.M-1)].reshape(self.D, self.M-1)
        self.tau      = x[self.D*(self.M-1):]
        self.ts       = self.map_tau2T(self.tau)
        self.get_coeffs(self.int_wpts, self.ts)
        self.reset_cost()
        self.add_energy_cost()
        self.add_time_cost()
        self.add_sampled_cost()
        return float(self.costs @ self.weights)

    def get_grad(self, x: np.ndarray) -> np.ndarray:
        self.int_wpts = x[:self.D*(self.M-1)].reshape(self.D, self.M-1)
        self.tau      = x[self.D*(self.M-1):]
        self.ts       = self.map_tau2T(self.tau)
        self.get_coeffs(self.int_wpts, self.ts)
        self.reset_grad_CT()
        self.add_energy_grad_CT()
        self.add_time_grad_CT()
        self.add_sampled_grad_CT()
        grad_q, grad_tau = self.propagate_grad_q_tau()
        return np.concatenate([grad_q.reshape(-1), grad_tau])

    # ── Planning functions ─────────────────────────────────────────────────

    def read_planning_conditions(self, map, head_state, tail_state,
                                  int_wpts, ts):
        """Set all planning conditions before running the optimizer."""
        self.map  = map
        self.D    = head_state.shape[1]
        self.M    = ts.shape[0]

        self.head_state = np.zeros((self.s, self.D))
        self.tail_state = np.zeros((self.s, self.D))
        for i in range(min(self.s, head_state.shape[0])):
            self.head_state[i] = head_state[i]
        for i in range(min(self.s, tail_state.shape[0])):
            self.tail_state[i] = tail_state[i]

        self.int_wpts = int_wpts
        self.ts       = ts

    def plan_once(self):
        """
        Run one unconstrained L-BFGS-B optimization. Raises ValueError if
        the collision cost is too large.

        2026-07-08: reverted the v1.6/v1.7 SLSQP + LinearConstraint seed
        pinning (kept a homotopy-class seed from crossing to the other side
        of an obstacle mid-optimization). Real-flight A/B on box_narrow
        showed v1.7 was NOT a statistically significant improvement over
        plain unconstrained L-BFGS-B (v1.5) on the metric it targeted
        (bimodal rate 4.2% vs 6.0%, z=1.00 -- inconclusive), while making
        the SEPARATE sign-skew issue significantly WORSE (3:20, p=0.0005
        vs v1.5's 14:7, p=0.19) and adding a real fragility of its own
        (SLSQP has no implicit trust region like L-BFGS-B's line search, so
        it needed a manual tau safety bound to stop map_tau2T() overflowing
        -- a patch for a problem unconstrained L-BFGS-B never had). Net: the
        extra complexity did not pay for itself. See homotopy_frontend.py
        CHANGELOG (removed constraint_for_seed()) for the full history.
        """
        self.tau = self.map_T2tau(self.ts)
        x0 = np.concatenate([
            self.int_wpts.reshape(-1),
            self.tau,
        ])

        res = opt.minimize(
            self.get_cost, x0,
            method="L-BFGS-B",
            jac=self.get_grad,
            tol=self.opt_tol,
            options={
                "maxiter": 15000,
                "maxfun":  15000,
                "maxcor":  10,
                "maxls":   20,
                "disp":    False,
            },
        )

        self.int_wpts      = res.x[:self.D*(self.M-1)].reshape(self.D, self.M-1)
        self.tau           = res.x[self.D*(self.M-1):]
        self.ts            = self.map_tau2T(self.tau)
        self.iter_num      += res.nit
        self.opt_running_times += 1

        self.weighted_cost = self.costs * self.weights
        self.final_cost    = float(self.weighted_cost.sum())

        if self.weighted_cost[3] > self.collision_cost_tol:
            raise ValueError(
                f"Collision cost too large: {self.weighted_cost[3]:.2f} "
                f"> {self.collision_cost_tol}")

    def generate_init_variables(self, head_state, tail_state, seed=0):
        """
        Generate initial waypoints and time allocation.

        If seed != 0, add a random perturbation to the waypoints to avoid the
        same local optima.
        """
        start_pos = head_state[0]
        end_pos   = tail_state[0]
        dist      = np.linalg.norm(end_pos - start_pos)

        if self.init_wpts_mode == "adaptive":
            n = max(math.ceil(dist / self.init_seg_len - 1), 1)
        else:
            n = self.init_wpts_num

        step       = (end_pos - start_pos) / (n + 1)
        int_wpts   = np.linspace(start_pos + step, end_pos, n, endpoint=False)
        if seed != 0:
            int_wpts += np.random.normal(0, 0.5, int_wpts.shape)

        ts = self.init_T * np.ones(n + 1)
        ts[0]  *= 1.5
        ts[-1] *= 1.5

        return int_wpts.T, ts   # (D, M-1), (M,)

    def batch_generate_init_variables(self, head_state, tail_state):
        """
        Generate 3 candidate initial waypoints for batch planning:
          [0] Straight        : waypoints on the direct start->end line
          [1] Curve right     : lateral offset to the right
          [2] Curve left      : lateral offset to the left

        Captures the 3 main homotopy classes per the paper's Section II-D.
        The offset uses a sinusoidal arch: 0 at the endpoints, maximum in the middle.
        """
        start_pos = head_state[0]
        end_pos   = tail_state[0]
        n         = self.init_wpts_num

        # Longitudinal (forward) direction
        fwd = end_pos - start_pos
        fwd_norm = np.linalg.norm(fwd)
        if fwd_norm > 1e-6:
            fwd = fwd / fwd_norm
        else:
            fwd = np.array([1.0, 0.0])

        # 2 lateral directions.
        #   lat_dirs[0] = RIGHT (CW  rotation -90°): (dx,dy) -> ( dy, -dx)
        #   lat_dirs[1] = LEFT  (CCW rotation +90°): (dx,dy) -> (-dy,  dx)
        lat_dirs = np.array([
            [ fwd[1], -fwd[0]],   # right
            [-fwd[1],  fwd[0]],   # left
        ])

        step     = (end_pos - start_pos) / (n + 1)
        baseline = np.linspace(start_pos + step, end_pos, n, endpoint=False)
        # baseline: (n, D)

        batch        = np.zeros((self.batch_num, self.D, n))
        lateral_move = 0.6   # meters, lateral offset amplitude

        # FIX: sinusoidal arch on the INTERIOR points.
        # linspace(0, pi, n+2)[1:-1] drops both endpoints (whose sin = 0),
        # so for n=2 it produces [0.866, 0.866] — not [0, 0].
        bump = np.sin(np.linspace(0, np.pi, n + 2)[1:-1])

        # Candidate 0: straight
        batch[0] = baseline.T

        # Candidate 1 (right) and 2 (left): curved
        lat_flag = 0
        for i in range(1, self.batch_num):
            offset   = lateral_move * bump[:, None] * lat_dirs[lat_flag]
            batch[i] = (baseline + offset).T
            lat_flag = 1 - lat_flag

        ts      = self.init_T * np.ones(n + 1)
        ts[0]  *= 1.5
        ts[-1] *= 1.5

        return batch, ts   # (batch_num, D, M-1), (M,)

    def warm_start_plan(self, map, head_state, tail_state, int_wpts, ts):
        """
        Plan with the given initial values.
        Retry with perturbation on failure (up to 5 times).
        """
        self.read_planning_conditions(map, head_state, tail_state,
                                       int_wpts, ts)
        for seed in range(5):
            try:
                self.plan_once()
                return
            except Exception as e:
                if seed < 4:
                    new_wpts, new_ts = self.generate_init_variables(
                        head_state, tail_state, seed + 1)
                    self.int_wpts = new_wpts
                    self.ts       = new_ts
                else:
                    raise RuntimeError(
                        f"Planning failed after 5 attempts: {e}")

    def batch_plan(self, map, head_state, tail_state):
        """
        Run 3 optimizers with different initial trajectories and pick the result
        with the lowest total cost.

        This is the main function called by the expert planner every time it
        records a training sample.

        Stores the best result in self.int_wpts, self.ts, self.final_cost.

        MULTI-HOMOTOPY (see the workspace CLAUDE.md): ALL candidates that
        converge are ALSO stored in self.all_solutions [(wpts, ts, cost), ...]
        so the expert can record every mode (left/right) as a separate
        label. The drone execution still uses the best one.
        """
        batch_init_wpts, ts = self.batch_generate_init_variables(
            head_state, tail_state)

        best_wpts  = None
        best_ts    = None
        best_cost  = np.inf
        self.all_solutions = []

        for i in range(self.batch_num):
            try:
                self.read_planning_conditions(
                    map, head_state, tail_state,
                    batch_init_wpts[i], ts.copy())
                self.plan_once()
                cost = float(self.weighted_cost.sum())
                self.all_solutions.append(
                    (self.int_wpts.copy(), self.ts.copy(), cost))
                if cost < best_cost:
                    best_cost = cost
                    best_wpts = self.int_wpts.copy()
                    best_ts   = self.ts.copy()
            except Exception as ex:
                pass   # this candidate failed, move on to the next

        if best_wpts is not None:
            self.int_wpts   = best_wpts
            self.ts         = best_ts
            self.final_cost = best_cost
        else:
            # All candidates failed — fall back to warm_start_plan
            int_wpts_fb, ts_fb = self.generate_init_variables(
                head_state, tail_state)
            self.warm_start_plan(map, head_state, tail_state,
                                  int_wpts_fb, ts_fb)
            self.all_solutions = [(self.int_wpts.copy(), self.ts.copy(),
                                   float(self.weighted_cost.sum()))]
