#!/usr/bin/env python3
"""A fake PX4 + MAVROS + px4_sensor_reader for OFFLINE mission tests.

Publishes what the flight nodes read (/px4/state, /px4/sensors JSON,
/mavros/altitude, /mavros/rc/in), serves /mavros/cmd/arming,
/mavros/set_mode, /mavros/param/get_parameters, /mavros/param/pull, and
integrates a simple vehicle model driven by /mavros/setpoint_raw/local:

    EKF z   = true z + gps_bias(t)      (drift, jumps — the thing under test)
    baro    = true z + N(0, 0.085) + ground effect
    PX4 z loop: vz = clip(1.0 * (z_sp - z_ekf), +-0.5), 0.3 s lag
    xy: first-order toward the setpoint (tau 1.0 s)

No real hardware, no MAVROS, no propellers.
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from mavros_msgs.msg import Altitude, PositionTarget, RCIn
from mavros_msgs.srv import CommandBool, ParamPull, SetMode
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters

_BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=10,
                 durability=DurabilityPolicy.VOLATILE)


class FakePX4(Node):
    def __init__(self, scenario):
        super().__init__("fake_px4")
        self.sc = scenario
        self.rng = np.random.default_rng(scenario.get("seed", 0))
        self.t0 = time.time()
        # truth
        self.x = self.y = 0.0
        self.z_true = 0.0
        self.vz = 0.0
        self.yaw = 0.3
        self.bias0 = scenario.get("ekf_bias0", -6.0)
        self.bias = self.bias0
        self.h0 = 130.0
        # fcu
        self.armed = False
        self.mode = "POSCTL"
        self.connected = True
        self.sp = None
        self.t_sp = 0.0
        self.t_land_start = None
        self.t_ground_since = None
        self.baro_on = True
        self.kill = 1065
        self.log = []          # (t, event)
        self.max_true_alt = 0.0
        self.samples = []      # (t, mode, armed, z_true, z_ekf, sp_z)
        # pubs
        self.p_state = self.create_publisher(String, "/px4/state", 10)
        self.p_sens  = self.create_publisher(String, "/px4/sensors", 10)
        self.p_alt   = self.create_publisher(Altitude, "/mavros/altitude", _BE)
        self.p_rc    = self.create_publisher(RCIn, "/mavros/rc/in", _BE)
        self.create_subscription(PositionTarget, "/mavros/setpoint_raw/local", self._cb_sp, 10)
        self.create_service(CommandBool, "/mavros/cmd/arming", self._srv_arm)
        self.create_service(SetMode, "/mavros/set_mode", self._srv_mode)
        self.create_service(GetParameters, "/mavros/param/get_parameters", self._srv_get)
        self.create_service(ParamPull, "/mavros/param/pull", self._srv_pull)
        self.create_timer(0.02, self._physics)
        self.create_timer(0.1, self._publish)

    # ── services ─────────────────────────────────────────────────────────
    def _srv_arm(self, req, res):
        if req.value:
            if self.kill > 1500:
                res.success = False
            else:
                self.armed = True; res.success = True
                self.log.append((self.T(), "ARM"))
        else:
            if self.z_true < 0.05 or not self.sc.get("reject_air_disarm", True):
                self.armed = False; res.success = True
                self.log.append((self.T(), "DISARM"))
            else:
                res.success = False
        return res

    def _srv_mode(self, req, res):
        self.mode = req.custom_mode
        self.log.append((self.T(), f"MODE {req.custom_mode}"))
        if req.custom_mode == "AUTO.LAND":
            self.t_land_start = self.T()
        res.mode_sent = True
        return res

    def _srv_get(self, req, res):
        vals = {"COM_RC_OVERRIDE": 2, "COM_OBL_RC_ACT": 4}
        for n in req.names:
            pv = ParameterValue()
            if n in vals:
                pv.type = ParameterType.PARAMETER_INTEGER; pv.integer_value = vals[n]
            res.values.append(pv)
        return res

    def _srv_pull(self, req, res):
        res.success = True; res.param_received = 2
        return res

    def _cb_sp(self, msg):
        self.sp = msg; self.t_sp = self.T()

    def T(self):
        return time.time() - self.t0

    # ── physics 50 Hz ─────────────────────────────────────────────────────
    def _physics(self):
        dt = 0.02
        t = self.T()
        sc = self.sc
        # scripted EKF bias
        self.bias = self.bias0 + sc.get("ekf_drift_rate", 0.0) * t \
            + sc.get("ekf_wander_amp", 0.0) * math.sin(2 * math.pi * t / sc.get("ekf_wander_period", 30.0))
        for tj, dj in sc.get("ekf_jumps", []):
            if t >= tj:
                self.bias += dj
        if sc.get("baro_off_at") is not None and t >= sc["baro_off_at"]:
            self.baro_on = False
        if sc.get("kill_at") is not None and t >= sc["kill_at"]:
            self.kill = 1933
        z_ekf = self.z_true + self.bias
        in_offb = self.armed and self.mode == "OFFBOARD" and self.sp is not None and (t - self.t_sp) < 0.5
        if in_offb:
            zsp = float(self.sp.position.z)
            vz_cmd = max(-0.5, min(0.5, 1.0 * (zsp - z_ekf)))
            if self.z_true <= 0.0 and vz_cmd < 0.05:
                vz_cmd = 0.0
            self.x += (float(self.sp.position.x) - self.x) * dt / 1.0
            self.y += (float(self.sp.position.y) - self.y) * dt / 1.0
            self.yaw = float(self.sp.yaw)
        elif self.armed and self.mode == "AUTO.LAND":
            vz_cmd = -0.5 if self.z_true > 0.0 else 0.0
        else:
            vz_cmd = 0.0
        self.vz += (vz_cmd - self.vz) * dt / 0.3
        self.z_true = max(0.0, self.z_true + self.vz * dt)
        if self.z_true <= 0.0:
            self.vz = min(0.0, self.vz)
        self.max_true_alt = max(self.max_true_alt, self.z_true)
        # land detector
        if self.armed and self.mode == "AUTO.LAND" and self.z_true <= 0.01:
            if self.t_ground_since is None:
                self.t_ground_since = t
            elif t - self.t_ground_since > 1.0:
                self.armed = False
                self.log.append((t, "AUTO-DISARM (landed)"))
        else:
            self.t_ground_since = None
        # kill: motors off, disarm after 5 s
        if self.kill > 1500 and self.armed:
            self.vz = 0.0; self.z_true = max(0.0, self.z_true - 0.6 * dt)   # falls
            if not hasattr(self, "_kill_t"):
                self._kill_t = t
            elif t - self._kill_t > 5.0:
                self.armed = False; self.log.append((t, "KILL-DISARM"))
        if int(t * 50) % 10 == 0:
            self.samples.append((t, self.mode, self.armed, self.z_true, z_ekf,
                                 float(self.sp.position.z) if self.sp else float("nan")))

    # ── publish 10 Hz ─────────────────────────────────────────────────────
    def _publish(self):
        t = self.T()
        z_ekf = self.z_true + self.bias
        st = {"connected": True, "armed": self.armed, "mode": self.mode, "guided": True, "manual": False}
        self.p_state.publish(String(data=json.dumps(st)))
        sens = {"connected": True, "mode": self.mode, "armed": self.armed,
                "local_x": self.x + self.rng.normal(0, 0.02), "local_y": self.y + self.rng.normal(0, 0.02),
                "local_z": z_ekf + self.rng.normal(0, self.sc.get("ekf_noise", 0.03)),
                "yaw": self.yaw, "vel_x": 0.0, "vel_y": 0.0, "vel_z": self.vz,
                "gps_quality": {"fix_type": 3, "satellites": 20, "hdop": 0.6, "vdop": 0.9, "h_acc_m": 0.5},
                "battery": {"voltage_V": 23.0, "percentage_display": 80.0}, "battery_pct": 80.0}
        self.p_sens.publish(String(data=json.dumps(sens)))
        if self.baro_on:
            m = Altitude()
            ge = -0.3 * max(0.0, 1.0 - self.z_true / 0.8) if self.armed else 0.0
            m.monotonic = self.h0 + self.z_true + ge + self.rng.normal(0, 0.085) \
                + self.sc.get("baro_drift_rate", 0.0) * t
            m.amsl = m.monotonic; m.local = z_ekf; m.relative = self.z_true
            self.p_alt.publish(m)
        rc = RCIn(); rc.channels = [1500] * 4 + [1499] + [1000] * 5 + [self.kill] + [1000] * 5
        self.p_rc.publish(rc)
