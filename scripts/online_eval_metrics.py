#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import math
import time
from collections import deque, defaultdict
from dataclasses import dataclass
from typing import Optional, Dict, Tuple, List

import numpy as np
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Vector3, Twist

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray  # seus msgs


# ------------------------- helpers -------------------------

def wrap_pi(a: float) -> float:
    """wrap angle to (-pi, pi]"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def quat_to_yaw(q) -> float:
    """Yaw from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def median_of_deque(dq: deque) -> float:
    if not dq:
        return 0.0
    arr = sorted(dq)
    mid = len(arr) // 2
    if len(arr) % 2 == 1:
        return float(arr[mid])
    return 0.5 * float(arr[mid - 1] + arr[mid])

def ensure_dir(p: str) -> None:
    os.makedirs(p, exist_ok=True)

def now_sec() -> float:
    """Uses sim time if /use_sim_time is true."""
    return rospy.Time.now().to_sec()

def safe_norm2(v: np.ndarray, eps: float = 1e-12) -> float:
    return float(np.dot(v, v)) + eps


@dataclass
class CPAStats:
    dcpa_min: float = float("inf")
    tcpa_at_min: float = float("nan")  # seconds
    seen: bool = False


# ------------------------- main evaluator node -------------------------

class OnlineExperimentEvaluator:
    """
    Computes metrics online and publishes them to /metrics/*.
    Also saves plots + JSON summary at end-of-run.
    """

    def __init__(self):
        rospy.init_node("online_experiment_evaluator")

        # ---- Params ----
        self.dt = float(rospy.get_param("~dt", 0.1))
        self.output_dir = str(rospy.get_param("~output_dir", "/tmp/mestrado_metrics"))
        self.run_name = str(rospy.get_param("~run_name", time.strftime("%Y%m%d_%H%M%S")))
        self.timeout_s = float(rospy.get_param("~timeout_s", 600.0))

        # safety params (for violations count, same structure as Lyu)
        self.d_safe = float(rospy.get_param("~d_safe", 10.0))  # dsafe (center-to-center add)
        self.R_os_fallback = float(rospy.get_param("~R_os_fallback", 1.4))

        # omega max for CEω (default: 360° in 6s -> 2*pi/6 rad/s)
        self.omega_max = float(rospy.get_param("~omega_max", 2.0 * math.pi / 6.0))

        # smoothing + window params
        self.median_window = int(rospy.get_param("~median_window", 11))  # yaw median window
        self.w_std_window = int(rospy.get_param("~w_std_window", 50))    # std over last N samples

        # plot/save settings
        self.save_plots = bool(rospy.get_param("~save_plots", True))
        self.plot_backend_agg = bool(rospy.get_param("~plot_backend_agg", True))  # headless friendly
        self.plot_heading_stride = int(rospy.get_param("~plot_heading_stride", 15))  # arrows spacing
        self.plot_obstacle_trails = bool(rospy.get_param("~plot_obstacle_trails", True))

        # ---- ROS pubs (metrics topics for rqt_plot) ----
        self.pub_dmin_clear = rospy.Publisher("/metrics/dmin_clearance", Float64, queue_size=10)
        self.pub_dmin_center = rospy.Publisher("/metrics/dmin_center", Float64, queue_size=10)
        self.pub_safety_viol = rospy.Publisher("/metrics/safety_violations_count", Float64, queue_size=10)

        self.pub_dcpa_min = rospy.Publisher("/metrics/dcpa_min", Float64, queue_size=10)
        self.pub_tcpa_min = rospy.Publisher("/metrics/tcpa_at_dcpa_min", Float64, queue_size=10)

        self.pub_t_elapsed = rospy.Publisher("/metrics/time_elapsed", Float64, queue_size=10)
        self.pub_L_path = rospy.Publisher("/metrics/path_length", Float64, queue_size=10)
        self.pub_TV_yaw = rospy.Publisher("/metrics/tv_yaw", Float64, queue_size=10)
        self.pub_w_std = rospy.Publisher("/metrics/yaw_rate_std_window", Float64, queue_size=10)
        self.pub_CEw = rospy.Publisher("/metrics/ce_omega_running", Float64, queue_size=10)
        self.pub_CTE = rospy.Publisher("/metrics/cte_running_mean", Float64, queue_size=10)
        self.pub_bearing = rospy.Publisher("/metrics/rel_bearing_deg_closest", Float64, queue_size=10)

        # computation proxies
        self.pub_apfm_period_ms = rospy.Publisher("/metrics/apfm_period_ms", Float64, queue_size=10)
        self.pub_metrics_compute_ms = rospy.Publisher("/metrics/metrics_compute_ms", Float64, queue_size=10)

        # ---- ROS subs ----
        self.sub_robot = rospy.Subscriber("/scenario/output_robot", RobotState, self.cb_robot, queue_size=10)
        self.sub_obs = rospy.Subscriber("/scenario/output_obstacles", ObstacleArray, self.cb_obstacles, queue_size=10)
        self.sub_goal = rospy.Subscriber("/scenario/goal", Vector3, self.cb_goal, queue_size=10)
        self.sub_reached = rospy.Subscriber("/scenario/reached_goal", Bool, self.cb_reached, queue_size=10)

        self.sub_cmd = rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd, queue_size=10)

        # controller debug signals
        self.sub_v_ref = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/v_ref", Float64, self.cb_v_ref, queue_size=10)
        self.sub_v_meas = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/v_meas", Float64, self.cb_v_meas, queue_size=10)
        self.sub_w_ref = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/w_ref", Float64, self.cb_w_ref, queue_size=10)
        self.sub_w_meas = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/w_meas", Float64, self.cb_w_meas, queue_size=10)

        # proxy of planner period (optional; Vector3 has no header)
        self.sub_apfm_total = rospy.Subscriber("/apfm/total_force", Vector3, self.cb_apfm_total, queue_size=10)

        # ---- State ----
        self.robot: Optional[RobotState] = None
        self.obstacles = []  # list of obstacle states
        self.goal_xy = np.array([0.0, 0.0], dtype=float)
        self.reached_goal = False

        # time bookkeeping
        self.t0: Optional[float] = None
        self.t_last: Optional[float] = None

        # latest controls
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.v_ref = float("nan")
        self.v_meas = float("nan")
        self.w_ref = float("nan")
        self.w_meas = float("nan")

        # path + yaw metrics
        self.last_pos = None
        self.L_path = 0.0

        self.last_yaw = None
        self.TV_yaw = 0.0  # total variation sum |Δψ|
        self.yaw_hist = deque(maxlen=self.median_window)
        self.w_hist = deque(maxlen=self.w_std_window)

        # CEω integral
        self.int_abs_w = 0.0

        # CTE (cross-track error) to line start->goal
        self.start_xy: Optional[np.ndarray] = None
        self.cte_sum = 0.0
        self.cte_n = 0

        # safety
        self.dmin_center = float("inf")
        self.dmin_clear = float("inf")
        self.safety_violations = 0  # count of timesteps with any violation

        # CPA stats per obstacle (by index)
        self.cpa_by_id: Dict[int, CPAStats] = defaultdict(CPAStats)

        # series storage for offline plots
        self.series = {
            "t": [],
            "x": [], "y": [],
            "yaw": [], "yaw_med": [],
            "v_ref": [], "v_meas": [],
            "w_ref": [], "w_meas": [],
            "cmd_v": [], "cmd_w": [],
            "dmin_center": [], "dmin_clear": [],
            "rel_bearing_deg": [],
            "cte": [],
            "cew": [],
            "apfm_period_ms": [],
            "metrics_compute_ms": [],
        }

        # obstacle trails (for 2D plot)
        self.obs_trails: Dict[int, List[Tuple[float, float]]] = defaultdict(list)

        # apfm period proxy
        self._last_apfm_total_t: Optional[float] = None
        self._last_apfm_period_ms: float = float("nan")

        rospy.Timer(rospy.Duration(self.dt), self.on_timer)

        ensure_dir(os.path.join(self.output_dir, self.run_name))
        rospy.loginfo(f"[evaluator] output_dir={self.output_dir} run_name={self.run_name}")

    # ----------------- callbacks -----------------
    def cb_robot(self, msg: RobotState):
        self.robot = msg

    def cb_obstacles(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def cb_goal(self, msg: Vector3):
        self.goal_xy = np.array([float(msg.x), float(msg.y)], dtype=float)

    def cb_reached(self, msg: Bool):
        self.reached_goal = bool(msg.data)

    def cb_cmd(self, msg: Twist):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)

    def cb_v_ref(self, msg: Float64):
        self.v_ref = float(msg.data)

    def cb_v_meas(self, msg: Float64):
        self.v_meas = float(msg.data)

    def cb_w_ref(self, msg: Float64):
        self.w_ref = float(msg.data)

    def cb_w_meas(self, msg: Float64):
        self.w_meas = float(msg.data)

    def cb_apfm_total(self, _msg: Vector3):
        t = now_sec()
        if self._last_apfm_total_t is None:
            self._last_apfm_total_t = t
            return
        dt = t - self._last_apfm_total_t
        self._last_apfm_total_t = t
        if dt > 1e-6:
            self._last_apfm_period_ms = 1000.0 * dt

    # ----------------- core computations -----------------

    def compute_dmin_and_violations(
        self,
        p_os: np.ndarray,
        R_os: float,
        v_os: np.ndarray
    ) -> Tuple[float, float, bool, float]:
        """
        Returns:
          dmin_center, dmin_clearance, violated_any, rel_bearing_deg_closest
        where:
          dmin_center = min ||p_ts - p_os||
          dmin_clearance = min (||p_ts - p_os|| - (R_os + R_ts))  (edge-to-edge)
          violated_any = True if exists obstacle with ||p_ts - p_os|| < d_m
          rel_bearing_deg_closest = bearing of closest obstacle in OS frame (deg)
        """
        if not self.obstacles:
            return float("inf"), float("inf"), False, float("nan")

        # yaw for bearing
        yaw = quat_to_yaw(self.robot.orientation) if self.robot is not None else 0.0

        dmin_center = float("inf")
        dmin_clear = float("inf")
        violated_any = False
        bearing_deg_closest = float("nan")

        best_r = None
        best_d = float("inf")

        for i, ob in enumerate(self.obstacles):
            p_ts = np.array([ob.position.x, ob.position.y], dtype=float)
            R_ts = float(getattr(ob, "radius", 0.0))

            p_ot = p_ts - p_os
            rho_ot = float(np.linalg.norm(p_ot))

            d_m = R_os + self.d_safe + R_ts  # center-to-center safe distance

            if rho_ot < d_m:
                violated_any = True

            dmin_center = min(dmin_center, rho_ot)
            dmin_clear = min(dmin_clear, rho_ot - (R_os + R_ts))

            if rho_ot < best_d:
                best_d = rho_ot
                best_r = p_ot

            # obstacle trails for plotting
            if self.plot_obstacle_trails:
                self.obs_trails[i].append((float(p_ts[0]), float(p_ts[1])))

        if best_r is not None and np.linalg.norm(best_r) > 1e-9:
            ang_los = math.atan2(best_r[1], best_r[0])
            rel_bearing = wrap_pi(ang_los - yaw)
            bearing_deg_closest = math.degrees(rel_bearing)

        return dmin_center, dmin_clear, violated_any, bearing_deg_closest

    def update_cpa(self, p_os: np.ndarray, v_os: np.ndarray):
        """
        Updates per-obstacle DCPA/TCPA minima (classic CPA):
          t_cpa = - (r·v_rel)/||v_rel||^2  (clamped to >=0)
          d_cpa = || r + v_rel * t_cpa ||
        using:
          r = p_ts - p_os
          v_rel = v_ts - v_os
        """
        if not self.obstacles:
            return

        for i, ob in enumerate(self.obstacles):
            p_ts = np.array([ob.position.x, ob.position.y], dtype=float)
            v_ts = np.array([ob.velocity.x, ob.velocity.y], dtype=float)

            r = p_ts - p_os
            v_rel = v_ts - v_os

            v2 = safe_norm2(v_rel)
            # if relative motion is ~zero, CPA not meaningful; skip
            if v2 < 1e-6:
                continue

            t_cpa = - float(np.dot(r, v_rel)) / v2
            if t_cpa < 0.0:
                t_cpa = 0.0

            d_cpa = float(np.linalg.norm(r + v_rel * t_cpa))

            st = self.cpa_by_id[i]
            st.seen = True
            if d_cpa < st.dcpa_min:
                st.dcpa_min = d_cpa
                st.tcpa_at_min = t_cpa

    def compute_cte_to_line(self, p: np.ndarray) -> float:
        """
        Cross-track error (MCTE-style) to nominal straight line from start_xy to goal_xy.
        If start not set, returns 0.
        """
        if self.start_xy is None:
            return 0.0
        a = self.start_xy
        b = self.goal_xy
        ab = b - a
        abn = np.linalg.norm(ab)
        if abn < 1e-9:
            return 0.0
        # distance point-line in 2D: |(p-a) x (b-a)| / ||b-a||
        ap = p - a
        cross = abs(ap[0] * ab[1] - ap[1] * ab[0])
        return float(cross / abn)

    # ----------------- timer loop -----------------
    def on_timer(self, _evt):
        t_compute0 = time.perf_counter()

        if self.robot is None:
            return

        t = now_sec()
        if self.t0 is None:
            self.t0 = t
            self.t_last = t

        t_elapsed = t - self.t0

        # timeout end
        if t_elapsed >= self.timeout_s:
            rospy.logwarn("[evaluator] Timeout reached. Finalizing run.")
            self.finalize(success=False)
            return

        # read robot state
        p_os = np.array([self.robot.position.x, self.robot.position.y], dtype=float)
        v_os = np.array([self.robot.velocity.x, self.robot.velocity.y], dtype=float)
        yaw = quat_to_yaw(self.robot.orientation)

        R_os = float(getattr(self.robot, "radius", 0.0))
        if R_os <= 1e-6:
            R_os = self.R_os_fallback

        # set start point once
        if self.start_xy is None:
            self.start_xy = p_os.copy()

        # path length integration
        if self.last_pos is not None:
            dp = float(np.linalg.norm(p_os - self.last_pos))
            self.L_path += dp
        self.last_pos = p_os

        # yaw total variation TV(ψ) = Σ |wrap(ψk+1 - ψk)|
        if self.last_yaw is not None:
            dy = wrap_pi(yaw - self.last_yaw)
            self.TV_yaw += abs(dy)
        self.last_yaw = yaw

        # median smoothing of yaw (like Lyu median filter idea)
        self.yaw_hist.append(yaw)
        yaw_med = median_of_deque(self.yaw_hist)

        # yaw rate statistics (std of w)
        if not math.isnan(self.w_meas):
            self.w_hist.append(self.w_meas)
        w_std = float(np.std(self.w_hist)) if len(self.w_hist) >= 2 else 0.0

        # CEω running: (1/(ωmax*T)) * ∫ |ω(t)| dt
        if self.t_last is not None:
            dt = max(t - self.t_last, 0.0)
        else:
            dt = 0.0
        self.t_last = t

        w_for_ce = self.w_meas if not math.isnan(self.w_meas) else self.cmd_w
        self.int_abs_w += abs(float(w_for_ce)) * dt
        CEw_running = 0.0
        if t_elapsed > 1e-6 and self.omega_max > 1e-9:
            CEw_running = (self.int_abs_w / (self.omega_max * t_elapsed))

        # dmin and safety violations count
        dmin_center, dmin_clear, violated_any, bearing_deg = self.compute_dmin_and_violations(p_os, R_os, v_os)
        self.dmin_center = min(self.dmin_center, dmin_center)
        self.dmin_clear = min(self.dmin_clear, dmin_clear)
        if violated_any:
            self.safety_violations += 1

        # DCPA/TCPA minima per obstacle (and global min across obstacles)
        self.update_cpa(p_os, v_os)
        dcpa_global = float("inf")
        tcpa_global = float("nan")
        for _, st in self.cpa_by_id.items():
            if st.seen and st.dcpa_min < dcpa_global:
                dcpa_global = st.dcpa_min
                tcpa_global = st.tcpa_at_min
        if dcpa_global == float("inf"):
            dcpa_global = float("nan")

        # CTE (running mean)
        cte = self.compute_cte_to_line(p_os)
        self.cte_sum += cte
        self.cte_n += 1
        cte_mean = self.cte_sum / max(self.cte_n, 1)

        # computation proxies
        apfm_period_ms = self._last_apfm_period_ms

        t_compute1 = time.perf_counter()
        metrics_compute_ms = 1000.0 * (t_compute1 - t_compute0)

        # publish metrics topics for rqt_plot
        self.pub_t_elapsed.publish(Float64(t_elapsed))
        self.pub_L_path.publish(Float64(self.L_path))
        self.pub_dmin_center.publish(Float64(dmin_center))
        self.pub_dmin_clear.publish(Float64(dmin_clear))
        self.pub_safety_viol.publish(Float64(float(self.safety_violations)))

        self.pub_dcpa_min.publish(Float64(dcpa_global if not math.isnan(dcpa_global) else -1.0))
        self.pub_tcpa_min.publish(Float64(tcpa_global if not math.isnan(tcpa_global) else -1.0))

        self.pub_TV_yaw.publish(Float64(self.TV_yaw))
        self.pub_w_std.publish(Float64(w_std))
        self.pub_CEw.publish(Float64(CEw_running))
        self.pub_CTE.publish(Float64(cte_mean))
        self.pub_bearing.publish(Float64(bearing_deg if not math.isnan(bearing_deg) else 0.0))

        if not math.isnan(apfm_period_ms):
            self.pub_apfm_period_ms.publish(Float64(apfm_period_ms))
        self.pub_metrics_compute_ms.publish(Float64(metrics_compute_ms))

        # store series for plots
        self.series["t"].append(t_elapsed)
        self.series["x"].append(float(p_os[0]))
        self.series["y"].append(float(p_os[1]))
        self.series["yaw"].append(float(yaw))
        self.series["yaw_med"].append(float(yaw_med))

        self.series["v_ref"].append(float(self.v_ref) if not math.isnan(self.v_ref) else float("nan"))
        self.series["v_meas"].append(float(self.v_meas) if not math.isnan(self.v_meas) else float("nan"))
        self.series["w_ref"].append(float(self.w_ref) if not math.isnan(self.w_ref) else float("nan"))
        self.series["w_meas"].append(float(self.w_meas) if not math.isnan(self.w_meas) else float("nan"))

        self.series["cmd_v"].append(float(self.cmd_v))
        self.series["cmd_w"].append(float(self.cmd_w))

        self.series["dmin_center"].append(float(dmin_center))
        self.series["dmin_clear"].append(float(dmin_clear))
        self.series["rel_bearing_deg"].append(float(bearing_deg) if not math.isnan(bearing_deg) else float("nan"))
        self.series["cte"].append(float(cte))
        self.series["cew"].append(float(CEw_running))
        self.series["apfm_period_ms"].append(float(apfm_period_ms) if not math.isnan(apfm_period_ms) else float("nan"))
        self.series["metrics_compute_ms"].append(float(metrics_compute_ms))

        # end condition
        if self.reached_goal:
            rospy.loginfo("[evaluator] Goal reached. Finalizing run.")
            self.finalize(success=True)

    # ----------------- finalize: save json + plots -----------------

    def finalize(self, success: bool):
        # guard to avoid multiple finalize
        if rospy.is_shutdown():
            return
        # stop timers by shutting down node after saving
        out_dir = os.path.join(self.output_dir, self.run_name)
        ensure_dir(out_dir)

        # summary metrics
        T_goal = self.series["t"][-1] if self.series["t"] else float("nan")

        # global minima DCPA/TCPA per obstacle + global
        dcpa_global = float("inf")
        tcpa_global = float("nan")
        dcpa_per_ob = {}
        for i, st in self.cpa_by_id.items():
            if st.seen:
                dcpa_per_ob[str(i)] = {"dcpa_min": st.dcpa_min, "tcpa_at_min": st.tcpa_at_min}
                if st.dcpa_min < dcpa_global:
                    dcpa_global = st.dcpa_min
                    tcpa_global = st.tcpa_at_min
        if dcpa_global == float("inf"):
            dcpa_global = float("nan")

        # yaw rate std full-run (use w_meas if available else cmd_w)
        w_series = np.array([w for w in self.series["w_meas"] if not math.isnan(w)], dtype=float)
        if w_series.size < 2:
            w_series = np.array(self.series["cmd_w"], dtype=float)
        w_std_full = float(np.std(w_series)) if w_series.size >= 2 else 0.0

        # CEω final (use running integral)
        T = max(T_goal, 1e-9)
        CEw = float(self.int_abs_w / (self.omega_max * T)) if self.omega_max > 1e-9 else float("nan")

        # CTE mean
        cte_mean = float(self.cte_sum / max(self.cte_n, 1))

        # computation proxy stats
        apfm_period = np.array([x for x in self.series["apfm_period_ms"] if not math.isnan(x)], dtype=float)
        apfm_period_mean = float(np.mean(apfm_period)) if apfm_period.size else float("nan")
        apfm_period_p95 = float(np.percentile(apfm_period, 95)) if apfm_period.size else float("nan")

        metrics_compute = np.array(self.series["metrics_compute_ms"], dtype=float)
        metrics_compute_mean = float(np.mean(metrics_compute)) if metrics_compute.size else float("nan")
        metrics_compute_p95 = float(np.percentile(metrics_compute, 95)) if metrics_compute.size else float("nan")

        summary = {
            "run_name": self.run_name,
            "success": bool(success),
            "T_goal_s": float(T_goal),
            "L_path_m": float(self.L_path),
            "dmin_center_min_m": float(self.dmin_center),
            "dmin_clearance_min_m": float(self.dmin_clear),
            "safety_violations_count": int(self.safety_violations),
            "DCPA_min_m": float(dcpa_global) if not math.isnan(dcpa_global) else None,
            "TCPA_at_DCPA_min_s": float(tcpa_global) if not math.isnan(tcpa_global) else None,
            "DCPA_TCPA_per_obstacle": dcpa_per_ob,
            "TV_yaw_rad": float(self.TV_yaw),
            "yaw_rate_std_full_rad_s": float(w_std_full),
            "CE_omega": float(CEw),
            "CTE_mean_m": float(cte_mean),
            "apfm_period_ms_mean": float(apfm_period_mean) if not math.isnan(apfm_period_mean) else None,
            "apfm_period_ms_p95": float(apfm_period_p95) if not math.isnan(apfm_period_p95) else None,
            "metrics_compute_ms_mean": float(metrics_compute_mean),
            "metrics_compute_ms_p95": float(metrics_compute_p95),
        }

        # save json
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

        # save series to json (lightweight); for big runs prefer CSV
        with open(os.path.join(out_dir, "timeseries.json"), "w") as f:
            json.dump(self.series, f)

        rospy.loginfo(f"[evaluator] Saved summary + timeseries to: {out_dir}")

        if self.save_plots:
            self.save_figures(out_dir)

        rospy.signal_shutdown("Run finished")

    def save_figures(self, out_dir: str):
        # Lazy import for matplotlib (headless safe)
        import matplotlib
        if self.plot_backend_agg:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = np.array(self.series["t"], dtype=float)
        x = np.array(self.series["x"], dtype=float)
        y = np.array(self.series["y"], dtype=float)
        yaw = np.array(self.series["yaw"], dtype=float)
        yaw_med = np.array(self.series["yaw_med"], dtype=float)

        # 1) Trajectory 2D + obstacles + safety circles + heading arrows
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(x, y, label="OS path")
        ax.scatter([x[0]], [y[0]], marker="o", label="start")
        ax.scatter([self.goal_xy[0]], [self.goal_xy[1]], marker="*", label="goal")

        # Heading arrows
        stride = max(1, self.plot_heading_stride)
        for k in range(0, len(x), stride):
            ax.arrow(x[k], y[k], 0.8 * math.cos(yaw[k]), 0.8 * math.sin(yaw[k]),
                     head_width=0.2, length_includes_head=True)

        # Obstacles (plot trails and final safety circle approximation)
        if self.plot_obstacle_trails and self.obs_trails:
            for oid, trail in self.obs_trails.items():
                if len(trail) >= 2:
                    tx = [p[0] for p in trail]
                    ty = [p[1] for p in trail]
                    ax.plot(tx, ty, linestyle="--", alpha=0.6, label=f"obs{oid}_trail" if oid < 3 else None)

                # draw a safety circle at final position with radius approx (R_os + d_safe + R_ts)
                # R_ts not stored in trail; so we draw just d_safe + R_os as a baseline safety envelope
                cx, cy = trail[-1]
                R_safe = self.R_os_fallback + self.d_safe
                circ = plt.Circle((cx, cy), R_safe, fill=False, alpha=0.35)
                ax.add_patch(circ)

        ax.set_aspect("equal", adjustable="box")
        ax.set_title("2D Trajectory + Obstacles + Safety Radius + Heading")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "traj_2d.png"), dpi=200)
        plt.close(fig)

        # 2) dmin(t)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["dmin_center"], label="dmin_center(t)")
        ax.plot(t, self.series["dmin_clear"], label="dmin_clearance(t)")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("distance [m]")
        ax.set_title("Minimum distance to obstacles over time")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "dmin_timeseries.png"), dpi=200)
        plt.close(fig)

        # 3) relative bearing (closest obstacle)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["rel_bearing_deg"], label="relative bearing (closest) [deg]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("deg")
        ax.set_title("Relative bearing to closest obstacle")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "bearing_timeseries.png"), dpi=200)
        plt.close(fig)

        # 4) heading yaw raw + median
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, yaw, label="yaw raw [rad]")
        ax.plot(t, yaw_med, label="yaw median [rad]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("rad")
        ax.set_title("Heading (yaw): raw vs median-filtered")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "yaw_timeseries.png"), dpi=200)
        plt.close(fig)

        # 5) v(t) and w(t) (ref vs meas + cmd)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["v_ref"], label="v_ref")
        ax.plot(t, self.series["v_meas"], label="v_meas")
        ax.plot(t, self.series["cmd_v"], label="cmd_v")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("m/s")
        ax.set_title("Linear velocity tracking")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "v_timeseries.png"), dpi=200)
        plt.close(fig)

        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["w_ref"], label="w_ref")
        ax.plot(t, self.series["w_meas"], label="w_meas")
        ax.plot(t, self.series["cmd_w"], label="cmd_w")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("rad/s")
        ax.set_title("Angular velocity tracking")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "w_timeseries.png"), dpi=200)
        plt.close(fig)

        # 6) computation proxy
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["apfm_period_ms"], label="APFM period [ms] (proxy)")
        ax.plot(t, self.series["metrics_compute_ms"], label="metrics compute [ms]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("ms")
        ax.set_title("Computation proxies")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "compute_timeseries.png"), dpi=200)
        plt.close(fig)

        # 7) CEω + CTE
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["cew"], label="CEω running")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("[-]")
        ax.set_title("Control Effort (CEω) running")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "cew_timeseries.png"), dpi=200)
        plt.close(fig)

        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["cte"], label="CTE [m]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("m")
        ax.set_title("Cross-track error (to straight start-goal line)")
        ax.legend(loc="best")
        fig.savefig(os.path.join(out_dir, "cte_timeseries.png"), dpi=200)
        plt.close(fig)

        rospy.loginfo(f"[evaluator] Plots saved to: {out_dir}")


if __name__ == "__main__":
    OnlineExperimentEvaluator()
    rospy.spin()
