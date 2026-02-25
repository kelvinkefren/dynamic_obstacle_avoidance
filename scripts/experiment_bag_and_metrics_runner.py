#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
experiment_bag_and_metrics_runner.py

O que este nó faz (tudo em 1):
1) Inicia gravação de rosbag (subprocess: `rosbag record ...`) salvando em uma pasta do experimento.
2) Calcula métricas online (as mesmas ideias do seu online_eval_metrics.py) e publica em /metrics/*
3) Ao finalizar (goal, colisão opcional ou timeout), salva:
   - summary.json (métricas finais)
   - timeseries.json + timeseries.csv
   - figuras (.png e .pdf) para dissertação
   - run_info.json (cenário, tópicos gravados, caminhos, params)
   - bagfile (.bag) no mesmo diretório

Compatível com seu pipeline (ROS1):
- /scenario/output_robot (RobotState)
- /scenario/output_obstacles (ObstacleArray)
- /scenario/goal (Vector3)
- /scenario/reached_goal (Bool)
- /cmd_vel (Twist)
- /apfm/total_force (Vector3) (proxy de período)
- /obstacle_avoidance/collision (Bool) (opcional)
- /current_scenario (String) (opcional)
- + debug do controlador (v_ref/v_meas/w_ref/w_meas) (opcional)

Parede virtual:
- Se seus obstáculos incluem nomes "wall_L_*" e "wall_R_*" (do gazebo_scenario_v2),
  este nó mostra/salva as paredes no plot 2D (como polilinhas aproximadas).

Como rodar:
  rosrun <seu_pkg> experiment_bag_and_metrics_runner.py _output_dir:=/tmp/runs _run_name:=teste01

Parâmetros principais:
  ~output_dir (str)   : diretório base (default /tmp/mestrado_runs)
  ~run_name (str)     : nome do run (default timestamp)
  ~timeout_s (float)  : timeout (default 600)
  ~stop_on_collision (bool) : encerra ao detectar colisão (default True)
  ~record_bag (bool)  : grava bag (default True)
  ~bag_compress (str) : "lz4" (default) ou "" para sem compressão
  ~bag_topics (list)  : lista de tópicos (se vazio, usa defaults)
  ~ignore_walls_in_cpa (bool): DCPA/TCPA ignoram paredes virtuais (default True)

Saída:
  <output_dir>/<run_name>/
    run.bag
    summary.json
    timeseries.json
    timeseries.csv
    traj_2d.(png|pdf)
    dmin_timeseries.(png|pdf)
    bearing_timeseries.(png|pdf)
    yaw_timeseries.(png|pdf)
    v_timeseries.(png|pdf)
    w_timeseries.(png|pdf)
    compute_timeseries.(png|pdf)
    cew_timeseries.(png|pdf)
    cte_timeseries.(png|pdf)
    run_info.json
"""
import os
import csv
import json
import math
import time
import signal
import subprocess
from collections import deque, defaultdict
from dataclasses import dataclass
from typing import Optional, Dict, Tuple, List

import numpy as np
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Vector3, Twist
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray  # seus msgs


# ------------------------- helpers -------------------------

EPS = 1e-9

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

def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < EPS:
        return np.array([0.0, 0.0], dtype=float)
    return v / n

def sort_points_along_axis(points_xy: List[Tuple[float, float]], axis_u: np.ndarray) -> List[Tuple[float, float]]:
    axis_u = unit(axis_u)
    if float(np.linalg.norm(axis_u)) < EPS:
        axis_u = np.array([1.0, 0.0], dtype=float)
    keys = [float(np.dot(np.array(p, dtype=float), axis_u)) for p in points_xy]
    return [p for _, p in sorted(zip(keys, points_xy), key=lambda t: t[0])]


@dataclass
class CPAStats:
    dcpa_min: float = float("inf")
    tcpa_at_min: float = float("nan")  # seconds
    seen: bool = False


class RosbagRecorder:
    def __init__(self, bag_path: str, topics: List[str], compress: str = "lz4"):
        self.bag_path = bag_path
        self.topics = topics
        self.compress = compress
        self.proc: Optional[subprocess.Popen] = None

    def start(self):
        cmd = ["rosbag", "record", "-O", self.bag_path]
        if self.compress:
            if self.compress.lower() == "lz4":
                cmd.append("--lz4")
            elif self.compress.lower() == "bz2":
                cmd.append("--bz2")
        cmd += self.topics

        # process group para matar tudo com SIGINT
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
            text=True
        )
        rospy.loginfo(f"[bag] Recording -> {self.bag_path}")
        rospy.loginfo(f"[bag] Topics: {len(self.topics)}")

    def stop(self, timeout_s: float = 5.0):
        if self.proc is None:
            return
        if self.proc.poll() is not None:
            return

        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)  # rosbag fecha limpo
        except Exception as e:
            rospy.logwarn(f"[bag] SIGINT failed: {e}")

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.proc.poll() is not None:
                break
            time.sleep(0.1)

        if self.proc.poll() is None:
            rospy.logwarn("[bag] Forçando término (SIGTERM).")
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except Exception as e:
                rospy.logwarn(f"[bag] SIGTERM failed: {e}")

        rospy.loginfo("[bag] Stopped.")


class ExperimentBagAndMetricsRunner:
    """
    Uma versão "pronta pra dissertação": grava bag + salva métricas + salva figuras no final.
    """
    def __init__(self):
        rospy.init_node("experiment_bag_and_metrics_runner")

        # ---- Params ----
        self.dt = float(rospy.get_param("~dt", 0.1))
        self.output_dir = str(rospy.get_param("~output_dir", "/tmp/mestrado_runs"))
        self.run_name = str(rospy.get_param("~run_name", time.strftime("%Y%m%d_%H%M%S")))
        self.timeout_s = float(rospy.get_param("~timeout_s", 600.0))
        self.stop_on_collision = bool(rospy.get_param("~stop_on_collision", True))

        # safety params
        self.d_safe = float(rospy.get_param("~d_safe", 10.0))
        self.R_os_fallback = float(rospy.get_param("~R_os_fallback", 1.4))

        # omega max for CEω (default: 360° in 6s -> 2*pi/6 rad/s)
        self.omega_max = float(rospy.get_param("~omega_max", 2.0 * math.pi / 6.0))

        # smoothing + window params
        self.median_window = int(rospy.get_param("~median_window", 11))
        self.w_std_window = int(rospy.get_param("~w_std_window", 50))

        # plot/save settings
        self.save_plots = bool(rospy.get_param("~save_plots", True))
        self.plot_backend_agg = bool(rospy.get_param("~plot_backend_agg", True))
        self.plot_heading_stride = int(rospy.get_param("~plot_heading_stride", 15))
        self.plot_obstacle_trails = bool(rospy.get_param("~plot_obstacle_trails", True))
        self.ignore_walls_in_cpa = bool(rospy.get_param("~ignore_walls_in_cpa", True))

        # bag params
        self.record_bag = bool(rospy.get_param("~record_bag", True))
        self.bag_compress = str(rospy.get_param("~bag_compress", "lz4"))
        self.bag_topics = rospy.get_param("~bag_topics", [])
        if not isinstance(self.bag_topics, list):
            self.bag_topics = []

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
        self.pub_apfm_period_ms = rospy.Publisher("/metrics/apfm_period_ms", Float64, queue_size=10)
        self.pub_metrics_compute_ms = rospy.Publisher("/metrics/metrics_compute_ms", Float64, queue_size=10)

        # ---- ROS subs ----
        self.sub_robot = rospy.Subscriber("/scenario/output_robot", RobotState, self.cb_robot, queue_size=10)
        self.sub_obs = rospy.Subscriber("/scenario/output_obstacles", ObstacleArray, self.cb_obstacles, queue_size=10)
        self.sub_goal = rospy.Subscriber("/scenario/goal", Vector3, self.cb_goal, queue_size=10)
        self.sub_reached = rospy.Subscriber("/scenario/reached_goal", Bool, self.cb_reached, queue_size=10)
        self.sub_cmd = rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd, queue_size=10)
        self.sub_apfm_total = rospy.Subscriber("/apfm/total_force", Vector3, self.cb_apfm_total, queue_size=10)

        # colisão (opcional)
        self.collision = False
        self.sub_collision = rospy.Subscriber("/obstacle_avoidance/collision", Bool, self.cb_collision, queue_size=10)

        # cenário (opcional)
        self.current_scenario = ""
        try:
            from std_msgs.msg import String
            self.sub_scn = rospy.Subscriber("/current_scenario", String, self.cb_scenario, queue_size=10)
        except Exception:
            self.sub_scn = None

        # controller debug (opcional)
        self.v_ref = float("nan")
        self.v_meas = float("nan")
        self.w_ref = float("nan")
        self.w_meas = float("nan")
        try:
            self.sub_v_ref = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/v_ref", Float64, self.cb_v_ref, queue_size=10)
            self.sub_v_meas = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/v_meas", Float64, self.cb_v_meas, queue_size=10)
            self.sub_w_ref = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/w_ref", Float64, self.cb_w_ref, queue_size=10)
            self.sub_w_meas = rospy.Subscriber("/migbot_velocity_feedback_controller/debug/w_meas", Float64, self.cb_w_meas, queue_size=10)
        except Exception:
            pass

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

        # path + yaw metrics
        self.last_pos = None
        self.L_path = 0.0
        self.last_yaw = None
        self.TV_yaw = 0.0
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
        self.safety_violations = 0

        # CPA stats per obstacle index
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

        # wall point clouds (for 2D plot)
        self.wall_L_points: List[Tuple[float, float]] = []
        self.wall_R_points: List[Tuple[float, float]] = []

        # apfm period proxy
        self._last_apfm_total_t: Optional[float] = None
        self._last_apfm_period_ms: float = float("nan")

        # output dir
        self.run_dir = os.path.join(self.output_dir, self.run_name)
        ensure_dir(self.run_dir)

        # start bag
        self.recorder: Optional[RosbagRecorder] = None
        if self.record_bag:
            topics = self._default_bag_topics() if len(self.bag_topics) == 0 else self.bag_topics
            bag_path = os.path.join(self.run_dir, "run.bag")
            self.recorder = RosbagRecorder(bag_path=bag_path, topics=topics, compress=self.bag_compress)
            self.recorder.start()

        rospy.loginfo(f"[runner] output_dir={self.output_dir} run_name={self.run_name}")
        rospy.Timer(rospy.Duration(self.dt), self.on_timer)

        # Salva info inicial do run
        self._save_run_info(initial=True)

    # ----------------- callbacks -----------------
    def cb_robot(self, msg: RobotState):
        self.robot = msg

    def cb_obstacles(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

        # capture wall points (virtual walls)
        for ob in msg.obstacles:
            name = getattr(ob, "name", "")
            if name.startswith("wall_L"):
                self.wall_L_points.append((float(ob.position.x), float(ob.position.y)))
            elif name.startswith("wall_R"):
                self.wall_R_points.append((float(ob.position.x), float(ob.position.y)))

        # keep only a reasonable number to avoid huge memory (downsample)
        max_pts = 5000
        if len(self.wall_L_points) > max_pts:
            self.wall_L_points = self.wall_L_points[-max_pts:]
        if len(self.wall_R_points) > max_pts:
            self.wall_R_points = self.wall_R_points[-max_pts:]

    def cb_goal(self, msg: Vector3):
        self.goal_xy = np.array([float(msg.x), float(msg.y)], dtype=float)

    def cb_reached(self, msg: Bool):
        self.reached_goal = bool(msg.data)

    def cb_collision(self, msg: Bool):
        self.collision = bool(msg.data)

    def cb_cmd(self, msg: Twist):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)

    def cb_scenario(self, msg):
        self.current_scenario = str(msg.data)

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

    # ----------------- bag topics -----------------
    def _default_bag_topics(self) -> List[str]:
        # Para reproduzir e provar seus resultados (e ter "o que o robô viu")
        topics = [
            "/clock",
            "/tf", "/tf_static",
            "/rosout",
            "/gazebo/model_states",
            "/scenario/input_robot", "/scenario/input_obstacles",
            "/scenario/output_robot", "/scenario/output_obstacles",
            "/scenario/goal", "/scenario/reached_goal",
            "/current_scenario",
            "/cmd_vel",
            "/apfm/total_force",
            "/obstacle_avoidance/custom_info",
            "/obstacle_avoidance/collision",
            # métricas ao vivo
            "/metrics/dmin_clearance",
            "/metrics/dmin_center",
            "/metrics/safety_violations_count",
            "/metrics/dcpa_min",
            "/metrics/tcpa_at_dcpa_min",
            "/metrics/time_elapsed",
            "/metrics/path_length",
            "/metrics/tv_yaw",
            "/metrics/yaw_rate_std_window",
            "/metrics/ce_omega_running",
            "/metrics/cte_running_mean",
            "/metrics/rel_bearing_deg_closest",
            "/metrics/apfm_period_ms",
            "/metrics/metrics_compute_ms",
            # debug do controlador (se existir)
            "/migbot_velocity_feedback_controller/debug/v_ref",
            "/migbot_velocity_feedback_controller/debug/v_meas",
            "/migbot_velocity_feedback_controller/debug/w_ref",
            "/migbot_velocity_feedback_controller/debug/w_meas",
        ]
        return topics

    # ----------------- core computations -----------------

    def compute_dmin_and_violations(
        self,
        p_os: np.ndarray,
        R_os: float,
        yaw: float
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

        dmin_center = float("inf")
        dmin_clear = float("inf")
        violated_any = False
        bearing_deg_closest = float("nan")

        best_r = None
        best_d = float("inf")

        for i, ob in enumerate(self.obstacles):
            p_ts = np.array([ob.position.x, ob.position.y], dtype=float)
            R_ts = float(getattr(ob, "radius", 0.0))
            name = str(getattr(ob, "name", ""))

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

            # obstacle trails (ignore walls in trails? keep both, but walls look messy)
            if self.plot_obstacle_trails and (not name.startswith("wall_")):
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

        Por padrão, ignora paredes virtuais (wall_*) porque não são "outros navios".
        """
        if not self.obstacles:
            return

        for i, ob in enumerate(self.obstacles):
            name = str(getattr(ob, "name", ""))
            if self.ignore_walls_in_cpa and name.startswith("wall_"):
                continue

            p_ts = np.array([ob.position.x, ob.position.y], dtype=float)
            v_ts = np.array([ob.velocity.x, ob.velocity.y], dtype=float)

            r = p_ts - p_os
            v_rel = v_ts - v_os

            v2 = safe_norm2(v_rel)
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
        """Cross-track error to nominal straight line from start_xy to goal_xy."""
        if self.start_xy is None:
            return 0.0
        a = self.start_xy
        b = self.goal_xy
        ab = b - a
        abn = np.linalg.norm(ab)
        if abn < 1e-9:
            return 0.0
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
            rospy.logwarn("[runner] Timeout reached. Finalizing run.")
            self.finalize(success=False, reason="timeout")
            return

        # collision end (optional)
        if self.stop_on_collision and self.collision:
            rospy.logwarn("[runner] Collision detected. Finalizing run.")
            self.finalize(success=False, reason="collision")
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

        # yaw total variation TV(ψ)
        if self.last_yaw is not None:
            dy = wrap_pi(yaw - self.last_yaw)
            self.TV_yaw += abs(dy)
        self.last_yaw = yaw

        # median smoothing of yaw
        self.yaw_hist.append(yaw)
        yaw_med = median_of_deque(self.yaw_hist)

        # yaw rate statistics (std of w)
        if not math.isnan(self.w_meas):
            self.w_hist.append(self.w_meas)
        w_std = float(np.std(self.w_hist)) if len(self.w_hist) >= 2 else 0.0

        # CEω running
        dt = max(t - self.t_last, 0.0) if self.t_last is not None else 0.0
        self.t_last = t
        w_for_ce = self.w_meas if not math.isnan(self.w_meas) else self.cmd_w
        self.int_abs_w += abs(float(w_for_ce)) * dt
        CEw_running = 0.0
        if t_elapsed > 1e-6 and self.omega_max > 1e-9:
            CEw_running = (self.int_abs_w / (self.omega_max * t_elapsed))

        # dmin and safety violations count
        dmin_center, dmin_clear, violated_any, bearing_deg = self.compute_dmin_and_violations(p_os, R_os, yaw)
        self.dmin_center = min(self.dmin_center, dmin_center)
        self.dmin_clear = min(self.dmin_clear, dmin_clear)
        if violated_any:
            self.safety_violations += 1

        # DCPA/TCPA minima
        self.update_cpa(p_os, v_os)
        dcpa_global = float("inf")
        tcpa_global = float("nan")
        for _, st in self.cpa_by_id.items():
            if st.seen and st.dcpa_min < dcpa_global:
                dcpa_global = st.dcpa_min
                tcpa_global = st.tcpa_at_min
        if dcpa_global == float("inf"):
            dcpa_global = float("nan")

        # CTE running mean
        cte = self.compute_cte_to_line(p_os)
        self.cte_sum += cte
        self.cte_n += 1
        cte_mean = self.cte_sum / max(self.cte_n, 1)

        # computation proxies
        apfm_period_ms = self._last_apfm_period_ms

        t_compute1 = time.perf_counter()
        metrics_compute_ms = 1000.0 * (t_compute1 - t_compute0)

        # publish metrics topics
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

        # store series
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
            rospy.loginfo("[runner] Goal reached. Finalizing run.")
            self.finalize(success=True, reason="goal")

    # ----------------- save outputs -----------------
    def _save_run_info(self, initial: bool):
        # pega cenário por param também (quando /current_scenario não existe)
        scn_param = ""
        try:
            scn_param = str(rospy.get_param("/gazebo_scenario/scenario", ""))
        except Exception:
            scn_param = ""

        info = {
            "run_name": self.run_name,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "scenario_topic": self.current_scenario,
            "scenario_param": scn_param,
            "dt": self.dt,
            "timeout_s": self.timeout_s,
            "stop_on_collision": self.stop_on_collision,
            "record_bag": self.record_bag,
            "bag_compress": self.bag_compress,
            "bag_topics": (self._default_bag_topics() if len(self.bag_topics) == 0 else self.bag_topics),
            "params": {
                "d_safe": self.d_safe,
                "omega_max": self.omega_max,
                "median_window": self.median_window,
                "w_std_window": self.w_std_window,
                "ignore_walls_in_cpa": self.ignore_walls_in_cpa,
            },
            "paths": {
                "run_dir": self.run_dir,
                "bag_path": os.path.join(self.run_dir, "run.bag") if self.record_bag else None,
            },
            "initial": bool(initial),
        }
        with open(os.path.join(self.run_dir, "run_info.json"), "w") as f:
            json.dump(info, f, indent=2)

    def finalize(self, success: bool, reason: str):
        # evita double finalize
        if getattr(self, "_finalized", False):
            return
        self._finalized = True

        # para bag
        if self.recorder is not None:
            self.recorder.stop()

        out_dir = self.run_dir
        ensure_dir(out_dir)

        # summary metrics
        T_goal = self.series["t"][-1] if self.series["t"] else float("nan")

        # global DCPA/TCPA per obstacle + global
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

        # yaw rate std full-run
        w_series = np.array([w for w in self.series["w_meas"] if not math.isnan(w)], dtype=float)
        if w_series.size < 2:
            w_series = np.array(self.series["cmd_w"], dtype=float)
        w_std_full = float(np.std(w_series)) if w_series.size >= 2 else 0.0

        # CEω final
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

        # cenário
        scn_param = ""
        try:
            scn_param = str(rospy.get_param("/gazebo_scenario/scenario", ""))
        except Exception:
            scn_param = ""

        summary = {
            "run_name": self.run_name,
            "success": bool(success),
            "reason": str(reason),
            "scenario_topic": self.current_scenario,
            "scenario_param": scn_param,
            "T_goal_s": float(T_goal),
            "L_path_m": float(self.L_path),
            "dmin_center_min_m": float(self.dmin_center),
            "dmin_clearance_min_m": float(self.dmin_clear),
            "safety_violations_count": int(self.safety_violations),
            "DCPA_min_m": (float(dcpa_global) if not math.isnan(dcpa_global) else None),
            "TCPA_at_DCPA_min_s": (float(tcpa_global) if not math.isnan(tcpa_global) else None),
            "DCPA_TCPA_per_obstacle": dcpa_per_ob,
            "TV_yaw_rad": float(self.TV_yaw),
            "yaw_rate_std_full_rad_s": float(w_std_full),
            "CE_omega": (float(CEw) if not math.isnan(CEw) else None),
            "CTE_mean_m": float(cte_mean),
            "apfm_period_ms_mean": (float(apfm_period_mean) if not math.isnan(apfm_period_mean) else None),
            "apfm_period_ms_p95": (float(apfm_period_p95) if not math.isnan(apfm_period_p95) else None),
            "metrics_compute_ms_mean": float(metrics_compute_mean),
            "metrics_compute_ms_p95": float(metrics_compute_p95),
        }

        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

        with open(os.path.join(out_dir, "timeseries.json"), "w") as f:
            json.dump(self.series, f)

        # CSV (mais fácil pra Excel/Overleaf)
        self._save_timeseries_csv(os.path.join(out_dir, "timeseries.csv"))

        # atualiza run_info
        self._save_run_info(initial=False)

        rospy.loginfo(f"[runner] Saved outputs to: {out_dir}")

        if self.save_plots:
            self.save_figures(out_dir)

        rospy.signal_shutdown("Run finished")

    def _save_timeseries_csv(self, csv_path: str):
        keys = list(self.series.keys())
        n = len(self.series[keys[0]]) if keys else 0
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(keys)
            for i in range(n):
                row = []
                for k in keys:
                    arr = self.series[k]
                    row.append(arr[i] if i < len(arr) else "")
                w.writerow(row)

    # ----------------- figures -----------------
    def save_figures(self, out_dir: str):
        import matplotlib
        if self.plot_backend_agg:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = np.array(self.series["t"], dtype=float)
        x = np.array(self.series["x"], dtype=float)
        y = np.array(self.series["y"], dtype=float)
        yaw = np.array(self.series["yaw"], dtype=float)
        yaw_med = np.array(self.series["yaw_med"], dtype=float)

        # axis for wall sorting (along start->goal)
        axis = (self.goal_xy - self.start_xy) if (self.start_xy is not None) else np.array([1.0, 0.0], dtype=float)

        wall_L = self.wall_L_points
        wall_R = self.wall_R_points
        # remove duplicates by rounding
        def unique_round(pts):
            s = set()
            out = []
            for px, py in pts:
                key = (round(px, 2), round(py, 2))
                if key not in s:
                    s.add(key)
                    out.append((px, py))
            return out

        wall_L = unique_round(wall_L)
        wall_R = unique_round(wall_R)
        wall_L = sort_points_along_axis(wall_L, axis) if len(wall_L) >= 2 else wall_L
        wall_R = sort_points_along_axis(wall_R, axis) if len(wall_R) >= 2 else wall_R

        def save(fig, name):
            fig.savefig(os.path.join(out_dir, f"{name}.png"), dpi=220)
            fig.savefig(os.path.join(out_dir, f"{name}.pdf"))
            plt.close(fig)

        # 1) Trajectory 2D + walls + obstacle trails + heading arrows
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(x, y, label="OS path")
        if x.size:
            ax.scatter([x[0]], [y[0]], marker="o", label="start")
        ax.scatter([self.goal_xy[0]], [self.goal_xy[1]], marker="*", label="goal")

        # walls (virtual)
        if len(wall_L) >= 2:
            ax.plot([p[0] for p in wall_L], [p[1] for p in wall_L], linewidth=2.0, alpha=0.7, label="wall_L (virtual)")
        if len(wall_R) >= 2:
            ax.plot([p[0] for p in wall_R], [p[1] for p in wall_R], linewidth=2.0, alpha=0.7, label="wall_R (virtual)")

        # Heading arrows
        stride = max(1, self.plot_heading_stride)
        for k in range(0, len(x), stride):
            ax.arrow(x[k], y[k], 0.9 * math.cos(yaw[k]), 0.9 * math.sin(yaw[k]),
                     head_width=0.25, length_includes_head=True, alpha=0.8)

        # obstacle trails (non-walls)
        if self.plot_obstacle_trails and self.obs_trails:
            for oid, trail in self.obs_trails.items():
                if len(trail) >= 2:
                    tx = [p[0] for p in trail]
                    ty = [p[1] for p in trail]
                    ax.plot(tx, ty, linestyle="--", alpha=0.45, label=f"obs{oid}_trail" if oid < 2 else None)

        ax.set_aspect("equal", adjustable="box")
        ax.set_title("2D Trajectory + Virtual Walls + Obstacle Trails + Heading")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend(loc="best")
        save(fig, "traj_2d")

        # 2) dmin(t)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["dmin_center"], label="dmin_center(t)")
        ax.plot(t, self.series["dmin_clear"], label="dmin_clearance(t)")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("distance [m]")
        ax.set_title("Minimum distance to obstacles over time")
        ax.legend(loc="best")
        save(fig, "dmin_timeseries")

        # 3) relative bearing
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["rel_bearing_deg"], label="relative bearing (closest) [deg]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("deg")
        ax.set_title("Relative bearing to closest obstacle")
        ax.legend(loc="best")
        save(fig, "bearing_timeseries")

        # 4) yaw raw + median
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, yaw, label="yaw raw [rad]")
        ax.plot(t, yaw_med, label="yaw median [rad]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("rad")
        ax.set_title("Heading (yaw): raw vs median-filtered")
        ax.legend(loc="best")
        save(fig, "yaw_timeseries")

        # 5) v(t)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["v_ref"], label="v_ref")
        ax.plot(t, self.series["v_meas"], label="v_meas")
        ax.plot(t, self.series["cmd_v"], label="cmd_v")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("m/s")
        ax.set_title("Linear velocity tracking")
        ax.legend(loc="best")
        save(fig, "v_timeseries")

        # 6) w(t)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["w_ref"], label="w_ref")
        ax.plot(t, self.series["w_meas"], label="w_meas")
        ax.plot(t, self.series["cmd_w"], label="cmd_w")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("rad/s")
        ax.set_title("Angular velocity tracking")
        ax.legend(loc="best")
        save(fig, "w_timeseries")

        # 7) computation proxy
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["apfm_period_ms"], label="APFM period [ms] (proxy)")
        ax.plot(t, self.series["metrics_compute_ms"], label="metrics compute [ms]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("ms")
        ax.set_title("Computation proxies")
        ax.legend(loc="best")
        save(fig, "compute_timeseries")

        # 8) CEω
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["cew"], label="CEω running")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("[-]")
        ax.set_title("Control Effort (CEω) running")
        ax.legend(loc="best")
        save(fig, "cew_timeseries")

        # 9) CTE
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, self.series["cte"], label="CTE [m]")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("m")
        ax.set_title("Cross-track error (to straight start-goal line)")
        ax.legend(loc="best")
        save(fig, "cte_timeseries")

        rospy.loginfo(f"[runner] Plots saved to: {out_dir}")


if __name__ == "__main__":
    ExperimentBagAndMetricsRunner()
    rospy.spin()
