#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
APF Harmônico Fechado (NZ poucos modos + EZ com VTO) + conversão Força -> (v_ref, w_ref) em UM ÚNICO NÓ.

- Subs:
  /scenario/output_robot        (dynamic_obstacle_avoidance/RobotState)
  /scenario/output_obstacles    (dynamic_obstacle_avoidance/ObstacleArray)
  /scenario/goal                (geometry_msgs/Vector3)

- Pubs (mantém compatibilidade com seu pipeline):
  /apfm/total_force             (geometry_msgs/Vector3)
  /apfm/attractive_force        (geometry_msgs/Vector3)
  /apfm/repulsive_force         (geometry_msgs/Vector3)
  /apfm/dynamic_force           (geometry_msgs/Vector3)   # aqui = NZ harmônico
  /apfm/static_force            (geometry_msgs/Vector3)   # fica zerado (como você pediu: estático = dinâmico)
  /apfm/emergency_force         (geometry_msgs/Vector3)   # aqui = EZ harmônico (VTO only)

  /migbot/v_ref                 (std_msgs/Float64)
  /migbot/w_ref                 (std_msgs/Float64)

  /obstacle_avoidance/distance_to_goal (std_msgs/Float64)
  /obstacle_avoidance/collision        (std_msgs/Bool)
  /obstacle_avoidance/custom_info      (dynamic_obstacle_avoidance/CustomInfo)  # opcional

Como usar:
  chmod +x apf_harmonic_closed_all_in_one.py
  rosrun <seu_pacote> apf_harmonic_closed_all_in_one.py

Parâmetros principais (ROS param ~):
  # Geometria / Lyu
  safe_distance, robot_domain_radius, safety_margin_radius
  obstacle_influence_range (rho0), emergency_band (rho_E)

  # Harmônico NZ (poucos modos: 0+1)
  nz_c0, nz_A1, nz_B1

  # Harmônico EZ (poucos modos: 0+2) * ||Vto||^2 (VTO only)
  ez_c0, ez_A2, ez_B2

  # Atração
  att_gain

  # Força -> (v,w)
  v_max, w_max, v_cruise, stop_tolerance, k_heading, heading_deadband_deg
  use_alignment_speed_scaling, alignment_power, min_alignment_scale

  # Política de lado
  side_policy: 'starboard' ou 'cross'
"""

import math
import numpy as np
import rospy

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Bool
from tf.transformations import euler_from_quaternion

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray, CustomInfo


EPS = 1e-9


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def unit(v: np.ndarray) -> np.ndarray:
    n = norm(v)
    if n < EPS:
        return np.zeros_like(v, dtype=float)
    return v / n


def rot_ccw(v: np.ndarray) -> np.ndarray:
    return np.array([-v[1], v[0]], dtype=float)


def rot_cw(v: np.ndarray) -> np.ndarray:
    return np.array([v[1], -v[0]], dtype=float)


def cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def angle(v: np.ndarray) -> float:
    return math.atan2(float(v[1]), float(v[0]))


def theta_m_lyu(d: float, dm: float) -> float:
    # theta_m = arctan(dm / sqrt(d^2 - dm^2))  (equivalente a arcsin(dm/d))
    if d <= dm + 1e-12:
        return math.pi / 2.0
    return math.atan2(dm, math.sqrt(max(d * d - dm * dm, 1e-12)))


class HarmonicClosedAPFAllInOne:
    def __init__(self):
        rospy.init_node("apf_harmonic_closed_all_in_one", anonymous=False)

        # ---------------- Topics ----------------
        self.robot_state_topic = rospy.get_param("~robot_state_topic", "/scenario/output_robot")
        self.obstacles_topic = rospy.get_param("~obstacles_topic", "/scenario/output_obstacles")
        self.goal_topic = rospy.get_param("~goal_topic", "/scenario/goal")

        self.v_ref_topic = rospy.get_param("~v_ref_topic", "/migbot/v_ref")
        self.w_ref_topic = rospy.get_param("~w_ref_topic", "/migbot/w_ref")

        # ---------------- ROS loop rate ----------------
        self.dt = float(rospy.get_param("~dt", 0.1))  # 10 Hz default

        # ---------------- Geometry / safety ----------------
        self.safe_distance = float(rospy.get_param("~safe_distance", 10.0))
        self.robot_domain_radius = float(rospy.get_param("~robot_domain_radius", 1.4))
        self.safety_margin_radius = float(rospy.get_param("~safety_margin_radius", 0.6))

        # rho0 / influence range
        self.obstacle_influence_range = float(rospy.get_param("~obstacle_influence_range", 3.0 * self.safe_distance))

        # emergency thin band rho_E (em metros)
        self.rho_E = float(rospy.get_param("~emergency_band", 0.5))

        # ---------------- Attraction ----------------
        self.att_gain = float(rospy.get_param("~att_gain", 3000.0))

        # ---------------- Harmonic NZ: mode 0 + mode 1 ----------------
        # g_NZ(α) = c0 + A1 cos α + B1 sin α
        self.nz_c0 = float(rospy.get_param("~nz_c0", 2000.0))
        self.nz_A1 = float(rospy.get_param("~nz_A1", 2000.0))
        self.nz_B1 = float(rospy.get_param("~nz_B1", 1.0))  # bias de lado (0 desliga)

        # ---------------- Harmonic EZ: mode 0 + mode 2, scaled by ||Vto||^2 ----------------
        # g_EZ(α) = ||Vto||^2 * (c0E + A2 cos 2α + B2 sin 2α)
        self.ez_c0 = float(rospy.get_param("~ez_c0", 100.0))
        self.ez_A2 = float(rospy.get_param("~ez_A2", 100.0))
        self.ez_B2 = float(rospy.get_param("~ez_B2", 0.0))

        # ---------------- Side policy ----------------
        # 'starboard' (estilo Lyu) ou 'cross' (sinal de cross(p_ot, v_to))
        self.side_policy = str(rospy.get_param("~side_policy", "starboard")).strip().lower()

        # ---------------- Force -> (v,w) ----------------
        self.v_max = float(rospy.get_param("~v_max", 2.0))
        self.w_max = float(rospy.get_param("~w_max", (2.0 * math.pi) / 6.0))
        self.v_cruise = float(rospy.get_param("~v_cruise", self.v_max))
        self.stop_tol = float(rospy.get_param("~stop_tolerance", 5.0))

        default_k = self.w_max / (math.pi / 2.0)
        self.k_heading = float(rospy.get_param("~k_heading", default_k))
        self.heading_deadband = math.radians(float(rospy.get_param("~heading_deadband_deg", 2.0)))

        self.use_align_scale = bool(rospy.get_param("~use_alignment_speed_scaling", True))
        self.align_power = int(rospy.get_param("~alignment_power", 4))
        self.min_align_scale = float(rospy.get_param("~min_alignment_scale", 0.0))
        self.force_deadband = float(rospy.get_param("~force_deadband", 1e-6))

        # ---------------- Publishers: forces (compat) ----------------
        self.repulsive_force_pub = rospy.Publisher("/apfm/repulsive_force", Vector3, queue_size=10)
        self.attractive_force_pub = rospy.Publisher("/apfm/attractive_force", Vector3, queue_size=10)
        self.total_force_pub = rospy.Publisher("/apfm/total_force", Vector3, queue_size=10)
        self.dynamic_force_pub = rospy.Publisher("/apfm/dynamic_force", Vector3, queue_size=10)      # NZ
        self.static_force_pub = rospy.Publisher("/apfm/static_force", Vector3, queue_size=10)        # 0
        self.emergency_force_pub = rospy.Publisher("/apfm/emergency_force", Vector3, queue_size=10)  # EZ

        # ---------------- Publishers: refs ----------------
        self.pub_v = rospy.Publisher(self.v_ref_topic, Float64, queue_size=10)
        self.pub_w = rospy.Publisher(self.w_ref_topic, Float64, queue_size=10)

        # ---------------- Debug / info ----------------
        self.distance_to_goal_pub = rospy.Publisher("/obstacle_avoidance/distance_to_goal", Float64, queue_size=10)
        self.collision_pub = rospy.Publisher("/obstacle_avoidance/collision", Bool, queue_size=10)
        self.custom_info_pub = rospy.Publisher("/obstacle_avoidance/custom_info", CustomInfo, queue_size=10)

        self.pub_heading_err = rospy.Publisher("~debug/heading_error", Float64, queue_size=10)
        self.pub_force_angle = rospy.Publisher("~debug/force_angle", Float64, queue_size=10)
        self.pub_yaw = rospy.Publisher("~debug/yaw", Float64, queue_size=10)

        # ---------------- Subscribers ----------------
        self.robot_state = None
        self.obstacles = None
        self.goal = np.array([0.0, 0.0], dtype=float)

        rospy.Subscriber(self.robot_state_topic, RobotState, self.cb_robot, queue_size=10)
        rospy.Subscriber(self.obstacles_topic, ObstacleArray, self.cb_obstacles, queue_size=10)
        rospy.Subscriber(self.goal_topic, Vector3, self.cb_goal, queue_size=10)

        rospy.Timer(rospy.Duration(self.dt), self.on_timer)

        rospy.loginfo("[APF Harmonic Closed AllInOne] started")
        rospy.loginfo("  robot_state_topic: %s", self.robot_state_topic)
        rospy.loginfo("  obstacles_topic:   %s", self.obstacles_topic)
        rospy.loginfo("  goal_topic:        %s", self.goal_topic)
        rospy.loginfo("  v_ref_topic:       %s", self.v_ref_topic)
        rospy.loginfo("  w_ref_topic:       %s", self.w_ref_topic)

    # ---------- Callbacks ----------
    def cb_robot(self, msg: RobotState):
        self.robot_state = msg

    def cb_obstacles(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def cb_goal(self, msg: Vector3):
        self.goal = np.array([float(msg.x), float(msg.y)], dtype=float)

    # ---------- Side choice ----------
    def choose_side_sign(self, p_ot: np.ndarray, v_to: np.ndarray, heading_dir: np.ndarray) -> float:
        """
        Retorna s em {+1,-1} para bias no termo tangencial do potencial.
        Aqui e_theta é construído a partir de e_r = (obstáculo -> OS).
        """
        e_r = unit(-p_ot)     # obstáculo -> OS
        e_th = rot_ccw(e_r)   # tangencial CCW em torno do obstáculo

        if self.side_policy == "cross":
            # sinal do cross(p_ot, v_to): simples e “local”
            return 1.0 if cross2(p_ot, v_to) >= 0.0 else -1.0

        # default: starboard-like (usa heading do OS)
        starboard = unit(rot_cw(heading_dir))  # 90 deg CW do heading
        cand1 = unit(rot_cw(unit(p_ot)))       # perp CW do n_ot (OS->TS)
        cand2 = unit(rot_ccw(unit(p_ot)))
        n_perp = cand1 if float(np.dot(cand1, starboard)) >= float(np.dot(cand2, starboard)) else cand2

        # converte para sinal no eixo e_th (obstáculo->OS)
        return 1.0 if float(np.dot(n_perp, e_th)) >= 0.0 else -1.0

    # ---------- Harmonic closed-form gradients ----------
    def grad_nz(self, r: float, alpha: float, dm: float, CR: float, s: float):
        """
        NZ: anel (a=dm, b=CR)
        phi_NZ = c0 ln(b/r)/ln(b/a) + R1(r)[A1 cos a + B1 sin a]
        retorna: (dphi_dr, (1/r) dphi_dalpha)
        """
        a = max(dm, 1e-6)
        b = max(CR, a + 1e-6)
        r = float(np.clip(r, a + 1e-6, b - 1e-6))

        ln_ba = math.log(b / a)
        D1 = a - (b * b) / a
        R1 = (r - (b * b) / r) / D1
        dR1 = (1.0 + (b * b) / (r * r)) / D1

        c0 = self.nz_c0
        A1 = self.nz_A1
        B1 = s * self.nz_B1

        base = (A1 * math.cos(alpha) + B1 * math.sin(alpha))

        dphi_dr = -c0 / (r * ln_ba) + dR1 * base
        v_alpha = (R1 / r) * (-A1 * math.sin(alpha) + B1 * math.cos(alpha))  # (1/r)*dphi/dalpha
        return dphi_dr, v_alpha

    def grad_ez(self, r: float, alpha: float, dm: float, s: float, vto2: float):
        """
        EZ: anel fino (a=dm, b=dm+rho_E)
        phi_EZ = vto2[ c0 ln(b/r)/ln(b/a) + R2(r)(A2 cos2a + B2 sin2a) ]
        retorna: (dphi_dr, (1/r) dphi_dalpha)
        """
        a = max(dm, 1e-6)
        b = max(dm + max(self.rho_E, 1e-3), a + 1e-3)
        r = float(np.clip(r, a + 1e-6, b - 1e-6))

        ln_ba = math.log(b / a)
        D2 = a * a - (b ** 4) / (a * a)
        R2 = (r * r - (b ** 4) / (r * r)) / D2
        dR2 = (2.0 * r + 2.0 * (b ** 4) / (r ** 3)) / D2

        c0 = self.ez_c0
        A2 = self.ez_A2
        B2 = s * self.ez_B2

        c2 = math.cos(2.0 * alpha)
        s2 = math.sin(2.0 * alpha)
        base = (A2 * c2 + B2 * s2)

        dphi_dr = vto2 * (-c0 / (r * ln_ba) + dR2 * base)
        v_alpha = vto2 * ((2.0 * R2 / r) * (-A2 * s2 + B2 * c2))  # (1/r)*dphi/dalpha
        return dphi_dr, v_alpha

    # ---------- Force computation ----------
    def compute_forces(self):
        if self.robot_state is None or self.obstacles is None:
            return None

        # OS state
        p_os = np.array([self.robot_state.position.x, self.robot_state.position.y], dtype=float)
        v_os = np.array([self.robot_state.velocity.x, self.robot_state.velocity.y], dtype=float)

        # yaw
        q = self.robot_state.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = wrap_pi(yaw)
        heading_dir = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)

        # goal
        p_g = self.goal
        p_og = p_g - p_os
        dist_goal = norm(p_og)
        n_og = unit(p_og)

        self.distance_to_goal_pub.publish(Float64(dist_goal))

        # attraction (simples, compatível com seu pipeline)
        F_att = self.att_gain * dist_goal * n_og

        # repulsion
        F_rep = np.zeros(2, dtype=float)
        F_nz = np.zeros(2, dtype=float)  # dynamic_force_pub
        F_ez = np.zeros(2, dtype=float)  # emergency_force_pub
        F_static = np.zeros(2, dtype=float)  # você pediu: estático = dinâmico (mantém 0 aqui)

        # collision threshold (igual tua ideia: safety_margin + robot_radius)
        tau = self.safety_margin_radius + self.robot_domain_radius
        any_collision = False

        # Para debug/info: publica info do obstáculo mais crítico (menor d)
        best_ci = None
        best_d = float("inf")

        for ob in self.obstacles:
            p_ts = np.array([ob.position.x, ob.position.y], dtype=float)
            v_ts = np.array([ob.velocity.x, ob.velocity.y], dtype=float)
            R_ts = float(ob.radius)

            p_ot = p_ts - p_os   # OS -> TS
            d = norm(p_ot)

            if d < best_d:
                best_d = d
                best_ci = (p_ts, v_ts, R_ts, p_ot, d)

            if d < tau:
                any_collision = True

            dm = self.robot_domain_radius + self.safe_distance + R_ts
            CR = dm + self.obstacle_influence_range

            if d > CR:
                continue

            # velocidade relativa: se o OS estiver parado, use a velocidade desejada rumo ao goal
            v_to = v_os - v_ts
            if norm(v_to) < 1e-3:
                v_des = self.v_cruise * n_og          # direção do goal
                v_to = v_des - v_ts

            vto2 = float(np.dot(v_to, v_to))

            # alpha bem definido
            alpha = wrap_pi(angle(p_ot) - angle(v_to))

            th = abs(alpha)
            thm = theta_m_lyu(d, dm)

            # unit vectors (obstáculo -> OS)
            e_r = unit(p_os - p_ts)
            e_th = rot_ccw(e_r)

            s = self.choose_side_sign(p_ot, v_to, heading_dir)

            # EZ: d <= dm  (VTO only HERE)
            if d <= dm:
                dphi_dr, v_alpha = self.grad_ez(d, alpha, dm, s, vto2)
                F = -(dphi_dr * e_r + v_alpha * e_th)
                F_ez += F
                F_rep += F
                continue

            # NZ: dm < d <= CR AND theta < theta_m
            if th < thm:
                dphi_dr, v_alpha = self.grad_nz(d, alpha, dm, CR, s)
                F = -(dphi_dr * e_r + v_alpha * e_th)
                F_nz += F
                F_rep += F

        self.collision_pub.publish(Bool(any_collision))

        # publish custom_info (apenas para o obstáculo mais próximo, para não “spammar”)
        if best_ci is not None:
            p_ts, v_ts, R_ts, p_ot, d = best_ci
            dm = self.robot_domain_radius + self.safe_distance + R_ts
            CR = dm + self.obstacle_influence_range
            v_to = v_os - v_ts

            ci = CustomInfo()
            ci.distance_to_obstacle = float(d)
            ci.center_to_center_safe_distance = float(dm)
            ci.collision_avoidance_radius = float(CR)
            ci.vector_to_obstacle = p_ot.tolist()
            ci.relative_speed_vector = v_to.tolist()
            ci.obstacle_domain_radius = float(R_ts)
            ci.distance_to_goal = float(dist_goal)
            ci.dm = float(dm)
            ci.CR = float(CR)
            # campos extras podem existir no seu msg; manter simples para robustez
            self.custom_info_pub.publish(ci)

        F_total = F_att + F_rep
        return yaw, dist_goal, F_total, F_att, F_rep, F_nz, F_static, F_ez

    # ---------- Force -> (v_ref,w_ref) ----------
    def force_to_vw(self, yaw: float, dist_goal: float, F: np.ndarray):
        Fx, Fy = float(F[0]), float(F[1])
        Fnorm = math.hypot(Fx, Fy)

        if Fnorm < self.force_deadband:
            return 0.0, 0.0, 0.0, 0.0

        force_angle = wrap_pi(math.atan2(Fy, Fx))
        heading_error = wrap_pi(force_angle - yaw)

        if abs(heading_error) < self.heading_deadband:
            heading_error = 0.0

        w_ref = self.k_heading * heading_error
        w_ref = max(-self.w_max, min(self.w_max, w_ref))

        v_ref = self.v_cruise

        if self.use_align_scale:
            align = max(0.0, math.cos(heading_error)) ** max(self.align_power, 1)
            align = max(self.min_align_scale, min(1.0, align))
            v_ref *= align

        # stop near goal
        if dist_goal <= self.stop_tol:
            v_ref = 0.0
            w_ref = 0.0
        else:
            scale = min(1.0, max(0.0, (dist_goal - self.stop_tol) / max(self.stop_tol, 1e-3)))
            v_ref *= scale

        v_ref = max(0.0, min(self.v_max, v_ref))
        return v_ref, w_ref, heading_error, force_angle

    # ---------- Publish helper ----------
    @staticmethod
    def pub_vec(pub, v2: np.ndarray):
        pub.publish(Vector3(x=float(v2[0]), y=float(v2[1]), z=0.0))

    # ---------- Timer ----------
    def on_timer(self, _evt):
        out = self.compute_forces()
        if out is None:
            return

        yaw, dist_goal, F_total, F_att, F_rep, F_nz, F_static, F_ez = out

        # publish forces (compatibilidade com seu setup)
        self.pub_vec(self.total_force_pub, F_total)
        self.pub_vec(self.attractive_force_pub, F_att)
        self.pub_vec(self.repulsive_force_pub, F_rep)
        self.pub_vec(self.dynamic_force_pub, F_nz)
        self.pub_vec(self.static_force_pub, F_nz)
        self.pub_vec(self.emergency_force_pub, F_ez)

        # convert to v,w and publish
        v_ref, w_ref, heading_error, force_angle = self.force_to_vw(yaw, dist_goal, F_total)
        self.pub_v.publish(Float64(v_ref))
        self.pub_w.publish(Float64(w_ref))

        # debug
        self.pub_heading_err.publish(Float64(heading_error))
        self.pub_force_angle.publish(Float64(force_angle))
        self.pub_yaw.publish(Float64(yaw))


if __name__ == "__main__":
    try:
        HarmonicClosedAPFAllInOne()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
