#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Bool
from tf.transformations import euler_from_quaternion

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray

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


def rot_cw(v: np.ndarray) -> np.ndarray:
    # 90° clockwise: [x,y] -> [y, -x]
    return np.array([v[1], -v[0]], dtype=float)


class Jadhav2023AllInOne:
    """
    Implementa Jadhav (2023) exatamente nos blocos principais:

      - Eqs (24)–(28): sink e vortex em coordenadas polares (campo harmônico)
      - Eq (29): gamma
      - Eq (32): decomposição do V_rel em (v_r, v_theta) (ASSINADA!)
      - Eqs (33)–(34): K_vor modificado (piecewise + fator f)
      - K=0 para |gamma|>5pi/8 (overtaking)

    Interface ROS compatível com seu projeto:
      subs: /scenario/output_robot, /scenario/output_obstacles, /scenario/goal
      pubs: /apfm/*_force, /migbot/v_ref, /migbot/w_ref, /obstacle_avoidance/*
    """

    def __init__(self):
        rospy.init_node("apf_jadhav_2023_all_in_one_v2", anonymous=False)

        # Topics
        self.robot_state_topic = rospy.get_param("~robot_state_topic", "/scenario/output_robot")
        self.obstacles_topic = rospy.get_param("~obstacles_topic", "/scenario/output_obstacles")
        self.goal_topic = rospy.get_param("~goal_topic", "/scenario/goal")
        self.v_ref_topic = rospy.get_param("~v_ref_topic", "/migbot/v_ref")
        self.w_ref_topic = rospy.get_param("~w_ref_topic", "/migbot/w_ref")

        self.dt = float(rospy.get_param("~dt", 0.1))

        # Jadhav parameters (use sinais como no paper: Lambda<0 sink, K0_vor tipicamente <0 nos exemplos)
        self.Lambda_sink = float(rospy.get_param("~Lambda_sink", -100.0))
        self.K0_vor = float(rospy.get_param("~K0_vor", -10.0))  # paper usa -10 nas figuras

        # R_safe (raio de sensoriamento/atuação do obstáculo)
        self.R_safe = float(rospy.get_param("~R_safe", 55.0))

        # R_tol (raio de colisão/limiar do Eq. 33)
        self.R_tol = float(rospy.get_param("~R_tol", 3.0))

        # opcional: ignore obstáculos muito longe (não é do paper, mas ajuda performance)
        self.R_influence = float(rospy.get_param("~R_influence", 200.0))

        # conversão para v,w (mesma filosofia do teu APF tradicional)
        self.v_max = float(rospy.get_param("~v_max", 2.0))
        self.w_max = float(rospy.get_param("~w_max", (2.0 * math.pi) / 6.0))
        self.v_cruise = float(rospy.get_param("~v_cruise", 1.5))
        self.stop_tol = float(rospy.get_param("~stop_tolerance", 5.0))

        default_k = self.w_max / (math.pi / 2.0)
        self.k_heading = float(rospy.get_param("~k_heading", default_k))
        self.heading_deadband = math.radians(float(rospy.get_param("~heading_deadband_deg", 2.0)))

        self.use_align_scale = bool(rospy.get_param("~use_alignment_speed_scaling", True))
        self.align_power = int(rospy.get_param("~alignment_power", 4))
        self.min_align_scale = float(rospy.get_param("~min_alignment_scale", 0.05))

        # Publishers
        self.repulsive_force_pub = rospy.Publisher("/apfm/repulsive_force", Vector3, queue_size=10)
        self.attractive_force_pub = rospy.Publisher("/apfm/attractive_force", Vector3, queue_size=10)
        self.total_force_pub = rospy.Publisher("/apfm/total_force", Vector3, queue_size=10)
        self.dynamic_force_pub = rospy.Publisher("/apfm/dynamic_force", Vector3, queue_size=10)
        self.static_force_pub = rospy.Publisher("/apfm/static_force", Vector3, queue_size=10)
        self.emergency_force_pub = rospy.Publisher("/apfm/emergency_force", Vector3, queue_size=10)

        self.pub_v = rospy.Publisher(self.v_ref_topic, Float64, queue_size=10)
        self.pub_w = rospy.Publisher(self.w_ref_topic, Float64, queue_size=10)

        self.distance_to_goal_pub = rospy.Publisher("/obstacle_avoidance/distance_to_goal", Float64, queue_size=10)
        self.collision_pub = rospy.Publisher("/obstacle_avoidance/collision", Bool, queue_size=10)

        # Subs
        self.robot_state = None
        self.obstacles = None
        self.goal = np.array([0.0, 0.0], dtype=float)

        rospy.Subscriber(self.robot_state_topic, RobotState, self.cb_robot, queue_size=10)
        rospy.Subscriber(self.obstacles_topic, ObstacleArray, self.cb_obstacles, queue_size=10)
        rospy.Subscriber(self.goal_topic, Vector3, self.cb_goal, queue_size=10)

        rospy.Timer(rospy.Duration(self.dt), self.on_timer)
        rospy.loginfo("[Jadhav 2023 v2] started")

    def cb_robot(self, msg: RobotState):
        self.robot_state = msg

    def cb_obstacles(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def cb_goal(self, msg: Vector3):
        self.goal = np.array([float(msg.x), float(msg.y)], dtype=float)

    # --------- Eqs (25)–(28): sink/vortex fields em forma vetorial ---------

    def sink_at_goal(self, p: np.ndarray, g: np.ndarray) -> np.ndarray:
        # v = (Lambda / (2*pi*r)) * e_r   (Eq 26)
        rvec = p - g
        r = norm(rvec)
        if r < 1e-6:
            return np.zeros(2, dtype=float)
        return (self.Lambda_sink / (2.0 * math.pi * r)) * (rvec / r)

    def vortex_at_obstacle(self, p: np.ndarray, o: np.ndarray, K: float) -> np.ndarray:
        # v = (K / (2*pi*r)) * e_theta  (Eq 28)
        rvec = p - o
        r = norm(rvec)
        if r < 1e-6:
            return np.zeros(2, dtype=float)
        e_r = rvec / r
        # paper define θ positivo no sentido horário => e_theta = rot_cw(e_r)
        e_theta = unit(rot_cw(e_r))
        return (K / (2.0 * math.pi * r)) * e_theta

    # --------- Eqs (29), (32), (33), (34): K_vor modificado EXATO ---------

    def compute_gamma(self, p_n: np.ndarray, yaw: float, p_o: np.ndarray) -> float:
        # gamma = atan2(y_o - y_n, x_o - x_n) - psi  (Eq 29)
        bearing = math.atan2(p_o[1] - p_n[1], p_o[0] - p_n[0])
        return wrap_pi(bearing - yaw)

    def vr_vtheta_from_Vrel(self, V_rel: np.ndarray, gamma: float) -> tuple:
        # [v_r; v_theta] = [[cosγ sinγ],[-sinγ cosγ]] V_rel   (Eq 32)
        cx = math.cos(gamma)
        sx = math.sin(gamma)
        vr = cx * V_rel[0] + sx * V_rel[1]
        vth = -sx * V_rel[0] + cx * V_rel[1]
        return vr, vth

    def modified_Kvor(self, p_n, v_n, yaw, p_o, v_o, d) -> tuple:
        """
        Retorna (Kvor, vr, vth, gamma, f) seguindo Eq (33)-(34).
        Observação: v_n e v_o aqui assumidos no mesmo frame (GCS), como seus msgs sugerem.
        """
        gamma = self.compute_gamma(p_n, yaw, p_o)

        # K=0 se obstáculo "não é encontro head-on/crossing": |gamma| > 5pi/8  (texto após Eq 33)
        if abs(gamma) > (5.0 * math.pi / 8.0):
            return 0.0, 0.0, 0.0, gamma, 1.0

        # paper define V_rel para dinâmico (Eq 31). Sem heading do obstáculo, usamos V_rel = v_o - v_n.
        V_rel = (v_o - v_n)

        vr, vth = self.vr_vtheta_from_Vrel(V_rel, gamma)

        # Eq (33): K=0 se v_theta > (-2*R_tol/d^2) * v_r
        thresh = (-2.0 * self.R_tol / (d * d)) * vr
        if vth > thresh:
            return 0.0, vr, vth, gamma, 1.0

        # Eq (34): f = max{1, (2 - ||Xn-Xo||^2 / R_safe - v_r)}
        # (exato como aparece no paper; em unidades reais pode ficar <=1 na maior parte do tempo)
        f_expr = 2.0 - (d * d) / max(self.R_safe, 1e-6) - vr
        f = max(1.0, f_expr)

        return f * self.K0_vor, vr, vth, gamma, f

    # --------- loop ---------

    def on_timer(self, _evt):
        if self.robot_state is None or self.obstacles is None:
            return

        p_n = np.array([float(self.robot_state.position.x),
                        float(self.robot_state.position.y)], dtype=float)

        q = [self.robot_state.orientation.x,
             self.robot_state.orientation.y,
             self.robot_state.orientation.z,
             self.robot_state.orientation.w]
        _, _, yaw = euler_from_quaternion(q)

        v_n = np.array([float(self.robot_state.velocity.x),
                        float(self.robot_state.velocity.y)], dtype=float)

        g = self.goal
        d_goal = norm(g - p_n)
        self.distance_to_goal_pub.publish(Float64(d_goal))

        if d_goal <= self.stop_tol:
            self._publish_all(np.zeros(2), np.zeros(2), np.zeros(2))
            self.pub_v.publish(Float64(0.0))
            self.pub_w.publish(Float64(0.0))
            return

        # campo do sink no goal
        F_att = self.sink_at_goal(p_n, g)

        F_rep = np.zeros(2, dtype=float)
        collision = False

        # acumula vórtices
        for ob in self.obstacles:
            p_o = np.array([float(ob.position.x), float(ob.position.y)], dtype=float)
            v_o = np.array([float(ob.velocity.x), float(ob.velocity.y)], dtype=float)

            d = norm(p_n - p_o)
            if d < 1e-6:
                continue

            if d < 2.0:  # só um flag simples
                collision = True

            if d > self.R_influence:
                continue

            # paper: começa a "agir" quando entra no R_safe (sensoriamento)
            if d > self.R_safe:
                continue

            K, vr, vth, gamma, f = self.modified_Kvor(p_n, v_n, yaw, p_o, v_o, d)

            rospy.loginfo_throttle(
                1.0,
                f"[Jadhav] d={d:.2f} gamma={gamma:.2f} vr={vr:.2f} vth={vth:.2f} f={f:.2f} K={K:.2f}"
            )

            if abs(K) > 0.0:
                F_rep += self.vortex_at_obstacle(p_n, p_o, K)

        F_total = F_att + F_rep

        self.collision_pub.publish(Bool(collision))
        self._publish_all(F_total, F_att, F_rep)

        # Força/campo -> v,w
        if norm(F_total) < 1e-6:
            self.pub_v.publish(Float64(0.0))
            self.pub_w.publish(Float64(0.0))
            return

        desired_heading = math.atan2(F_total[1], F_total[0])
        yaw_err = wrap_pi(desired_heading - yaw)

        if abs(yaw_err) < self.heading_deadband:
            yaw_err = 0.0

        w_ref = float(np.clip(self.k_heading * yaw_err, -self.w_max, self.w_max))

        v_ref = self.v_cruise
        if self.use_align_scale:
            align = max(0.0, math.cos(yaw_err))
            scale = max(self.min_align_scale, align ** self.align_power)
            v_ref *= scale

        v_ref = float(np.clip(v_ref, 0.0, self.v_max))

        self.pub_v.publish(Float64(v_ref))
        self.pub_w.publish(Float64(w_ref))

    def _publish_all(self, F_total, F_att, F_rep):
        self.total_force_pub.publish(Vector3(float(F_total[0]), float(F_total[1]), 0.0))
        self.attractive_force_pub.publish(Vector3(float(F_att[0]), float(F_att[1]), 0.0))
        self.repulsive_force_pub.publish(Vector3(float(F_rep[0]), float(F_rep[1]), 0.0))

        # compat
        self.dynamic_force_pub.publish(Vector3(0.0, 0.0, 0.0))
        self.static_force_pub.publish(Vector3(float(F_rep[0]), float(F_rep[1]), 0.0))
        self.emergency_force_pub.publish(Vector3(0.0, 0.0, 0.0))


if __name__ == "__main__":
    Jadhav2023AllInOne()
    rospy.spin()
