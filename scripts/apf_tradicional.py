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


class ClassicAPFAllInOne:
    """
    APF clássico (Khatib):
      - F_att = k_att * (p_goal - p)
      - F_rep_i = k_rep * (1/d - 1/Q*) * (1/d^2) * (p - p_obs)/d   se d <= Q*
    e conversão Força -> (v_ref, w_ref) no mesmo nó.
    """

    def __init__(self):
        rospy.init_node("apf_classic_all_in_one", anonymous=False)

        # ---- Topics ----
        self.robot_state_topic = rospy.get_param("~robot_state_topic", "/scenario/output_robot")
        self.obstacles_topic = rospy.get_param("~obstacles_topic", "/scenario/output_obstacles")
        self.goal_topic = rospy.get_param("~goal_topic", "/scenario/goal")

        self.v_ref_topic = rospy.get_param("~v_ref_topic", "/migbot/v_ref")
        self.w_ref_topic = rospy.get_param("~w_ref_topic", "/migbot/w_ref")

        self.dt = float(rospy.get_param("~dt", 0.1))

        # ---- Geometria / distâncias ----
        self.robot_domain_radius = float(rospy.get_param("~robot_domain_radius", 1.4))
        self.safe_distance = float(rospy.get_param("~safe_distance", 10.0))
        self.safety_margin_radius = float(rospy.get_param("~safety_margin_radius", 0.6))

        # ---- APF clássico: ganhos ----
        self.k_att = float(rospy.get_param("~k_att", 30.0))     # ajuste
        self.k_rep = float(rospy.get_param("~k_rep", 2000.0))   # ajuste

        # Raio de influência Q* (influence distance)
        # Tipicamente ~ 2~5x dm. Aqui deixo por param.
        self.Q_star = float(rospy.get_param("~Q_star", 40.0))

        # ---- Força -> (v,w) ----
        self.v_max = float(rospy.get_param("~v_max", 1.0))
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

        # ---- Publishers forces (mesmos tópicos) ----
        self.repulsive_force_pub = rospy.Publisher("/apfm/repulsive_force", Vector3, queue_size=10)
        self.attractive_force_pub = rospy.Publisher("/apfm/attractive_force", Vector3, queue_size=10)
        self.total_force_pub = rospy.Publisher("/apfm/total_force", Vector3, queue_size=10)

        # Para manter compatibilidade (APF clássico não separa NZ/EZ, então publico 0 nesses):
        self.dynamic_force_pub = rospy.Publisher("/apfm/dynamic_force", Vector3, queue_size=10)
        self.static_force_pub = rospy.Publisher("/apfm/static_force", Vector3, queue_size=10)
        self.emergency_force_pub = rospy.Publisher("/apfm/emergency_force", Vector3, queue_size=10)

        # ---- Publishers refs ----
        self.pub_v = rospy.Publisher(self.v_ref_topic, Float64, queue_size=10)
        self.pub_w = rospy.Publisher(self.w_ref_topic, Float64, queue_size=10)

        # ---- Debug/info ----
        self.distance_to_goal_pub = rospy.Publisher("/obstacle_avoidance/distance_to_goal", Float64, queue_size=10)
        self.collision_pub = rospy.Publisher("/obstacle_avoidance/collision", Bool, queue_size=10)

        # ---- Subscribers ----
        self.robot_state = None
        self.obstacles = None
        self.goal = np.array([0.0, 0.0], dtype=float)

        rospy.Subscriber(self.robot_state_topic, RobotState, self.cb_robot, queue_size=10)
        rospy.Subscriber(self.obstacles_topic, ObstacleArray, self.cb_obstacles, queue_size=10)
        rospy.Subscriber(self.goal_topic, Vector3, self.cb_goal, queue_size=10)

        rospy.Timer(rospy.Duration(self.dt), self.on_timer)
        rospy.loginfo("[Classic APF AllInOne] started")

    def cb_robot(self, msg: RobotState):
        self.robot_state = msg

    def cb_obstacles(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def cb_goal(self, msg: Vector3):
        self.goal = np.array([float(msg.x), float(msg.y)], dtype=float)

    @staticmethod
    def pub_vec(pub, v2: np.ndarray):
        pub.publish(Vector3(x=float(v2[0]), y=float(v2[1]), z=0.0))

    def compute_forces(self):
        if self.robot_state is None or self.obstacles is None:
            return None

        # Estado OS
        p_os = np.array([self.robot_state.position.x, self.robot_state.position.y], dtype=float)

        q = self.robot_state.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = wrap_pi(yaw)

        # Goal
        p_g = self.goal
        p_og = p_g - p_os
        dist_goal = norm(p_og)
        self.distance_to_goal_pub.publish(Float64(dist_goal))

        # Atrativo clássico
        F_att = self.k_att * p_og

        # Repulsivo clássico
        F_rep = np.zeros(2, dtype=float)

        tau = self.safety_margin_radius + self.robot_domain_radius
        any_collision = False

        for ob in self.obstacles:
            p_obs = np.array([ob.position.x, ob.position.y], dtype=float)
            R_obs = float(ob.radius)

            # distância centro-centro
            d_cc = norm(p_os - p_obs)

            if d_cc < tau:
                any_collision = True

            # distância "efetiva" até a borda (opcional; ajuda a ficar menos agressivo)
            # d = max(d_cc - (R_robot + R_obs), eps)
            d = max(d_cc - (self.robot_domain_radius + R_obs), 1e-3)

            # você pode incorporar safe_distance aqui (faz o repulsivo começar antes)
            d_safe = max(d - self.safe_distance, 1e-3)

            # ativa se dentro de Q*
            if d_safe <= self.Q_star:
                # direção do obstáculo -> robô
                eta = unit(p_os - p_obs)

                term = (1.0 / d_safe) - (1.0 / self.Q_star)
                mag = self.k_rep * term * (1.0 / (d_safe * d_safe))
                F_rep += mag * eta

        self.collision_pub.publish(Bool(any_collision))
        F_total = F_att + F_rep
        return yaw, dist_goal, F_total, F_att, F_rep

    def force_to_vw(self, yaw: float, dist_goal: float, F: np.ndarray):
        Fx, Fy = float(F[0]), float(F[1])
        Fnorm = math.hypot(Fx, Fy)

        if Fnorm < self.force_deadband:
            return 0.0, 0.0

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
        return v_ref, w_ref

    def on_timer(self, _evt):
        out = self.compute_forces()
        if out is None:
            return

        yaw, dist_goal, F_total, F_att, F_rep = out

        # publish forces (compatibilidade)
        self.pub_vec(self.total_force_pub, F_total)
        self.pub_vec(self.attractive_force_pub, F_att)
        self.pub_vec(self.repulsive_force_pub, F_rep)

        # APF clássico não usa essas separações:
        z = np.zeros(2, dtype=float)
        self.pub_vec(self.dynamic_force_pub, z)
        self.pub_vec(self.static_force_pub, z)
        self.pub_vec(self.emergency_force_pub, z)

        # convert to v,w and publish
        v_ref, w_ref = self.force_to_vw(yaw, dist_goal, F_total)
        self.pub_v.publish(Float64(v_ref))
        self.pub_w.publish(Float64(w_ref))


if __name__ == "__main__":
    try:
        ClassicAPFAllInOne()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
