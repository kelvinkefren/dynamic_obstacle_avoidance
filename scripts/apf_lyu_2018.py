#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node: COLREGs-constrained Modified Artificial Potential Field (APF)
Faithful (equation-by-equation) implementation of Lyu & Yin (2018),
with *one-to-one* variable naming aligned to the paper notation.

This is a drop-in replacement for:
  dynamic_obstacle_avoidance/scripts/apfm_obstacle_avoidance.py

Publishes (same ROS interface as original pipeline):
  - /apfm/repulsive_force   (geometry_msgs/Vector3)
  - /apfm/attractive_force  (geometry_msgs/Vector3)
  - /apfm/total_force       (geometry_msgs/Vector3)
  - /apfm/dynamic_force     (geometry_msgs/Vector3)
  - /apfm/static_force      (geometry_msgs/Vector3)
  - /apfm/emergency_force   (geometry_msgs/Vector3)

Also publishes (like your auxiliary ObstacleAvoidance module):
  - /obstacle_avoidance/distance_to_goal (std_msgs/Float64)
  - /obstacle_avoidance/custom_info      (dynamic_obstacle_avoidance/CustomInfo)
  - /obstacle_avoidance/collision        (std_msgs/Bool)

Subscribes:
  - /scenario/output_robot     (dynamic_obstacle_avoidance/RobotState)
  - /scenario/output_obstacles (dynamic_obstacle_avoidance/ObstacleArray)
  - /scenario/goal             (geometry_msgs/Vector3)

──────────────────────────────────────────────────────────────────────────────
Paper-to-code notation mapping (used throughout):

Positions:
  p_os : position of own ship (OS)                 (robot position)
  p_ts : position of target ship / obstacle (TS)
  p_g  : goal position

Vectors and distances:
  p_ot = p_ts - p_os                              (vector OS→TS)
  ρ_ot = ||p_ot||                                  (distance OS-TS)
  ρ_og = ||p_g - p_os||                             (distance OS-goal)

Unit vectors:
  n_ot = p_ot / ||p_ot||                            (unit OS→TS)
  n_og = (p_g - p_os) / ||p_g - p_os||              (unit OS→goal)
  n_ot_perp : unit perpendicular to n_ot            (left/right)

Velocities:
  v_os : OS velocity
  v_ts : TS velocity
  v_to = v_os - v_ts                                (relative velocity)
  ||v_to|| : magnitude of relative velocity
  v_to,perp = v_to - (v_to·n_ot) n_ot                (perp component)
  ||v_to,perp|| : magnitude of perpendicular component

Safety / domains:
  R_os : OS domain radius
  R_ts : TS domain radius
  d_safe : desired safe distance between domains
  d_m = R_os + d_safe + R_ts                         (paper’s center-to-center safe distance)
  ρ_0 : obstacle influence range parameter
  CR = d_m + ρ_0                                     (collision avoidance radius / influence radius)
  τ : small positive constant in (d-τ) denominators  (paper uses tau to avoid singularity)

Angles:
  θ : angle between p_ot and v_to
  θ_m : half-angle of collision cone

Forces (paper):
  Eq. (6):   F_att
  Eq. (8–13): F_rd1 + F_rd2 + F_rd3  (dynamic obstacle repulsion)
  Eq. (14–15): F_rs1 + F_rs3         (static obstacle repulsion)
  Eq. (16–18): F_re1 + F_re2 + F_re3 (emergency repulsion)
  Total: F_total = F_att + Σ F_rep,i

COLREGs note:
  For dynamic obstacles inside CR and collision cone, we bias the lateral term F_rd2
  toward starboard (as described in the paper’s COLREGs discussion).
──────────────────────────────────────────────────────────────────────────────
"""

import math
from typing import Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray, CustomInfo


# ------------------------- small vector helpers -------------------------

def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def _unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = _norm(v)
    if n < eps:
        return np.zeros_like(v, dtype=float)
    return v / n


def _rot_ccw(v: np.ndarray) -> np.ndarray:
    """Rotate vector +90 degrees (CCW)."""
    return np.array([-v[1], v[0]], dtype=float)


def _rot_cw(v: np.ndarray) -> np.ndarray:
    """Rotate vector -90 degrees (CW)."""
    return np.array([v[1], -v[0]], dtype=float)


def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    """2D cross product z-component."""
    return float(a[0] * b[1] - a[1] * b[0])


def _angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Angle in radians in [0, pi]."""
    na = _norm(a)
    nb = _norm(b)
    if na < 1e-9 or nb < 1e-9:
        return math.pi
    c = float(np.dot(a, b) / (na * nb))
    c = _clamp(c, -1.0, 1.0)
    return math.acos(c)


def _quat_to_yaw(q) -> float:
    """Yaw from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ------------------------- Lyu (2018) APF node -------------------------

class APFMAvoidanceLyu2018FullIO:
    def __init__(self):
        rospy.init_node('apfm_avoidance')

        # ---- Paper parameters (renamed to match notation) ----
        # ε (epsilon): attractive scaling factor
        self.epsilon = float(rospy.get_param('~attraction_scaling_factor', 3000.0))

        # η_d, η_s, η_e: repulsive scaling factors (dynamic/static/emergency)
        self.eta_d = float(rospy.get_param('~obstacle_scaling_factor_dynamic', 2000.0))
        self.eta_s = float(rospy.get_param('~obstacle_scaling_factor_static', 300000.0))
        self.eta_e = float(rospy.get_param('~scaling_factor_emergency', 2000.0))

        # τ (tau): small positive constant used in (d - τ) denominators (paper)
        self.tau = float(rospy.get_param('~safety_margin_radius', 0.6))

        # R_os fallback param if RobotState.radius not set
        self.R_os_param = float(rospy.get_param('~robot_domain_radius', 1.4))

        # d_safe: desired safe distance (between domains)
        self.d_safe = float(rospy.get_param('~safe_distance', 10.0))

        # ρ_0: obstacle influence range parameter
        default_rho_0 = 3.0 * self.d_safe
        self.rho_0 = float(rospy.get_param('~obstacle_influence_range', default_rho_0))

        # Consider obstacle dynamic if ||v_ts|| > threshold (practical)
        self.v_ts_moving_threshold = float(rospy.get_param('~vts_moving_threshold', 1e-3))

        # Timer period
        self.dt = float(rospy.get_param('~dt', 0.1))

        # ---- Publishers (same as original pipeline) ----
        self.repulsive_force_pub = rospy.Publisher('/apfm/repulsive_force', Vector3, queue_size=10)
        self.attractive_force_pub = rospy.Publisher('/apfm/attractive_force', Vector3, queue_size=10)
        self.total_force_pub = rospy.Publisher('/apfm/total_force', Vector3, queue_size=10)
        self.dynamic_force_pub = rospy.Publisher('/apfm/dynamic_force', Vector3, queue_size=10)
        self.static_force_pub = rospy.Publisher('/apfm/static_force', Vector3, queue_size=10)
        self.emergency_force_pub = rospy.Publisher('/apfm/emergency_force', Vector3, queue_size=10)

        self.custom_info_pub = rospy.Publisher('/obstacle_avoidance/custom_info', CustomInfo, queue_size=10)
        self.distance_to_goal_pub = rospy.Publisher('/obstacle_avoidance/distance_to_goal', Float64, queue_size=10)
        self.collision_pub = rospy.Publisher('/obstacle_avoidance/collision', Bool, queue_size=10)

        # ---- Subscribers ----
        self.robot_sub = rospy.Subscriber('/scenario/output_robot', RobotState, self._robot_cb)
        self.obstacle_sub = rospy.Subscriber('/scenario/output_obstacles', ObstacleArray, self._obstacles_cb)
        self.goal_sub = rospy.Subscriber('/scenario/goal', Vector3, self._goal_cb)

        self.robot_state: Optional[RobotState] = None
        self.obstacles = None  # list[ObstacleState]
        self.p_g = np.array([0.0, 0.0], dtype=float)

        rospy.Timer(rospy.Duration(self.dt), self._on_timer)

    # ----------------- callbacks -----------------
    def _robot_cb(self, msg: RobotState):
        self.robot_state = msg

    def _obstacles_cb(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def _goal_cb(self, msg: Vector3):
        self.p_g = np.array([float(msg.x), float(msg.y)], dtype=float)

    # ----------------- core math (paper-aligned notation) -----------------

    def _F_att(self, p_os: np.ndarray, p_g: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray]:
        """
        Attractive force (Eq. 6):
          F_att = ε * ρ_og * n_og
        where:
          ρ_og = ||p_g - p_os||,  n_og = (p_g - p_os) / ρ_og
        """
        p_og = p_g - p_os
        rho_og = _norm(p_og)
        if rho_og < 1e-9:
            return np.zeros(2, dtype=float), 0.0, np.zeros(2, dtype=float)
        n_og = p_og / rho_og
        F_att = self.epsilon * rho_og * n_og
        return F_att, rho_og, n_og

    def _theta_m(self, rho_ot: float, d_m: float) -> float:
        """
        Collision cone half-angle θ_m:
          θ_m = arctan( d_m / sqrt(rho_ot^2 - d_m^2) ), for rho_ot > d_m
        else θ_m = π/2.
        """
        if rho_ot <= d_m:
            return math.pi / 2.0
        denom = max(rho_ot * rho_ot - d_m * d_m, 1e-12)
        return math.atan2(d_m, math.sqrt(denom))

    def _select_n_ot_perp_starboard(self, n_ot: np.ndarray, heading_dir: np.ndarray) -> np.ndarray:
        """
        Choose n_ot_perp (unit) such that it most aligns with starboard of OS heading.
        Starboard direction = CW rotation of heading direction.
        """
        starboard_dir = _unit(_rot_cw(heading_dir))
        cand_cw = _unit(_rot_cw(n_ot))
        cand_ccw = _unit(_rot_ccw(n_ot))
        return cand_cw if float(np.dot(cand_cw, starboard_dir)) >= float(np.dot(cand_ccw, starboard_dir)) else cand_ccw

    def _F_rep_one_obstacle(
        self,
        p_os: np.ndarray,
        v_os: np.ndarray,
        heading_dir: np.ndarray,
        p_ts: np.ndarray,
        v_ts: np.ndarray,
        R_os: float,
        R_ts: float,
        rho_og: float,
        n_og: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, CustomInfo, Bool]:
        """
        Compute repulsive force contribution for one obstacle (paper piecewise definition).

        Returns:
          F_rep_i, F_rd_i, F_rs_i, F_re_i, CustomInfo, collision_state (Bool)
        """
        # Geometry: p_ot = p_ts - p_os,  ρ_ot = ||p_ot||,  n_ot = p_ot / ρ_ot
        p_ot = p_ts - p_os
        rho_ot = _norm(p_ot)
        n_ot = _unit(p_ot)

        # Relative velocity: v_to = v_os - v_ts
        v_to = v_os - v_ts
        v_to_norm = _norm(v_to)

        # Safety distances:
        # d_m = R_os + d_safe + R_ts
        # CR  = d_m + rho_0
        d_m = R_os + self.d_safe + R_ts
        CR = d_m + self.rho_0

        # Collision boolean (helper-style): compare to (τ + R_os)
        tau_collision = self.tau + R_os
        collision_state = Bool(data=(rho_ot < tau_collision))

        # Angle definitions:
        # θ: angle between p_ot and v_to
        if rho_ot > 1e-9 and v_to_norm > 1e-9:
            theta = _angle_between(p_ot, v_to)
        else:
            theta = math.pi

        theta_m = self._theta_m(rho_ot, d_m)

        # Perpendicular component magnitude ||v_to,perp||
        # v_to,perp = v_to - (v_to·n_ot) n_ot   (since n_ot is unit)
        v_to_perp = v_to - float(np.dot(v_to, n_ot)) * n_ot
        v_to_perp_norm = _norm(v_to_perp)

        # Prepare CustomInfo (your pipeline/debug)
        ci = CustomInfo()
        ci.distance_to_obstacle = float(rho_ot)
        ci.center_to_center_safe_distance = float(d_m)
        ci.collision_avoidance_radius = float(CR)
        ci.vector_to_obstacle = p_ot.tolist()
        ci.relative_speed_vector = v_to.tolist()
        ci.unit_vector_to_obstacle = n_ot.tolist()
        ci.obstacle_domain_radius = float(R_ts)
        ci.distance_to_goal = float(rho_og)
        ci.CR = float(CR)
        ci.dm = float(d_m)
        ci.angle_between_direction_and_velocity = float(math.degrees(theta))
        ci.angle_for_safe_distance = float(math.degrees(theta_m))
        ci.angle_difference_for_safety = float(math.degrees(theta_m - theta))

        # Default: starboard-perpendicular for debug
        n_ot_perp_starboard = self._select_n_ot_perp_starboard(n_ot, heading_dir)
        ci.perpendicular_unit_vector_to_obstacle = n_ot_perp_starboard.tolist()

        # Outside influence radius: no action
        if not (rho_ot <= CR):
            ci.action_type = "Sem ação"
            ci.avoidance_type = "Lyu2018"
            return np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), ci, collision_state

        # If goal reached, return zero to avoid orbiting.
        if rho_og < 1e-6:
            ci.action_type = "Sem ação"
            ci.avoidance_type = "Goal reached"
            return np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), ci, collision_state

        # ----------------- Emergency Zone (EZ): ρ_ot <= d_m  -----------------
        # F_rep = F_re1 + F_re2 + F_re3  (Eq. 16–18)
        if rho_ot <= d_m:
            ci.action_type = "Obstáculo Emergência"
            ci.avoidance_type = "Lyu2018 emergency"

            # Paper says choose left/right by minimum avoidance time; here we keep your original heuristic:
            # decide side based on sign of cross(p_ot, v_to). (No math change requested.)
            if v_to_norm < 1e-9:
                n_ot_perp = _unit(_rot_ccw(n_ot))
            else:
                side = _cross2(p_ot, v_to)
                n_ot_perp = _unit(_rot_ccw(n_ot) if side > 0.0 else _rot_cw(n_ot))

            ci.perpendicular_unit_vector_to_obstacle = n_ot_perp.tolist()

            # Common sub-terms for Eq. (16–18)
            denom_rho_tau = max(rho_ot - self.tau, 1e-6)          # (d - τ)
            A = (1.0 / denom_rho_tau) - (1.0 / max(d_m, 1e-6))    # (1/(d-τ) - 1/d_m)
            B = (rho_og ** 2) / (denom_rho_tau ** 2)              # (ρ_og^2 / (d-τ)^2)

            # Eq. (16)
            F_re1 = -2.0 * self.eta_e * R_ts * A * B * n_ot

            # Eq. (17)
            F_re2 = (
                2.0 * self.eta_e * R_ts
                * (rho_og / max(rho_ot, 1e-6))
                * (v_to_norm ** 2)
                * (math.cos(theta) * math.sin(theta))
                * n_ot_perp
            )

            # Eq. (18)
            F_re3 = (
                2.0 * self.eta_e * R_ts
                * rho_og
                * (A ** 2 + (v_to_norm ** 2) * (math.cos(theta) ** 2))
                * n_og
            )

            F_re = F_re1 + F_re2 + F_re3
            return F_re, np.zeros(2), np.zeros(2), F_re, ci, collision_state

        # ----------------- Risk Region (RR): d_m < ρ_ot <= CR and θ < θ_m -----------------
        # Outside collision cone, no repulsion (paper gating).
        if not (theta < theta_m):
            ci.action_type = "Sem ação"
            ci.avoidance_type = "No collision cone"
            return np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), ci, collision_state

        # Common angle factor: exp(θ_m - θ)
        delta_theta = (theta_m - theta)
        exp_delta = math.exp(delta_theta)

        # Dynamic obstacle: ||v_ts|| > threshold and ||v_to|| > 0
        if _norm(v_ts) > self.v_ts_moving_threshold and v_to_norm > 1e-9:
            ci.action_type = "Obstáculo Dinâmico"
            ci.avoidance_type = "Lyu2018 dynamic (starboard)"

            # Eq. (9–13) use:
            denom_rho_dm = max(rho_ot - d_m, 1e-6)                 # (d - d_m)
            rho_0 = max(self.rho_0, 1e-6)
            A = (1.0 / denom_rho_dm) - (1.0 / rho_0)               # (1/(d-dm) - 1/ρ0)

            # Geometry term: d_m / ( d * sqrt(d^2 - d_m^2) )
            denom_geom = max(rho_ot * rho_ot - d_m * d_m, 1e-12)
            G = d_m / (max(rho_ot, 1e-6) * math.sqrt(denom_geom))

            # F_to0 (Eq. 10): A * ( G + sin(θ_m)/||v_to|| )
            F_to0 = A * (G + (math.sin(theta_m) / v_to_norm))

            # F_rd1 (Eq. 9)
            T_theta = (math.sin(theta) / v_to_norm)               # sin(θ)/||v_to||
            term1 = A * exp_delta * (G + T_theta)
            term2 = (exp_delta - 1.0) / (denom_rho_dm ** 2)
            F_rd1 = -self.eta_d * R_ts * (rho_og ** 2) * (term1 + term2 - F_to0) * n_ot

            # F_rd2 (Eq. 11): choose n_ot_perp to bias starboard (COLREGs)
            n_ot_perp = n_ot_perp_starboard
            ci.perpendicular_unit_vector_to_obstacle = n_ot_perp.tolist()

            # F_top (Eq. 12): A * ( 1/d + cos(θ_m)/||v_to|| )
            F_top = A * ((1.0 / max(rho_ot, 1e-6)) + (math.cos(theta_m) / v_to_norm))

            term3 = A * exp_delta * ((1.0 / max(rho_ot, 1e-6)) + (math.cos(theta) / v_to_norm))
            # Paper uses ||v_to,perp|| * (e^{..}-1) / ( d (d-dm)^2 )
            term4 = v_to_perp_norm * (exp_delta - 1.0) / (max(rho_ot, 1e-6) * (denom_rho_dm ** 2))
            F_rd2 = self.eta_d * R_ts * (rho_og ** 2) * (term3 + term4 - F_top) * n_ot_perp

            # F_rd3 (Eq. 13)
            F_rd3 = self.eta_d * R_ts * rho_og * (A ** 2) * (exp_delta - 1.0) * n_og

            F_rd = F_rd1 + F_rd2 + F_rd3
            return F_rd, F_rd, np.zeros(2), np.zeros(2), ci, collision_state

        # ----------------- Static obstacle (RR): v_ts = 0 -----------------
        ci.action_type = "Obstáculo Estático"
        ci.avoidance_type = "Lyu2018 static"

        # Eq. (14–15) use (d - τ) and ρ_0
        denom_rho_tau = max(rho_ot - self.tau, 1e-6)              # (d - τ)
        rho_0 = max(self.rho_0, 1e-6)
        A = (1.0 / denom_rho_tau) - (1.0 / rho_0)                # (1/(d-τ) - 1/ρ0)
        B = (rho_og ** 2) / (max(rho_ot, 1e-6) ** 2)              # (ρ_og^2 / d^2)

        # Eq. (14)
        F_rs1 = -self.eta_s * R_ts * A * B * n_ot
        # Eq. (15)
        F_rs3 = self.eta_s * R_ts * rho_og * (A ** 2) * n_og

        F_rs = F_rs1 + F_rs3
        return F_rs, np.zeros(2), F_rs, np.zeros(2), ci, collision_state

    # ----------------- timer loop -----------------
    def _on_timer(self, _evt):
        if self.robot_state is None:
            rospy.logwarn_throttle(2.0, "No robot state yet. Skipping APFM.")
            return
        if self.obstacles is None:
            rospy.logwarn_throttle(2.0, "No obstacles yet. Skipping APFM.")
            return

        # OS state
        p_os = np.array([self.robot_state.position.x, self.robot_state.position.y], dtype=float)
        v_os = np.array([self.robot_state.velocity.x, self.robot_state.velocity.y], dtype=float)

        # OS heading direction (needed for starboard selection of n_ot_perp)
        v_os_speed = _norm(v_os)
        if v_os_speed > 1e-6:
            heading_dir = v_os / v_os_speed
        else:
            yaw = _quat_to_yaw(self.robot_state.orientation)
            heading_dir = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)

        # R_os (prefer message radius; fallback)
        R_os = float(self.robot_state.radius) if float(self.robot_state.radius) > 1e-6 else float(self.R_os_param)

        # Attractive force + distance to goal
        F_att, rho_og, n_og = self._F_att(p_os, self.p_g)

        # Publish distance_to_goal (aux module behavior)
        self.distance_to_goal_pub.publish(Float64(data=float(rho_og)))

        # Accumulators
        F_rep = np.zeros(2, dtype=float)
        F_rd_sum = np.zeros(2, dtype=float)
        F_rs_sum = np.zeros(2, dtype=float)
        F_re_sum = np.zeros(2, dtype=float)

        # Per obstacle processing
        for ob in self.obstacles:
            p_ts = np.array([ob.position.x, ob.position.y], dtype=float)
            v_ts = np.array([ob.velocity.x, ob.velocity.y], dtype=float)
            R_ts = float(ob.radius)

            F_rep_i, F_rd_i, F_rs_i, F_re_i, ci, collision_state = self._F_rep_one_obstacle(
                p_os=p_os,
                v_os=v_os,
                heading_dir=heading_dir,
                p_ts=p_ts,
                v_ts=v_ts,
                R_os=R_os,
                R_ts=R_ts,
                rho_og=rho_og,
                n_og=n_og,
            )

            # Optional: append obstacle name if exists (non-standard field)
            name = getattr(ob, "name", "")
            if name:
                ci.avoidance_type = f"{ci.avoidance_type} | name={name}"

            # Publish collision state (one per obstacle per cycle, like your helper)
            self.collision_pub.publish(collision_state)

            # Publish CustomInfo only if within CR
            if ci.distance_to_obstacle <= ci.collision_avoidance_radius:
                self.custom_info_pub.publish(ci)

            # Sum forces
            F_rep += F_rep_i
            F_rd_sum += F_rd_i
            F_rs_sum += F_rs_i
            F_re_sum += F_re_i

        # Total force (paper): F_total = F_att + F_rep
        F_total = F_att + F_rep

        # Publish forces (same topics as original)
        self._pub_vec(self.total_force_pub, F_total)
        self._pub_vec(self.attractive_force_pub, F_att)
        self._pub_vec(self.repulsive_force_pub, F_rep)
        self._pub_vec(self.dynamic_force_pub, F_rd_sum)
        self._pub_vec(self.static_force_pub, F_rs_sum)
        self._pub_vec(self.emergency_force_pub, F_re_sum)

    def _pub_vec(self, pub: rospy.Publisher, f: np.ndarray):
        pub.publish(Vector3(x=float(f[0]), y=float(f[1]), z=0.0))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    APFMAvoidanceLyu2018FullIO().run()
