#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Velocity-Obstacle (VO) local planner publishing v_ref and w_ref.

Purpose
-------
This node is a practical replacement for an APF *force* generator + force->(v,w)
converter, but using VO concepts directly.

Inputs
------
- /scenario/output_robot      (dynamic_obstacle_avoidance/RobotState)
- /scenario/output_obstacles  (dynamic_obstacle_avoidance/ObstacleArray)
- /scenario/goal              (geometry_msgs/Vector3)  # uses x,y

Outputs
-------
- /migbot/v_ref  (std_msgs/Float64)
- /migbot/w_ref  (std_msgs/Float64)

VO check (time horizon)
----------------------
For each obstacle with position p_o and velocity v_o, and ownship position p_r,
we test candidate ownship velocities v against the condition:

  exists t in [0,T] such that || (p_o - p_r) + (v - v_o) * t || <= R

where R = r_robot + r_obstacle + clearance.

Instead of explicitly building the VO cone, we do the equivalent membership test
using Closest Point of Approach (CPA) over the time horizon:

  t* = argmin_t || p_rel + v_rel t || = -(p_rel·v_rel)/||v_rel||^2
  t_cpa = clamp(t*, 0, T)
  d_cpa = || p_rel + v_rel t_cpa ||

Candidate is invalid if d_cpa <= R.

Search
------
We sample candidate headings (full circle, centered on goal heading) using a
fixed speed (assumed_speed). Among valid candidates, pick the one maximizing:

  score = w_goal * cos(|Δgoal|) + w_clear * min_cpa - w_turn * |Δturn|

Then convert chosen heading to (v_ref, w_ref) with a simple heading controller.

Notes
-----
- This is a *simplified* VO planner: fixed speed, heading sampling, no explicit
  acceleration constraints (your low-level controller will smooth it).
- For more "pure" VO, you can also sample (speed, heading) pairs.
"""

import math
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray, ObstacleState


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def norm2(vx: float, vy: float) -> float:
    return math.hypot(vx, vy)


def cpa_distance_and_time(
    p_rel: Tuple[float, float],
    v_rel: Tuple[float, float],
    horizon: float,
) -> Tuple[float, float]:
    """Closest Point of Approach in [0, horizon]. Returns (d_cpa, t_cpa)."""
    px, py = p_rel
    vx, vy = v_rel
    vv = vx * vx + vy * vy
    if vv < 1e-9:
        return math.hypot(px, py), 0.0

    t_star = -(px * vx + py * vy) / vv
    t_cpa = max(0.0, min(float(horizon), float(t_star)))
    cx = px + vx * t_cpa
    cy = py + vy * t_cpa
    return math.hypot(cx, cy), t_cpa


class VOToVWRefNode:
    def __init__(self):
        # --- Topics ---
        self.robot_state_topic = rospy.get_param('~robot_state_topic', '/scenario/output_robot')
        self.obstacles_topic = rospy.get_param('~obstacles_topic', '/scenario/output_obstacles')
        self.goal_topic = rospy.get_param('~goal_topic', '/scenario/goal')

        self.v_ref_topic = rospy.get_param('~v_ref_topic', '/migbot/v_ref')
        self.w_ref_topic = rospy.get_param('~w_ref_topic', '/migbot/w_ref')

        # --- VO / sampling ---
        self.assumed_speed = float(rospy.get_param('~assumed_speed', 2.0))
        self.time_horizon = float(rospy.get_param('~time_horizon', 10.0))
        self.clearance = float(rospy.get_param('~clearance', 0.5))
        self.angle_step_deg = float(rospy.get_param('~angle_step_deg', 5.0))

        self.ignore_far = bool(rospy.get_param('~ignore_far_obstacles', True))
        self.max_considered_dist = float(rospy.get_param('~max_considered_dist', 80.0))

        # Scoring weights
        self.w_goal = float(rospy.get_param('~w_goal', 10.0))
        self.w_clear = float(rospy.get_param('~w_clear', 1.0))
        self.w_turn = float(rospy.get_param('~w_turn', 0.5))

        # --- Output limits / behavior ---
        self.v_max = float(rospy.get_param('~v_max', 2.0))
        self.w_max = float(rospy.get_param('~w_max', (2.0 * math.pi) / 6.0))  # 360deg/6s
        self.v_cruise = float(rospy.get_param('~v_cruise', min(self.v_max, self.assumed_speed)))

        self.stop_tolerance = float(rospy.get_param('~stop_tolerance', 5.0))

        # Heading control
        default_k = self.w_max / (math.pi / 2.0)  # w_max at 90deg
        self.k_heading = float(rospy.get_param('~k_heading', default_k))
        self.heading_deadband = math.radians(float(rospy.get_param('~heading_deadband_deg', 2.0)))

        # Alignment speed scaling
        self.use_align_scale = bool(rospy.get_param('~use_alignment_speed_scaling', True))
        self.align_power = int(rospy.get_param('~alignment_power', 4))
        self.min_align_scale = float(rospy.get_param('~min_alignment_scale', 0.0))

        # Fallback behavior when no valid candidate
        self.fallback_speed_scale = float(rospy.get_param('~fallback_speed_scale', 0.5))

        # Loop
        self.rate_hz = float(rospy.get_param('~rate_hz', 10.0))

        # --- State ---
        self.robot: Optional[RobotState] = None
        self.obstacles: List[ObstacleState] = []
        self.goal_xy: Optional[Tuple[float, float]] = None

        # --- Pub/Sub ---
        self.pub_v = rospy.Publisher(self.v_ref_topic, Float64, queue_size=10)
        self.pub_w = rospy.Publisher(self.w_ref_topic, Float64, queue_size=10)

        # Debug
        self.pub_best_heading = rospy.Publisher('~debug/best_heading', Float64, queue_size=10)
        self.pub_min_cpa = rospy.Publisher('~debug/min_cpa_distance', Float64, queue_size=10)
        self.pub_found_valid = rospy.Publisher('~debug/found_valid', Float64, queue_size=10)
        self.pub_heading_err = rospy.Publisher('~debug/heading_error', Float64, queue_size=10)
        self.pub_yaw = rospy.Publisher('~debug/yaw', Float64, queue_size=10)
        self.pub_dist_goal = rospy.Publisher('~debug/distance_to_goal', Float64, queue_size=10)

        rospy.Subscriber(self.robot_state_topic, RobotState, self._cb_robot, queue_size=10)
        rospy.Subscriber(self.obstacles_topic, ObstacleArray, self._cb_obstacles, queue_size=10)
        rospy.Subscriber(self.goal_topic, Vector3, self._cb_goal, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1e-3)), self._on_timer)

        rospy.loginfo('[VOToVWRef] Sub: robot=%s obstacles=%s goal=%s | Pub: v_ref=%s w_ref=%s',
                      self.robot_state_topic, self.obstacles_topic, self.goal_topic,
                      self.v_ref_topic, self.w_ref_topic)
        rospy.loginfo('[VOToVWRef] Params: v_max=%.2f, w_max=%.3f, v_cruise=%.2f, assumed_speed=%.2f, horizon=%.1f, clearance=%.2f, step=%.1f deg',
                      self.v_max, self.w_max, self.v_cruise, self.assumed_speed,
                      self.time_horizon, self.clearance, self.angle_step_deg)

    def _cb_robot(self, msg: RobotState):
        self.robot = msg

    def _cb_obstacles(self, msg: ObstacleArray):
        self.obstacles = list(msg.obstacles)

    def _cb_goal(self, msg: Vector3):
        self.goal_xy = (float(msg.x), float(msg.y))

    def _robot_yaw(self) -> float:
        assert self.robot is not None
        q = self.robot.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return wrap_pi(yaw)

    def _robot_heading_ref(self) -> float:
        """Use velocity direction if moving; otherwise use yaw."""
        assert self.robot is not None
        vx = float(self.robot.velocity.x)
        vy = float(self.robot.velocity.y)
        if norm2(vx, vy) > 0.2:
            return wrap_pi(math.atan2(vy, vx))
        return self._robot_yaw()

    def _on_timer(self, _evt):
        if self.robot is None or self.goal_xy is None:
            return

        rx = float(self.robot.position.x)
        ry = float(self.robot.position.y)
        gx, gy = self.goal_xy

        dx = gx - rx
        dy = gy - ry
        dist_goal = math.hypot(dx, dy)
        self.pub_dist_goal.publish(Float64(data=float(dist_goal)))

        # Stop near goal
        if dist_goal <= self.stop_tolerance:
            self.pub_v.publish(Float64(data=0.0))
            self.pub_w.publish(Float64(data=0.0))
            self.pub_best_heading.publish(Float64(data=0.0))
            self.pub_min_cpa.publish(Float64(data=0.0))
            self.pub_found_valid.publish(Float64(data=1.0))
            self.pub_heading_err.publish(Float64(data=0.0))
            self.pub_yaw.publish(Float64(data=float(self._robot_yaw())))
            return

        goal_heading = wrap_pi(math.atan2(dy, dx))
        heading_ref = self._robot_heading_ref()

        # Candidate headings
        step = max(1e-3, math.radians(self.angle_step_deg))
        n_steps = max(1, int(round((2.0 * math.pi) / step)))
        headings = [wrap_pi(goal_heading - math.pi + i * (2.0 * math.pi / n_steps)) for i in range(n_steps)]

        best_heading, best_min_cpa, found_valid = self._select_best_heading(
            headings=headings,
            goal_heading=goal_heading,
            heading_ref=heading_ref,
            rx=rx,
            ry=ry,
        )

        # Convert heading -> (v_ref, w_ref)
        yaw = self._robot_yaw()
        heading_error = wrap_pi(best_heading - yaw)
        if abs(heading_error) < self.heading_deadband:
            heading_error = 0.0

        w_ref = self.k_heading * heading_error
        w_ref = max(-self.w_max, min(self.w_max, w_ref))

        v_ref = float(self.v_cruise)

        if self.use_align_scale:
            align = max(0.0, math.cos(heading_error)) ** max(self.align_power, 1)
            align = max(self.min_align_scale, min(1.0, align))
            v_ref *= align

        # If no valid heading exists, be more conservative with speed
        if not found_valid:
            v_ref *= max(0.0, min(1.0, self.fallback_speed_scale))

        v_ref = max(0.0, min(self.v_max, v_ref))

        self.pub_v.publish(Float64(data=float(v_ref)))
        self.pub_w.publish(Float64(data=float(w_ref)))

        # Debug
        self.pub_best_heading.publish(Float64(data=float(best_heading)))
        self.pub_min_cpa.publish(Float64(data=float(best_min_cpa)))
        self.pub_found_valid.publish(Float64(data=1.0 if found_valid else 0.0))
        self.pub_heading_err.publish(Float64(data=float(heading_error)))
        self.pub_yaw.publish(Float64(data=float(yaw)))

    def _select_best_heading(
        self,
        headings: List[float],
        goal_heading: float,
        heading_ref: float,
        rx: float,
        ry: float,
    ) -> Tuple[float, float, bool]:
        assert self.robot is not None

        obstacles = self.obstacles
        if self.ignore_far and obstacles:
            obstacles = [
                ob for ob in obstacles
                if math.hypot(float(ob.position.x) - rx, float(ob.position.y) - ry) <= self.max_considered_dist
            ]

        robot_r = float(getattr(self.robot, 'radius', 0.0))
        if robot_r < 0.0:
            robot_r = 0.0

        best_valid: Optional[Tuple[float, float, float]] = None  # (score, heading, min_cpa)
        best_any: Optional[Tuple[float, float, float]] = None

        for hdg in headings:
            v_cand = (self.assumed_speed * math.cos(hdg), self.assumed_speed * math.sin(hdg))

            min_cpa = float('inf')
            is_valid = True

            for ob in obstacles:
                ox = float(ob.position.x)
                oy = float(ob.position.y)
                ovx = float(ob.velocity.x)
                ovy = float(ob.velocity.y)
                ob_r = float(ob.radius)

                R = robot_r + ob_r + self.clearance

                px = ox - rx
                py = oy - ry
                dist_now = math.hypot(px, py)

                if dist_now <= R:
                    min_cpa = min(min_cpa, dist_now)
                    is_valid = False
                    continue

                if self.ignore_far:
                    if dist_now - R > (self.assumed_speed + norm2(ovx, ovy)) * self.time_horizon + 5.0:
                        min_cpa = min(min_cpa, dist_now)
                        continue

                v_rel = (v_cand[0] - ovx, v_cand[1] - ovy)
                d_cpa, _t = cpa_distance_and_time((px, py), v_rel, self.time_horizon)
                min_cpa = min(min_cpa, d_cpa)
                if d_cpa <= R:
                    is_valid = False

            ang_err = abs(wrap_pi(hdg - goal_heading))
            progress = math.cos(ang_err)
            turn = abs(wrap_pi(hdg - heading_ref))
            clearance_reward = min(min_cpa, 100.0)

            score = self.w_goal * progress + self.w_clear * clearance_reward - self.w_turn * turn

            if best_any is None or score > best_any[0]:
                best_any = (score, hdg, min_cpa)

            if is_valid:
                if best_valid is None or score > best_valid[0]:
                    best_valid = (score, hdg, min_cpa)

        if best_valid is not None:
            _s, hdg, min_cpa = best_valid
            return hdg, min_cpa, True

        assert best_any is not None
        _s, hdg, min_cpa = best_any
        return hdg, min_cpa, False


def main():
    rospy.init_node('vo_to_vw_ref', anonymous=False)
    VOToVWRefNode()
    rospy.spin()


if __name__ == '__main__':
    main()
