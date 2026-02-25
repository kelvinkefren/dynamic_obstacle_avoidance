#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dynamic Window Approach (DWA) local planner publishing v_ref and w_ref.

This node replaces an APF force generator (or VO planner) by directly producing
(v_ref, w_ref) commands using the Dynamic Window Approach.

Inputs
------
- /scenario/output_robot      (dynamic_obstacle_avoidance/RobotState)
- /scenario/output_obstacles  (dynamic_obstacle_avoidance/ObstacleArray)
- /scenario/goal              (geometry_msgs/Vector3)  # uses x,y

Outputs
-------
- /migbot/v_ref  (std_msgs/Float64)
- /migbot/w_ref  (std_msgs/Float64)

DWA summary
-----------
1) Compute the *dynamic window* around current (v,w) based on acceleration limits.
2) Sample candidate (v,w) pairs in that window.
3) Forward-simulate each candidate for a short horizon, predicting obstacle motion
   with constant-velocity model.
4) Score each trajectory by goal progress/heading, clearance, and speed.
5) Publish the best (v_ref,w_ref). Stop when within stop_tolerance.

This implementation is intentionally lightweight and ROS1-friendly.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray, ObstacleState


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def hypot2(x: float, y: float) -> float:
    return math.hypot(x, y)


@dataclass
class TrajEval:
    v: float
    w: float
    score: float
    min_sep: float
    dist_final: float
    heading_score: float


class DWAToVWRefNode:
    def __init__(self):
        # --- Topics ---
        self.robot_state_topic = rospy.get_param('~robot_state_topic', '/scenario/output_robot')
        self.obstacles_topic = rospy.get_param('~obstacles_topic', '/scenario/output_obstacles')
        self.goal_topic = rospy.get_param('~goal_topic', '/scenario/goal')

        self.v_ref_topic = rospy.get_param('~v_ref_topic', '/migbot/v_ref')
        self.w_ref_topic = rospy.get_param('~w_ref_topic', '/migbot/w_ref')

        # --- Limits (match your requirement) ---
        self.v_max = float(rospy.get_param('~v_max', 2.0))
        self.v_min = float(rospy.get_param('~v_min', 0.0))  # boats typically don't go backward
        self.w_max = float(rospy.get_param('~w_max', (2.0 * math.pi) / 6.0))  # 360deg/6s

        # --- Acceleration limits (tune to your platform) ---
        # These limits define the dynamic window width.
        self.a_v_max = float(rospy.get_param('~a_v_max', 0.6))      # m/s^2
        self.a_w_max = float(rospy.get_param('~a_w_max', 1.2))      # rad/s^2

        # --- Sampling ---
        self.v_step = float(rospy.get_param('~v_step', 0.2))        # m/s
        self.w_step = float(rospy.get_param('~w_step', 0.1))        # rad/s

        # --- Simulation ---
        self.predict_time = float(rospy.get_param('~predict_time', 3.0))  # seconds
        self.sim_dt = float(rospy.get_param('~sim_dt', 0.1))              # seconds

        # --- Safety / obstacle handling ---
        self.clearance = float(rospy.get_param('~clearance', 0.5))
        self.ignore_far = bool(rospy.get_param('~ignore_far_obstacles', True))
        self.max_considered_dist = float(rospy.get_param('~max_considered_dist', 80.0))

        # --- Goal behavior ---
        self.stop_tolerance = float(rospy.get_param('~stop_tolerance', 5.0))

        # --- Scoring weights ---
        # Increase w_clear if you want more conservative deviations.
        self.w_heading = float(rospy.get_param('~w_heading', 2.0))
        self.w_goal = float(rospy.get_param('~w_goal', 6.0))
        self.w_clear = float(rospy.get_param('~w_clear', 1.5))
        self.w_vel = float(rospy.get_param('~w_vel', 0.5))
        self.w_turn = float(rospy.get_param('~w_turn', 0.1))  # penalize large yaw rates

        # Optional: penalize getting too close even if not colliding
        self.clear_soft_cap = float(rospy.get_param('~clear_soft_cap', 30.0))

        # Loop
        self.rate_hz = float(rospy.get_param('~rate_hz', 10.0))
        self.max_dt_window = float(rospy.get_param('~max_dt_window', 0.2))  # cap for stability

        # --- State ---
        self.robot: Optional[RobotState] = None
        self.obstacles: List[ObstacleState] = []
        self.goal_xy: Optional[Tuple[float, float]] = None

        self._prev_yaw: Optional[float] = None
        self._prev_time: Optional[rospy.Time] = None
        self._w_est: float = 0.0

        # --- Pub/Sub ---
        self.pub_v = rospy.Publisher(self.v_ref_topic, Float64, queue_size=10)
        self.pub_w = rospy.Publisher(self.w_ref_topic, Float64, queue_size=10)

        # Debug
        self.pub_best_v = rospy.Publisher('~debug/best_v', Float64, queue_size=10)
        self.pub_best_w = rospy.Publisher('~debug/best_w', Float64, queue_size=10)
        self.pub_best_score = rospy.Publisher('~debug/best_score', Float64, queue_size=10)
        self.pub_best_min_sep = rospy.Publisher('~debug/best_min_sep', Float64, queue_size=10)
        self.pub_dist_goal = rospy.Publisher('~debug/distance_to_goal', Float64, queue_size=10)
        self.pub_w_est = rospy.Publisher('~debug/w_est', Float64, queue_size=10)

        rospy.Subscriber(self.robot_state_topic, RobotState, self._cb_robot, queue_size=10)
        rospy.Subscriber(self.obstacles_topic, ObstacleArray, self._cb_obstacles, queue_size=10)
        rospy.Subscriber(self.goal_topic, Vector3, self._cb_goal, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1e-3)), self._on_timer)

        rospy.loginfo('[DWAToVWRef] Sub: robot=%s obstacles=%s goal=%s | Pub: v_ref=%s w_ref=%s',
                      self.robot_state_topic, self.obstacles_topic, self.goal_topic,
                      self.v_ref_topic, self.w_ref_topic)
        rospy.loginfo('[DWAToVWRef] Limits: v=[%.2f, %.2f], w=±%.3f rad/s | Acc: a_v=%.2f, a_w=%.2f',
                      self.v_min, self.v_max, self.w_max, self.a_v_max, self.a_w_max)
        rospy.loginfo('[DWAToVWRef] Sim: predict_time=%.2f s, sim_dt=%.2f s | Sampling: v_step=%.2f, w_step=%.2f',
                      self.predict_time, self.sim_dt, self.v_step, self.w_step)

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
        return wrap_pi(float(yaw))

    def _robot_speed(self) -> float:
        assert self.robot is not None
        vx = float(self.robot.velocity.x)
        vy = float(self.robot.velocity.y)
        return hypot2(vx, vy)

    def _estimate_w(self, yaw: float, now: rospy.Time):
        # Estimate yaw rate from successive yaw readings.
        if self._prev_time is None or self._prev_yaw is None:
            self._prev_time = now
            self._prev_yaw = yaw
            self._w_est = 0.0
            return

        dt = (now - self._prev_time).to_sec()
        if dt <= 1e-3:
            return

        dyaw = wrap_pi(yaw - self._prev_yaw)
        self._w_est = dyaw / dt

        self._prev_time = now
        self._prev_yaw = yaw

    def _dynamic_window(self, v_cur: float, w_cur: float, dt: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        dt = max(0.0, min(float(dt), float(self.max_dt_window)))

        v_lo = v_cur - self.a_v_max * dt
        v_hi = v_cur + self.a_v_max * dt
        w_lo = w_cur - self.a_w_max * dt
        w_hi = w_cur + self.a_w_max * dt

        v_lo = clamp(v_lo, self.v_min, self.v_max)
        v_hi = clamp(v_hi, self.v_min, self.v_max)
        w_lo = clamp(w_lo, -self.w_max, self.w_max)
        w_hi = clamp(w_hi, -self.w_max, self.w_max)

        if v_hi < v_lo:
            v_lo, v_hi = v_hi, v_lo
        if w_hi < w_lo:
            w_lo, w_hi = w_hi, w_lo

        return (v_lo, v_hi), (w_lo, w_hi)

    def _sample_range(self, lo: float, hi: float, step: float) -> List[float]:
        step = max(1e-6, float(step))
        if hi - lo < 1e-9:
            return [float(lo)]
        vals = []
        x = lo
        # ensure inclusion of hi
        while x <= hi + 1e-9:
            vals.append(float(x))
            x += step
        # if due to step we missed hi (numerically), append hi
        if abs(vals[-1] - hi) > 1e-6:
            vals.append(float(hi))
        return vals

    def _predict_min_separation(self, rx: float, ry: float, yaw: float, v: float, w: float,
                                obstacles: List[ObstacleState], robot_r: float) -> Tuple[float, float, float, float]:
        """Simulate candidate for predict_time and compute:
        - min_sep: minimum separation margin (distance - (R_total)) along trajectory
        - x,y,yaw at end
        - dist_final_to_goal computed outside

        Obstacle prediction: constant velocity.
        """
        x = float(rx)
        y = float(ry)
        th = float(yaw)

        min_sep = float('inf')

        t = 0.0
        while t <= self.predict_time + 1e-9:
            # check separation at this timestep
            for ob in obstacles:
                ox0 = float(ob.position.x)
                oy0 = float(ob.position.y)
                ovx = float(ob.velocity.x)
                ovy = float(ob.velocity.y)
                ob_r = float(ob.radius)

                ox = ox0 + ovx * t
                oy = oy0 + ovy * t

                R = robot_r + ob_r + self.clearance
                d = math.hypot(ox - x, oy - y)
                sep = d - R
                if sep < min_sep:
                    min_sep = sep

            # propagate robot
            x += v * math.cos(th) * self.sim_dt
            y += v * math.sin(th) * self.sim_dt
            th = wrap_pi(th + w * self.sim_dt)
            t += self.sim_dt

        if min_sep == float('inf'):
            min_sep = 999.0

        return min_sep, x, y, th

    def _evaluate_candidates(self, rx: float, ry: float, yaw: float, gx: float, gy: float,
                             v_window: Tuple[float, float], w_window: Tuple[float, float],
                             obstacles: List[ObstacleState], robot_r: float) -> Optional[TrajEval]:

        v_lo, v_hi = v_window
        w_lo, w_hi = w_window

        v_samples = self._sample_range(v_lo, v_hi, self.v_step)
        w_samples = self._sample_range(w_lo, w_hi, self.w_step)

        best: Optional[TrajEval] = None

        # precompute goal heading from current pose
        goal_heading_now = wrap_pi(math.atan2(gy - ry, gx - rx))

        for v in v_samples:
            for w in w_samples:
                # quick reject: extremely small v and w (can stall)
                # (keep it allowed, but it will score poorly)

                min_sep, x_f, y_f, th_f = self._predict_min_separation(rx, ry, yaw, v, w, obstacles, robot_r)

                # Hard collision check
                if min_sep < 0.0:
                    continue

                dist_final = math.hypot(gx - x_f, gy - y_f)
                goal_heading_final = wrap_pi(math.atan2(gy - y_f, gx - x_f))

                # Heading: how aligned the final heading is towards the goal direction
                heading_err = abs(wrap_pi(goal_heading_final - th_f))
                heading_score = math.cos(heading_err)  # [-1,1]

                # Goal progress: prefer smaller final distance
                goal_score = 1.0 / (1.0 + dist_final)

                # Clearance reward (soft-capped to avoid dominating everything)
                clear_reward = clamp(min_sep, 0.0, self.clear_soft_cap)

                # Speed preference
                vel_score = (v / self.v_max) if self.v_max > 1e-6 else 0.0

                # Turn penalty
                turn_pen = abs(w) / max(self.w_max, 1e-6)

                score = (
                    self.w_heading * heading_score +
                    self.w_goal * goal_score +
                    self.w_clear * clear_reward +
                    self.w_vel * vel_score -
                    self.w_turn * turn_pen
                )

                # Extra mild bias: keep close to goal heading now (helps in open water)
                # (does not override obstacle safety)
                bias = math.cos(abs(wrap_pi(goal_heading_now - th_f)))
                score += 0.2 * bias

                if best is None or score > best.score:
                    best = TrajEval(v=float(v), w=float(w), score=float(score),
                                    min_sep=float(min_sep), dist_final=float(dist_final),
                                    heading_score=float(heading_score))

        return best

    def _on_timer(self, _evt):
        if self.robot is None or self.goal_xy is None:
            return

        now = rospy.Time.now()

        rx = float(self.robot.position.x)
        ry = float(self.robot.position.y)
        gx, gy = self.goal_xy

        dist_goal = math.hypot(gx - rx, gy - ry)
        self.pub_dist_goal.publish(Float64(data=float(dist_goal)))

        # Stop near goal
        if dist_goal <= self.stop_tolerance:
            self.pub_v.publish(Float64(data=0.0))
            self.pub_w.publish(Float64(data=0.0))
            self.pub_best_v.publish(Float64(data=0.0))
            self.pub_best_w.publish(Float64(data=0.0))
            self.pub_best_score.publish(Float64(data=0.0))
            self.pub_best_min_sep.publish(Float64(data=0.0))
            return

        yaw = self._robot_yaw()
        self._estimate_w(yaw, now)
        self.pub_w_est.publish(Float64(data=float(self._w_est)))

        v_cur = self._robot_speed()
        w_cur = float(self._w_est)

        # dt for dynamic window
        if self._prev_time is None:
            dt = 1.0 / max(self.rate_hz, 1e-3)
        else:
            dt = max(1e-3, (now - self._prev_time).to_sec())

        v_win, w_win = self._dynamic_window(v_cur, w_cur, dt)

        obstacles = self.obstacles
        if self.ignore_far and obstacles:
            obstacles = [
                ob for ob in obstacles
                if math.hypot(float(ob.position.x) - rx, float(ob.position.y) - ry) <= self.max_considered_dist
            ]

        robot_r = float(getattr(self.robot, 'radius', 0.0))
        if robot_r < 0.0:
            robot_r = 0.0

        best = self._evaluate_candidates(rx, ry, yaw, gx, gy, v_win, w_win, obstacles, robot_r)

        if best is None:
            # Fallback: rotate towards goal slowly, no forward motion
            goal_heading = wrap_pi(math.atan2(gy - ry, gx - rx))
            err = wrap_pi(goal_heading - yaw)
            w_cmd = clamp(1.5 * err, -self.w_max, self.w_max)
            v_cmd = 0.0

            self.pub_v.publish(Float64(data=float(v_cmd)))
            self.pub_w.publish(Float64(data=float(w_cmd)))

            self.pub_best_v.publish(Float64(data=float(v_cmd)))
            self.pub_best_w.publish(Float64(data=float(w_cmd)))
            self.pub_best_score.publish(Float64(data=-1.0))
            self.pub_best_min_sep.publish(Float64(data=0.0))
            return

        # Commands are the selected (v,w) inside the dynamic window
        v_cmd = clamp(best.v, self.v_min, self.v_max)
        w_cmd = clamp(best.w, -self.w_max, self.w_max)

        self.pub_v.publish(Float64(data=float(v_cmd)))
        self.pub_w.publish(Float64(data=float(w_cmd)))

        self.pub_best_v.publish(Float64(data=float(best.v)))
        self.pub_best_w.publish(Float64(data=float(best.w)))
        self.pub_best_score.publish(Float64(data=float(best.score)))
        self.pub_best_min_sep.publish(Float64(data=float(best.min_sep)))


def main():
    rospy.init_node('dwa_to_vw_ref', anonymous=False)
    DWAToVWRefNode()
    rospy.spin()


if __name__ == '__main__':
    main()
