#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point, Vector3, Quaternion
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray
from tf.transformations import quaternion_from_euler

class ScenarioController:
    def __init__(self):
        rospy.init_node('scenario_controller')

        # Parameters
        self.scenario = rospy.get_param('~scenario', 0)  # Default to scenario 0
        self.goal_x = rospy.get_param('~goal_x', 30.0)    # Default goal_x
        self.goal_y = rospy.get_param('~goal_y', 30.0)    # Default goal_y
        self.robot_publish_rate = rospy.get_param('~robot_publish_rate', 30.0)  # Default 30 Hz
        self.obstacle_publish_rate = rospy.get_param('~obstacle_publish_rate', 30.0)  # Default 30 Hz
        self.goal_publish_rate = rospy.get_param('~goal_publish_rate', 1.0)  # Default 1 Hz

        # 30-degree orientation (converted to radians)
        self.orientation_angle = math.radians(30)

        # Convert the 30-degree orientation to a quaternion
        self.robot_orientation = Quaternion(*quaternion_from_euler(0, 0, self.orientation_angle))

        # Publishers
        self.robot_pub = rospy.Publisher('/scenario/output_robot', RobotState, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/scenario/output_obstacles', ObstacleArray, queue_size=10)
        self.goal_pub = rospy.Publisher('/scenario/goal', Vector3, queue_size=10)

        # Initialize empty states for bypass mode
        self.current_robot_state = None
        self.current_obstacle_state = None

        # Set up timers to regularly publish the states and goal
        rospy.Timer(rospy.Duration(1.0 / self.robot_publish_rate), self.publish_robot_state)
        rospy.Timer(rospy.Duration(1.0 / self.obstacle_publish_rate), self.publish_obstacle_state)
        rospy.Timer(rospy.Duration(1.0 / self.goal_publish_rate), self.publish_goal)

        # Subscribers for scenario 0 (bypass mode)
        if self.scenario == 0:
            self.robot_sub = rospy.Subscriber('/scenario/input_robot', RobotState, self.robot_callback)
            self.obstacle_sub = rospy.Subscriber('/scenario/input_obstacles', ObstacleArray, self.obstacle_callback)

    def publish_goal(self, event):
        goal = Vector3(x=self.goal_x, y=self.goal_y, z=0.0)
        self.goal_pub.publish(goal)

    def publish_robot_state(self, event):
        if self.scenario == 0 and self.current_robot_state is not None:
            self.robot_pub.publish(self.current_robot_state)
        elif self.scenario == 1:
            predefined_robot = RobotState(
                position=Point(x=0, y=0, z=0),
                velocity=Vector3(x=0.1, y=0.1, z=0),
                orientation=self.robot_orientation,
                radius=1.0
            )
            self.robot_pub.publish(predefined_robot)
        elif self.scenario == 2:
            predefined_robot = RobotState(
                position=Point(x=0, y=0, z=0),
                velocity=Vector3(x=0.1, y=0.1, z=0),
                orientation=self.robot_orientation,
                radius=1.5
            )
            self.robot_pub.publish(predefined_robot)

    def publish_obstacle_state(self, event):
        if self.scenario == 0 and self.current_obstacle_state is not None:
            self.obstacle_pub.publish(self.current_obstacle_state)
        elif self.scenario == 1:
            predefined_obstacles = ObstacleArray(obstacles=[
                ObstacleState(position=Point(x=4, y=5, z=0), velocity=Vector3(x=0, y=0, z=0), radius=2.0),
                ObstacleState(position=Point(x=4, y=3, z=0), velocity=Vector3(x=-1, y=-1, z=0), radius=1.5),
                ObstacleState(position=Point(x=1.1, y=0.9, z=0), velocity=Vector3(x=0, y=0, z=0), radius=1.5)
            ])
            self.obstacle_pub.publish(predefined_obstacles)
        elif self.scenario == 2:
            predefined_obstacles = ObstacleArray(obstacles=[
                # Add obstacles for scenario 2 if needed
            ])
            self.obstacle_pub.publish(predefined_obstacles)

    def robot_callback(self, data):
        # Bypass mode: store incoming robot state
        if self.scenario == 0:
            self.current_robot_state = data

    def obstacle_callback(self, data):
        # Bypass mode: store incoming obstacle state
        if self.scenario == 0:
            self.current_obstacle_state = data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ScenarioController()
        node.run()
    except rospy.ROSInterruptException:
        pass
