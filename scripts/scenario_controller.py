#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Vector3
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray

class ScenarioController:
    def __init__(self):
        rospy.init_node('scenario_controller')

        # Parameters
        self.scenario = rospy.get_param('~scenario', 0)  # Default to scenario 0
        self.goal_x = rospy.get_param('~goal_x', 0.0)    # Default goal_x
        self.goal_y = rospy.get_param('~goal_y', 0.0)    # Default goal_y

        # Publishers
        self.robot_pub = rospy.Publisher('/scenario/output_robot', RobotState, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/scenario/output_obstacles', ObstacleArray, queue_size=10)
        self.goal_pub = rospy.Publisher('/scenario/goal', Vector3, queue_size=10)

        # Set up a timer to regularly publish the states and goal for scenarios 1 and 2
        if self.scenario in [1, 2]:
            rospy.Timer(rospy.Duration(1.0), self.publish_robot_state)
            rospy.Timer(rospy.Duration(1.0), self.publish_obstacle_state)
        else:
            # Subscribers for scenario 0 (bypass mode)
            self.robot_sub = rospy.Subscriber('/scenario/input_robot', RobotState, self.robot_callback)
            self.obstacle_sub = rospy.Subscriber('/scenario/input_obstacles', ObstacleArray, self.obstacle_callback)

        # Always publish the goal
        rospy.Timer(rospy.Duration(1.0), self.publish_goal)

    def publish_goal(self, event):
        goal = Vector3(x=self.goal_x, y=self.goal_y, z=0.0)
        self.goal_pub.publish(goal)

    def publish_robot_state(self, event):
        if self.scenario == 1:
            predefined_robot = RobotState(
                position=Point(x=10, y=10, z=0),
                velocity=Vector3(x=0.1, y=0.1, z=0),
                radius=1.0
            )
            self.robot_pub.publish(predefined_robot)
        elif self.scenario == 2:
            predefined_robot = RobotState(
                position=Point(x=20, y=20, z=0),
                velocity=Vector3(x=1, y=1, z=0),
                radius=1.5
            )
            self.robot_pub.publish(predefined_robot)

    def publish_obstacle_state(self, event):
        if self.scenario == 1:
            predefined_obstacles = ObstacleArray(obstacles=[
                ObstacleState(position=Point(x=15, y=15, z=0), velocity=Vector3(x=0, y=0, z=0), radius=2.0),
                ObstacleState(position=Point(x=13, y=13, z=0), velocity=Vector3(x=-1, y=-1, z=0), radius=1.5),
                ObstacleState(position=Point(x=11, y=11, z=0), velocity=Vector3(x=0, y=0, z=0), radius=1.5)
            ])
            self.obstacle_pub.publish(predefined_obstacles)
        elif self.scenario == 2:
            predefined_obstacles = ObstacleArray(obstacles=[
                ObstacleState(position=Point(x=50, y=50, z=0), velocity=Vector3(x=1, y=-1, z=0), radius=1.0),
                ObstacleState(position=Point(x=60, y=60, z=0), velocity=Vector3(x=-1, y=1, z=0), radius=1.2)
            ])
            self.obstacle_pub.publish(predefined_obstacles)

    def robot_callback(self, data):
        # Bypass mode
        if self.scenario == 0:
            self.robot_pub.publish(data)

    def obstacle_callback(self, data):
        # Bypass mode
        if self.scenario == 0:
            self.obstacle_pub.publish(data)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = ScenarioController()
    node.run()
