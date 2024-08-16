#!/usr/bin/env python3

import unittest
import rospy
import rostest
from geometry_msgs.msg import Point, Vector3
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray

class TestScenarioController(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_scenario_controller', anonymous=True)

        self.robot_msg_received = False
        self.obstacle_msg_received = False
        self.goal_msg_received = False

        # Publishers for the input topics (only used for scenario 0)
        self.robot_input_pub = rospy.Publisher('/scenario/input_robot', RobotState, queue_size=10)
        self.obstacle_input_pub = rospy.Publisher('/scenario/input_obstacles', ObstacleArray, queue_size=10)

        # Subscribers to the output topics
        rospy.Subscriber('/scenario/output_robot', RobotState, self.robot_callback)
        rospy.Subscriber('/scenario/output_obstacles', ObstacleArray, self.obstacle_callback)
        rospy.Subscriber('/scenario/goal', Vector3, self.goal_callback)

        rospy.sleep(1)  # Give some time for publishers and subscribers to connect

    def robot_callback(self, msg):
        self.robot_msg_received = True
        self.received_robot_msg = msg

    def obstacle_callback(self, msg):
        self.obstacle_msg_received = True
        self.received_obstacle_msg = msg

    def goal_callback(self, msg):
        self.goal_msg_received = True
        self.received_goal_msg = msg

    def test_scenario_0_with_published_inputs(self):
        # Set parameters for scenario 0
        rospy.set_param('~scenario', 0)
        rospy.set_param('~goal_x', 5.0)
        rospy.set_param('~goal_y', 7.5)

        # Publish test robot state
        test_robot = RobotState(
            position=Point(x=10, y=10, z=0),
            velocity=Vector3(x=1, y=1, z=0),
            radius=1.0
        )
        self.robot_input_pub.publish(test_robot)

        # Publish test obstacle state
        test_obstacles = ObstacleArray(obstacles=[
            ObstacleState(position=Point(x=20, y=20, z=0), velocity=Vector3(x=0, y=0, z=0), radius=2.0),
            ObstacleState(position=Point(x=30, y=30, z=0), velocity=Vector3(x=0, y=0, z=0), radius=1.5)
        ])
        self.obstacle_input_pub.publish(test_obstacles)

        rospy.sleep(5)  # Allow time for the messages to be processed

        # Check if robot state was received
        self.assertTrue(self.robot_msg_received, "Robot state was not received.")
        self.assertEqual(self.received_robot_msg.position.x, 10)
        self.assertEqual(self.received_robot_msg.position.y, 10)
        self.assertEqual(self.received_robot_msg.radius, 1.0)

        # Check if obstacle state was received
        self.assertTrue(self.obstacle_msg_received, "Obstacle state was not received.")
        self.assertEqual(len(self.received_obstacle_msg.obstacles), 2)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].position.x, 20)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].position.y, 20)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].radius, 2.0)

        # Check if goal was received
        self.assertTrue(self.goal_msg_received, "Goal was not received.")
        self.assertEqual(self.received_goal_msg.x, 5.0)
        self.assertEqual(self.received_goal_msg.y, 7.5)

    def test_scenario_1_predefined(self):
        # Set parameters for scenario 1
        rospy.set_param('~scenario', 1)
        rospy.set_param('~goal_x', 5.0)
        rospy.set_param('~goal_y', 7.5)

        rospy.sleep(2)  # Allow time for the node to process and publish

        # Check if robot state was received
        self.assertTrue(self.robot_msg_received, "Robot state was not received.")
        self.assertEqual(self.received_robot_msg.position.x, 10)
        self.assertEqual(self.received_robot_msg.position.y, 10)
        self.assertEqual(self.received_robot_msg.radius, 1.0)

        # Check if obstacle state was received
        self.assertTrue(self.obstacle_msg_received, "Obstacle state was not received.")
        self.assertEqual(len(self.received_obstacle_msg.obstacles), 2)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].position.x, 30)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].position.y, 30)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].radius, 2.0)

        # Check if goal was received
        self.assertTrue(self.goal_msg_received, "Goal was not received.")
        self.assertEqual(self.received_goal_msg.x, 5.0)
        self.assertEqual(self.received_goal_msg.y, 7.5)

    def test_scenario_2_predefined(self):
        # Set parameters for scenario 2
        rospy.set_param('~scenario', 2)
        rospy.set_param('~goal_x', 5.0)
        rospy.set_param('~goal_y', 7.5)

        rospy.sleep(2)  # Allow time for the node to process and publish

        # Check if robot state was received
        self.assertTrue(self.robot_msg_received, "Robot state was not received.")
        self.assertEqual(self.received_robot_msg.position.x, 20)
        self.assertEqual(self.received_robot_msg.position.y, 20)
        self.assertEqual(self.received_robot_msg.radius, 1.5)

        # Check if obstacle state was received
        self.assertTrue(self.obstacle_msg_received, "Obstacle state was not received.")
        self.assertEqual(len(self.received_obstacle_msg.obstacles), 2)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].position.x, 50)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].position.y, 50)
        self.assertEqual(self.received_obstacle_msg.obstacles[0].radius, 1.0)

        # Check if goal was received
        self.assertTrue(self.goal_msg_received, "Goal was not received.")
        self.assertEqual(self.received_goal_msg.x, 5.0)
        self.assertEqual(self.received_goal_msg.y, 7.5)

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('dynamic_obstacle_avoidance', 'test_scenario_controller', TestScenarioController)
