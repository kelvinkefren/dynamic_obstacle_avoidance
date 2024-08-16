#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Point
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray

class DynamicObstacleAvoidance:
    def __init__(self):
        rospy.init_node('dynamic_obstacle_avoidance')

        # Parameters
        self.attraction_scaling_factor = rospy.get_param('~attraction_scaling_factor', 6000)
        self.obstacle_scaling_factor_dynamic = rospy.get_param('~obstacle_scaling_factor_dynamic', 20000)
        self.obstacle_scaling_factor_static = rospy.get_param('~obstacle_scaling_factor_static', 3000000)
        self.scaling_factor_emergency = rospy.get_param('~scaling_factor_emergency', 40000)
        self.safety_margin_radius = rospy.get_param('~safety_margin_radius', 0.3)
        self.robot_domain_radius = rospy.get_param('~robot_domain_radius', 0.5)
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)
        self.obstacle_influence_range = rospy.get_param('~obstacle_influence_range', 5)
        self.distance_to_goal = rospy.get_param('~distance_to_goal', 0.1)

        # Subscribers
        self.robot_sub = rospy.Subscriber('/scenario/output_robot', RobotState, self.robot_callback)
        self.obstacle_sub = rospy.Subscriber('/scenario/output_obstacles', ObstacleArray, self.obstacle_callback)
        self.goal_sub = rospy.Subscriber('/scenario/goal', Vector3, self.goal_callback)

        # Initialize data
        self.robot_data = None
        self.obstacle_data = None
        self.goal_data = None

    def robot_callback(self, data):
        self.robot_data = data
        self.process_data()

    def obstacle_callback(self, data):
        # Store the incoming obstacles
        self.obstacles = data.obstacles

        # Process the obstacle data
        self.list_of_obstacle_positions = [[ob.position.x, ob.position.y] for ob in self.obstacles]
        self.list_of_obstacle_radii = [ob.radius for ob in self.obstacles]
        self.list_of_obstacle_velocities = [[ob.velocity.x, ob.velocity.y] for ob in self.obstacles]

        self.process_data()

    def goal_callback(self, data):
        self.goal_data = data
        self.process_data()

    def process_data(self):
        # Ensure we have received data from all topics before processing
        if self.robot_data is None or self.obstacle_data is None or self.goal_data is None:
            return

        # Extracting robot position
        current_robot_position = self.robot_data.position
        current_robot_velocity = self.robot_data.velocity


        # Extracting goal position
        goal_position = self.goal_data

        # Calculating attraction force towards the goal
        attractive_force = self.calculate_attractive_force(current_robot_position, goal_position)

        # Calculating repulsive forces from obstacles
        total_repulsive_force = Vector3(0, 0, 0)
        for obstacle in self.obstacle_data.obstacles:
            repulsive_force = self.calculate_repulsive_force(current_robot_position, obstacle)
            total_repulsive_force.x += repulsive_force.x
            total_repulsive_force.y += repulsive_force.y

        # Combine forces
        combined_force = Vector3(
            attractive_force.x + total_repulsive_force.x,
            attractive_force.y + total_repulsive_force.y,
            0
        )

        rospy.loginfo(f"Combined Force: {combined_force}")

    def calculate_attractive_force(self, current_robot_position, goal_position):
        # Vector from robot to goal
        vector_to_goal = Vector3(
            goal_position.x - current_robot_position.x,
            goal_position.y - current_robot_position.y,
            0
        )
        # Distance to goal
        distance_to_goal = (vector_to_goal.x**2 + vector_to_goal.y**2)**0.5

        # Normalize vector to goal
        normalized_vector_to_goal = Vector3(
            vector_to_goal.x / distance_to_goal,
            vector_to_goal.y / distance_to_goal,
            0
        )

        # Calculate attractive force
        attractive_force = Vector3(
            self.attraction_scaling_factor * normalized_vector_to_goal.x,
            self.attraction_scaling_factor * normalized_vector_to_goal.y,
            0
        )

        return attractive_force

    def calculate_repulsive_force(self, current_robot_position, obstacle):
        # Vector from robot to obstacle
        vector_to_obstacle = Vector3(
            obstacle.position.x - current_robot_position.x,
            obstacle.position.y - current_robot_position.y,
            0
        )
        # Distance to obstacle
        distance_to_obstacle = (vector_to_obstacle.x**2 + vector_to_obstacle.y**2)**0.5

        # If the obstacle is within influence range
        if distance_to_obstacle < self.obstacle_influence_range:
            # Normalize vector to obstacle
            normalized_vector_to_obstacle = Vector3(
                vector_to_obstacle.x / distance_to_obstacle,
                vector_to_obstacle.y / distance_to_obstacle,
                0
            )

            # Calculate repulsive force
            repulsive_force = Vector3(
                -self.obstacle_scaling_factor_static * (1 / distance_to_obstacle**2) * normalized_vector_to_obstacle.x,
                -self.obstacle_scaling_factor_static * (1 / distance_to_obstacle**2) * normalized_vector_to_obstacle.y,
                0
            )

            return repulsive_force

        return Vector3(0, 0, 0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = DynamicObstacleAvoidance()
    node.run()
