#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray
from forceCalculationApfModified import ObstacleAvoidance

class APFMAvoidance:
    def __init__(self):
        rospy.init_node('apfm_avoidance')

        self.obstacle_avoidance = ObstacleAvoidance()
        
        # Publishers for the forces
        self.repulsive_force_pub = rospy.Publisher('/apfm/repulsive_force', Vector3, queue_size=10)
        self.attractive_force_pub = rospy.Publisher('/apfm/attractive_force', Vector3, queue_size=10)
        self.total_force_pub = rospy.Publisher('/apfm/total_force', Vector3, queue_size=10)
        self.dynamic_force_pub = rospy.Publisher('/apfm/dynamic_force', Vector3, queue_size=10)
        self.static_force_pub = rospy.Publisher('/apfm/static_force', Vector3, queue_size=10)
        self.emergency_force_pub = rospy.Publisher('/apfm/emergency_force', Vector3, queue_size=10)

        # Subscriber to robot state, obstacles, and goal
        self.robot_sub = rospy.Subscriber('/scenario/output_robot', RobotState, self.process_data)
        self.obstacle_sub = rospy.Subscriber('/scenario/output_obstacles', ObstacleArray, self.obstacle_callback)
        self.goal_sub = rospy.Subscriber('/scenario/goal', Vector3, self.goal_callback)

        self.robot_state = None
        self.obstacles = None
        self.goal_position = None

    def obstacle_callback(self, data):
        self.obstacles = data.obstacles

    def goal_callback(self, data):
        self.goal_position = [data.x, data.y]

    def process_data(self, robot_state):
        self.robot_state = robot_state

        if self.obstacles and self.goal_position:
            # Extract positions, radii, and velocities of obstacles
            list_of_obstacle_positions = [[ob.position.x, ob.position.y] for ob in self.obstacles]
            list_of_obstacle_radii = [ob.radius for ob in self.obstacles]
            list_of_obstacle_velocities = [[ob.velocity.x, ob.velocity.y] for ob in self.obstacles]

            # Current robot position and velocity
            current_robot_position = [self.robot_state.position.x, self.robot_state.position.y]
            current_robot_velocity = [self.robot_state.velocity.x, self.robot_state.velocity.y]

            # Calculate forces
            total_force, attractive_force, repulsive_force, Frd, Frs, Fre = self.obstacle_avoidance.modified_potential_field(
                self.goal_position, list_of_obstacle_positions, list_of_obstacle_radii, list_of_obstacle_velocities, current_robot_position, current_robot_velocity)

            # Publish the forces
            self.publish_force(self.total_force_pub, total_force)
            self.publish_force(self.attractive_force_pub, attractive_force)
            self.publish_force(self.repulsive_force_pub, repulsive_force)
            self.publish_force(self.dynamic_force_pub, Frd)
            self.publish_force(self.static_force_pub, Frs)
            self.publish_force(self.emergency_force_pub, Fre)

    def publish_force(self, publisher, force):
        vector = Vector3(x=force[0], y=force[1], z=0.0)
        publisher.publish(vector)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = APFMAvoidance()
    node.run()
