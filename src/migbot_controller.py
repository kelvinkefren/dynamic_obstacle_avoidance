import rospy
from geometry_msgs.msg import Twist, Wrench, Vector3
from gazebo_msgs.msg import ModelStates
from obstacle_avoidance import ObstacleAvoidance
import numpy as np

class Migbot:
    def __init__(self):
        self.pose = None
        self.twist = None

    def update(self, pose, twist):
        self.pose = pose
        self.twist = twist

class Obstacle:
    def __init__(self, name, radius):
        self.name = name
        self.pose = None
        self.twist = None
        self.radius = radius

    def update(self, pose, twist):
        self.pose = pose
        self.twist = twist

class MigbotController:
    def __init__(self, obstacle_avoidance, migbot, obstacles):
        self.obstacle_avoidance = obstacle_avoidance
        self.migbot = migbot
        self.obstacles = obstacles
        self.goal_position = np.array([70, 70])  # Defina a posição do objetivo

        # Subscribers e Publishers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        self.wrench_pub = rospy.Publisher('/Wrench', Wrench, queue_size=10)
        self.twist_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz

    def model_states_callback(self, data):
        for i, name in enumerate(data.name):
            if name == 'migbot':
                self.migbot.update(data.pose[i], data.twist[i])
            elif name in [ob.name for ob in self.obstacles]:
                [ob for ob in self.obstacles if ob.name == name][0].update(data.pose[i], data.twist[i])

    def calculate_force_and_publish(self):
        if self.migbot.pose and self.migbot.twist:
            obstacle_positions = np.array([[ob.pose.position.x, ob.pose.position.y] for ob in self.obstacles])
            obstacle_radii = np.array([ob.radius for ob in self.obstacles])
            obstacle_velocities = np.array([[ob.twist.linear.x, ob.twist.linear.y] for ob in self.obstacles])

            current_position = np.array([self.migbot.pose.position.x, self.migbot.pose.position.y])
            current_velocity = np.array([self.migbot.twist.linear.x, self.migbot.twist.linear.y])

            total_force = self.obstacle_avoidance.modified_potential_field(self.goal_position, obstacle_positions, obstacle_radii, obstacle_velocities, current_position, current_velocity)

            # Convertendo a força para comandos ROS
            force_vector = Vector3()
            force_vector.x = float(total_force[0])
            force_vector.y = float(total_force[1])
            force_vector.z = 0.0

            # Publicando no tópico Twist para controlar a velocidade do robô
            twist_msg = Twist()
            twist_msg.linear.x = min(1.0, force_vector.x)
            twist_msg.angular.z = 0.0  # Por enquanto, vamos assumir movimento em linha reta

            self.twist_pub.publish(twist_msg)

            # Publicando no tópico Wrench para aplicar a força no robô
            wrench_msg = Wrench()
            wrench_msg.force.x = 0
            wrench_msg.force.y = 0
            wrench_msg.force.z = 0
            # wrench_msg.force = force_vector
            self.wrench_pub.publish(wrench_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_force_and_publish()
            self.rate.sleep()

