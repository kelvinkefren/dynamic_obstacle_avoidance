#!/usr/bin/env python3


import math
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Wrench, Vector3, Twist, Quaternion
from forceCalculationApfModified import ObstacleAvoidance
import numpy as np
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import SetModelState
import signal
import os
from datetime import datetime
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time




# Definindo constantes 
ROBOT_NAME = 'migbot'  #Modelo a ser controlado
OBSTACLE_NAMES = ['vegetation1_buoy','vegetation2_buoy','vegetation3_buoy','branche1_buoy','branche2_buoy']  # Lista de nomes dos obstáculos, adicione mais conforme necessário
OBSTACLE_RADIUS = [1,1,1,9,7]
TOPIC_SUB = "/gazebo/model_states" #Topico do gazebo para pegar Pose e Twist dos models
TOPIC_PUB_TWIST = '/husky_velocity_controller/cmd_vel' #Topico para ser enviado o sinal de controle em velocidad. Incomplete
TOPIC_PUB_WRENCH = '/Wrench' #Topico para enviar o sinal de controle em força
MAX_LINEAR_SPEED = 1.0 #Maxima velocidade linear do robot
GOAL_DEFINITION = [70, 70] #Posicao que se deseja chegar
DISTANCE_THRESHOLD = 2 #Distancia para parar
MAX_WRENCH = 0.3  
MAX_TORQUE_WRENCH = 0.1
MIN_TORQUE_WRENCH = -MAX_TORQUE_WRENCH   #ε (epsilon): É um fator de atração escalar. Seu valor na simulação é de 3.000. Esse fator modula o campo potencial atrativo que direciona o navio em direção ao seu objetivo.

ATTRACTION_SCALING_FACTOR = 6000       #ηd (eta_d): É um fator de escala para obstáculos dinâmicos (Target Ships - TSs) a longa distância. Seu valor na simulação é de 2.000. Esse fator influencia a força repulsiva que os navios em movimento exercem sobre o navio observador (Own Ship - OS).
OBSTACLE_SCALING_FACTOR_DYNAMIC = 200000  # fatores de escala para os obstáculos dinâmicos
OBSTACLE_SCALING_FACTOR_STATIC = 30000*100  #ηs (eta_s): É um fator de escala para obstáculos estáticos a longa distância. Seu valor na simulação é de 300.000. Esse parâmetro determina a força repulsiva exercida por obstáculos fixos (como ilhas ou rochedos) no navio observador.
SCALING_FACTOR_EMERGENCY = 40000 #τ (tau): Representa um pequeno raio de margem de segurança artificial do OS. Seu valor é de 0,3 milhas náuticas (nm) = 555.600 metros. Ele garante que, na superfície dos obstáculos, o potencial repulsivo seja grande, mas limitado.
SAFETY_MARGIN_RADIUS = 6     # Ros É o raio de domínio do navio observado. Seu valor é de 0,5 nm = 926.000 metros.
ROBOT_DOMAIN_RADIUS = 2      #Dsafe É a distância segura permitida entre o OS e o TS. Ela varia de 0,5 = 926.000 metros a 1 nm= 1.852.000 metros, mas 1 nm é selecionado para águas abertas na sua simulação. Esta distância é determinada pelo espaço no mar, erros do sensor de posição e velocidade relativa do OS e TS em um passo de tempo.
SAFE_DISTANCE = 3 #distância segura permissível entre o robô e um obstáculo
OBSTACLE_INFLUENCE_RANGE = 20 #ρo (rho_o): É o alcance de influência do conjunto de TS definido pelos operadores. É maior em condições de baixa visibilidade ou águas abertas. Seu valor varia entre 3= 5.556.000 a 5 nm.= 9.260.000
#maxturn: Representa a máxima alteração de curso permitida. Seu valor é de 5 graus. Este parâmetro garante que o navio não faça manobras abruptas que poderiam ser perigosas.
#time step: Representa o intervalo de tempo entre cada etapa da simulação. Seu valor é de 15 segundos. Esse é o intervalo no qual o modelo de movimento é atualizado e a simulação avança.

INITIAL_ROBOT_POSE = np.array([10,10]) #Initial pose for the robot model
INITIAL_ROBOT_VELOCITY = np.array([0,0])

SCENARIOS = {
    'scenario_0': {
        'vegetation1_buoy': {'position': [58.1, 44.5], 'velocity': [-0.4, -0]},
        'vegetation2_buoy': {'position': [28, 28], 'velocity': [0, 0]},
        'vegetation3_buoy': {'position': [20.4, 43.0], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [54, 72], 'velocity': [0, -0.3]},
        'branche3_buoy': {'position': [36, 72], 'velocity': [0.28, -0.16]},
    },
    'scenario_1': {
        'vegetation1_buoy': {'position': [40, 40], 'velocity': [0, 0]},
        'vegetation2_buoy': {'position': [0, 90], 'velocity': [0, 0]},
        'vegetation3_buoy': {'position': [0, 80], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_2': {
        'vegetation1_buoy': {'position': [35, 20], 'velocity': [0, 1]},
        'vegetation2_buoy': {'position': [0, 90], 'velocity': [0, 0]},
        'vegetation3_buoy': {'position': [0, 80], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_3': {
        'vegetation1_buoy': {'position': [20, 60], 'velocity': [1.1, 0]},
        'vegetation2_buoy': {'position': [0, 90], 'velocity': [0, 0]},
        'vegetation3_buoy': {'position': [0, 80], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_4': {
        'vegetation1_buoy': {'position': [50, 10], 'velocity': [0, 1.5]},
        'vegetation2_buoy': {'position': [70, 40], 'velocity': [-0.5, 0.5]},
        'vegetation3_buoy': {'position': [0, 80], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_5': {
        'vegetation1_buoy': {'position': [25, 40], 'velocity': [0.5, -0.5]},
        'vegetation2_buoy': {'position': [55, 55], 'velocity': [0, 0]},
        'vegetation3_buoy': {'position': [0, 80], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_6': {
        'vegetation1_buoy': {'position': [20, 20], 'velocity': [1, 1]},
        'vegetation2_buoy': {'position': [40, 60], 'velocity': [-0.5, -0.5]},
        'vegetation3_buoy': {'position': [0, 80], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_7': {
        'vegetation1_buoy': {'position': [30, 70], 'velocity': [0, -1.4]},
        'vegetation2_buoy': {'position': [70, 30], 'velocity': [-2.5, 0]},
        'vegetation3_buoy': {'position': [0, 80], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_8': {
        'vegetation1_buoy': {'position': [55, 10], 'velocity': [0, 1.5]},
        'vegetation2_buoy': {'position': [60, 70], 'velocity': [-0.5, -0.5]},
        'vegetation3_buoy': {'position': [80, 50], 'velocity': [0, 0]},
        'branche1_buoy': {'position': [0, 10], 'velocity': [0, 0]},
        'branche2_buoy': {'position': [0, 0], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [0, 5], 'velocity': [0, 0]},
    },
    'scenario_9': {
        'vegetation1_buoy': {'position': [20, 40], 'velocity': [0, 1]},
        'vegetation2_buoy': {'position': [35, 55], 'velocity': [1, 0]},
        'vegetation3_buoy': {'position': [50, 40], 'velocity': [0, -1]},
        'branche1_buoy': {'position': [60, 20], 'velocity': [-1, 0]},
        'branche2_buoy': {'position': [70, 40], 'velocity': [0, 1]},
        'branche3_buoy': {'position': [5, 5], 'velocity': [0, 0]},
    },
    'scenario_10': {
        'vegetation3_buoy': {'position': [70.2, 19.8], 'velocity': [0, 0]},
        'vegetation2_buoy': {'position': [61.2, 44.1], 'velocity': [0, 0]},
        'vegetation1_buoy': {'position': [50, 20], 'velocity': [-1, 1]},
        'branche1_buoy': {'position': [67.5, 63], 'velocity': [-0.8, -0.8]},
        'branche2_buoy': {'position': [54, 72], 'velocity': [0, -0.3]},
        'branche3_buoy': {'position': [36, 72], 'velocity': [0.28, -0.16]},
    }

}


chosen_scenario = SCENARIOS['scenario_10']

class Migbot:
    def __init__(self):
        self.pose = None
        self.twist = None

    def update(self, pose, twist):
        self.pose = pose
        self.twist = twist

class Obstacle:
    def __init__(self, name, radius=3):
        self.name = name
        self.pose = None
        self.twist = None
        self.radius = radius

    def update(self, pose, twist):
        self.pose = pose
        self.twist = twist
        #rospy.loginfo(f"Name: { self.name }")
        #rospy.loginfo(f"{ self.pose }")
        #rospy.loginfo(f"{ self.twist }" )
   
class MigbotController:
    def __init__(self, migbot, obstacles):
        self.migbot = migbot
        self.obstacles = obstacles
        self.obstacle_avoidance = ObstacleAvoidance()  # Instância da classe ObstacleAvoidance
        
        self.side_decision = 1  # Inicialmente, vamos para (10,10)
        self.GOAL_1 = np.array(GOAL_DEFINITION)
        self.GOAL_2 = np.array(INITIAL_ROBOT_POSE)
        self.current_goal = self.GOAL_1 if self.side_decision == 1 else self.GOAL_2

        # Subscribers e Publishers
        rospy.Subscriber(TOPIC_SUB, ModelStates, self.model_states_callback)
        self.wrench_pub = rospy.Publisher(TOPIC_PUB_WRENCH, Wrench, queue_size=10)
        self.Twist_pub = rospy.Publisher(TOPIC_PUB_TWIST, Twist, queue_size=10)

        self.filename = self.generate_filename()
        self.time_step_counter = 0
        self.velocity_change_interval = 100  # muda a velocidade a cada 100 etapas de tempo


        # Definindo a taxa
        self.rate = rospy.Rate(10)  # 10 Hz

    def calculate_new_velocity(self, current_vx, current_vy, angle_change_degrees=-30):
        # Convertendo graus para radianos
        alpha = np.radians(angle_change_degrees)
        
        # Criando a matriz de transformação
        A = np.array([[np.cos(alpha), np.sin(alpha)], [-np.sin(alpha), np.cos(alpha)]])
        
        # Calculando a nova velocidade
        new_velocity = np.dot(A, np.array([current_vx, current_vy]))
        return new_velocity

    def model_states_callback(self, data):
        for i, name in enumerate(data.name):
            if name == ROBOT_NAME:
                self.migbot.update(data.pose[i], data.twist[i])
                # Logando as informações do migbot
                #rospy.loginfo(f"Migbot Position: {data.pose[i].position}")
                #rospy.loginfo(f"Migbot Velocity: {data.twist[i].linear}")
            elif name in [ob.name for ob in self.obstacles]:
                [ob for ob in self.obstacles if ob.name == name][0].update(data.pose[i], data.twist[i])

                # Logando as informações do obstáculo
                #rospy.loginfo(f"Obstacle {name} Position: {data.pose[i].position}")
                #rospy.loginfo(f"Obstacle {name} Velocity: {data.twist[i].linear}")

    def save_data_to_csv(self, filename, migbot, obstacles, 
                        distance_to_goal, distances_to_obstacles, 
                        attractive_force_magnitude, repulsive_forces_magnitude, 
                        total_force_magnitude, orientation,Frd_magnitude,Fre_magnitude,Frs_magnitude):

        # Check if the file already exists (to determine if we need to write the header)
        write_header = not os.path.exists(self.filename)

        with open(filename, 'a') as file:
            if write_header:
                headers = ["Timestamp",
                    "Migbot_Pos_x", "Migbot_Pos_y", "Migbot_Pos_z", 
                    "Migbot_Vel_x", "Migbot_Vel_y", "Migbot_Vel_z", 
                    "Distance_to_goal", "Orientation", "Attraction_force_magnitude", 
                    "repulsive_forces_magnitude", "Total_force_magnitude", "Frd_magnitude",
                    "Fre_magnitude","Frs_magnitude", "migbot_velocity_magnitude"
                ]
            
                for ob in obstacles:
                    headers.extend([
                        f"{ob.name}_Pos_x", f"{ob.name}_Pos_y", f"{ob.name}_Pos_z",
                        f"{ob.name}_Vel_x", f"{ob.name}_Vel_y", f"{ob.name}_Vel_z",
                        f"Distance_to_{ob.name}"
                    ])
            
                file.write(",".join(headers) + "\n")

            # Obtenha o timestamp atual
            current_time = rospy.get_time()
            migbot_velocity_magnitude = math.sqrt( migbot.twist.linear.x*migbot.twist.linear.x + migbot.twist.linear.y*migbot.twist.linear.y)
            data = [current_time,
                migbot.pose.position.x, migbot.pose.position.y, migbot.pose.position.z,
                migbot.twist.linear.x, migbot.twist.linear.y, migbot.twist.linear.z,
                distance_to_goal, orientation, attractive_force_magnitude, 
                repulsive_forces_magnitude, total_force_magnitude, Frd_magnitude, Fre_magnitude,
                Frs_magnitude, migbot_velocity_magnitude
            ]
            
            for ob, distance in zip(obstacles, distances_to_obstacles):
                data.extend([
                    ob.pose.position.x, ob.pose.position.y, ob.pose.position.z,
                    ob.twist.linear.x, ob.twist.linear.y, ob.twist.linear.z,
                    distance
                ])
            
            file.write(",".join(map(str, data)) + "\n")
            
    def generate_filename(self):
        # Generate a timestamped filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"migbot_datafile_{timestamp}.csv"
        # Use expanduser to find the home directory
        home_directory = os.path.expanduser('~')
        # Combine the home directory with the filename
        full_path = os.path.join(home_directory, filename)
        return full_path
  
    def apf_modified_total_force(self):
        # Aqui, chamamos a função modified_potential_field para calcular a força total
        # Por enquanto, estou retornando um valor fictício
        goal_position = self.current_goal  # Exemplo de posição do objetivo

        list_of_obstacle_positions = [[ob.pose.position.x, ob.pose.position.y] for ob in self.obstacles]
        list_of_obstacle_radii = [ob.radius for ob in self.obstacles]
        list_of_obstacle_velocities = [[ob.twist.linear.x, ob.twist.linear.y] for ob in self.obstacles]
        # Convertendo a posição e a velocidade do robô para uma lista
        current_robot_position = [self.migbot.pose.position.x, self.migbot.pose.position.y]
        current_robot_velocity = [self.migbot.twist.linear.x, self.migbot.twist.linear.y]

        #current_robot_position = self.migbot.pose
        #current_robot_velocity = self.migbot.twist

        total_force, attractive_force, repulsive_force, Frd, Frs, Fre  = self.obstacle_avoidance.modified_potential_field(goal_position, list_of_obstacle_positions, list_of_obstacle_radii, list_of_obstacle_velocities, current_robot_position, current_robot_velocity)
        
        # Converta a força em um vetor e torque (a lógica de conversão precisa ser adicionada)

        # Converta a força em um vetor
        force_vector = Vector3()
        force_vector.x = float(abs(total_force[0]))  # Componente x da força
        force_vector.y = 0.0 #total_force[1]  # Componente y da força
        force_vector.z = 0.0  # Assumindo que não há força na direção z

        # Calcular a distância ao objetivo
        distance_to_goal = math.sqrt(
            (current_robot_position[0] - self.current_goal[0]) ** 2 +
            (current_robot_position[1] - self.current_goal[1]) ** 2
        )
        
        # Distances to obstacles
        distances_to_obstacles = [np.linalg.norm(np.array([ob.pose.position.x, ob.pose.position.y]) - current_robot_position) 
                                  for ob in self.obstacles]

        # Forces magnitudes
        attractive_force_magnitude = np.linalg.norm(attractive_force)
        repulsive_forces_magnitude = np.linalg.norm(repulsive_force)
        total_force_magnitude = np.linalg.norm(total_force)
        Frd_magnitude = np.linalg.norm(Frd)
        Fre_magnitude = np.linalg.norm(Fre)
        Frs_magnitude = np.linalg.norm(Frs)

        # Orientation
        orientation_q = self.migbot.pose.orientation
        _, _, orientation = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        orientation = np.degrees(orientation)  # Convert radian to degrees
        
        # Verificar se a distância ao objetivo é menor que um limiar
        self.save_data_to_csv(self.filename, self.migbot, self.obstacles, 
                              distance_to_goal, distances_to_obstacles, 
                              attractive_force_magnitude, repulsive_forces_magnitude, 
                              total_force_magnitude, orientation,Frd_magnitude,Fre_magnitude,Frs_magnitude)

        if distance_to_goal < DISTANCE_THRESHOLD:
            if self.side_decision == 1:
                self.current_goal = self.GOAL_2
                self.side_decision = 0
            else:
                self.current_goal = self.GOAL_1
                self.side_decision = 1
            # Parar o migbot
            Twist_msg = Twist()
            self.Twist_pub.publish(Twist_msg)
            wrench_msg = Wrench()
            wrench_msg.force.x = 0.0
            wrench_msg.force.y = 0.0
            wrench_msg.force.z = 0.0
            wrench_msg.torque.x = 0.0
            wrench_msg.torque.y = 0.0
            wrench_msg.torque.z = 0.0
            
            self.wrench_pub.publish(wrench_msg)
            return
            
        
            


        # Convertendo o quaternião para ângulos de Euler
    
        orientation_q = self.migbot.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # Agora a variável yaw contém o ângulo de rotação ao redor do eixo z
        heading = yaw
        

        # Certifique-se de que ambos os ângulos estão no intervalo [-pi, pi]
        angle_force = np.arctan2(total_force[1], total_force[0])
        angle_force = (angle_force + np.pi) % (2 * np.pi) - np.pi
        angle_force_degrees = math.degrees(angle_force)

        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        yaw_degrees = math.degrees(yaw)

        # Calcule a diferença angular e ajuste para o intervalo [-pi, pi]
        angular_difference =   -(angle_force - yaw)
        angular_difference = (angular_difference + np.pi) % (2 * np.pi) - np.pi
        angular_difference_degree = math.degrees(angular_difference)  



        #rospy.loginfo(f"yaw: { yaw_degrees }  graus")
        #rospy.loginfo(f"angle_force: { angle_force_degrees }  graus")
        #rospy.loginfo(f"Robot position: { current_robot_position }  N")
        #angular_difference = np.arctan2(total_force[1], total_force[0]) - heading


        

        # Converta essa diferença angular em torque
        # Por simplicidade, vamos assumir um fator de proporção de 1.0
        proportion_factor = 1.0
        torque_value = proportion_factor * angular_difference

        torque_vector = Vector3()
        torque_vector.x = 0.0
        torque_vector.y = 0.0
        torque_vector.z = torque_value  # O torque será aplicado na direção z


        return force_vector, torque_vector
    

    def run(self):
        while not rospy.is_shutdown():
            if self.migbot.pose and self.migbot.twist:
                force, torque = self.apf_modified_total_force()
                rospy.loginfo("Calculou a força e o torque")
                wrench_msg = Wrench()
                Twist_msg = Twist()
                #rospy.loginfo(f"heading: {self.obstacle_avoidance.getDistanceToGoal():.2f}")
                            
                wrench_msg.force.x = min(MAX_WRENCH,force.x)
                wrench_msg.torque.z = max(MIN_TORQUE_WRENCH, min(MAX_TORQUE_WRENCH, torque.z))
                Twist_msg.linear.x = min(MAX_LINEAR_SPEED,force.x)
                Twist_msg.angular.z = max(MIN_TORQUE_WRENCH, min(MAX_TORQUE_WRENCH, torque.z)) #torque.z

                rospy.loginfo(f"toque = {torque.z}")
                
                # Calculando a velocidade linear
                linear_speed = math.sqrt(Twist_msg.linear.x**2 + Twist_msg.linear.y**2 + Twist_msg.linear.z**2)
                wrench_aplied = math.sqrt(wrench_msg.force.x**2 + wrench_msg.force.y**2 + wrench_msg.force.z**2)
                
                self.Twist_pub.publish(Twist_msg)            
                self.wrench_pub.publish(wrench_msg)  
                #rospy.loginfo(f"Velocidade Linear: {linear_speed:.2f} m/s")


                # Calculando a velocidade vetorial e seu módulo
                vel_x = self.migbot.twist.linear.x
                vel_y = self.migbot.twist.linear.y
                vel_z = self.migbot.twist.linear.z
                pos_x = self.migbot.pose.position.x
                pos_y = self.migbot.pose.position.y
                pos_z = self.migbot.pose.position.z


                velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

                rospy.loginfo(f"Força: {force} N")    
                rospy.loginfo(f"Velocidade Vetorial: x={vel_x:.2f} m/s, y={vel_y:.2f} m/s, z={vel_z:.2f} m/s")
                rospy.loginfo(f"Posicao : x={pos_x:.2f} m/s, y={pos_y:.2f} m/s, z={pos_z:.2f} m/s")
                rospy.loginfo(f"Velocidade em Módulo: {velocity_magnitude:.2f} m/s")

                self.time_step_counter += 1
                if self.time_step_counter % self.velocity_change_interval == 0:
                    for obstacle in self.obstacles:
                        new_velocity = self.calculate_new_velocity(obstacle.twist.linear.x, obstacle.twist.linear.y)  # Esta função retorna a nova velocidade para o obstáculo
                        set_obstacle_position_and_velocity(obstacle.name, [obstacle.pose.position.x, obstacle.pose.position.y], new_velocity)




            self.rate.sleep()


def set_obstacle_position_and_velocity(name, position, velocity):
    rospy.wait_for_service('/gazebo/set_model_state')
    
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = name
        state.pose.position.x = position[0]
        state.pose.position.y = position[1]
        state.twist.linear.x = velocity[0]
        state.twist.linear.y = velocity[1]
        state.pose.orientation = Quaternion(0, 0, 0.3827, 0.9239)  # Sem rotação

        response = set_state(state)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)




def cleanup(scenarios):
    for obstacle_name, obstacle_data in scenarios.items():  # Assumindo 'scenario_1' como o cenário padrão
        rospy.loginfo(f"Resetting: obstacle_name = {obstacle_name}")
        set_obstacle_position_and_velocity(obstacle_name, obstacle_data['position'], [0, 0])

def sigint_handler(sig, frame, scenario):
    cleanup(scenario)
    rospy.signal_shutdown("Terminating...")

if __name__ == '__main__':
    rospy.init_node('migbot_controller', anonymous=True)
    #rospy.loginfo("Node iniciated.")
    migbot = Migbot()
    obstacles = [Obstacle(name, radius) for name, radius in zip(OBSTACLE_NAMES, OBSTACLE_RADIUS)]   # Usando a constante aqui

    controller = MigbotController(migbot, obstacles)
    controller.obstacle_avoidance.set_parameters(
        attraction_scaling_factor=ATTRACTION_SCALING_FACTOR, 
        obstacle_scaling_factor_dynamic= OBSTACLE_SCALING_FACTOR_DYNAMIC,
        obstacle_scaling_factor_static = OBSTACLE_SCALING_FACTOR_STATIC,
        scaling_factor_emergency = SCALING_FACTOR_EMERGENCY,
        safety_margin_radius = SAFETY_MARGIN_RADIUS,
        robot_domain_radius = ROBOT_DOMAIN_RADIUS,        
        safe_distance=SAFE_DISTANCE,
        obstacle_influence_range = OBSTACLE_INFLUENCE_RANGE,
    )

    success = set_obstacle_position_and_velocity(ROBOT_NAME, INITIAL_ROBOT_POSE, INITIAL_ROBOT_VELOCITY)

    
    for obstacle_name, obstacle_data in chosen_scenario.items():
        rospy.loginfo(f"passou aqui: obstacle_name = {obstacle_name}, position = {obstacle_data['position']}, velocity = {obstacle_data['velocity']}")
        set_obstacle_position_and_velocity(obstacle_name, obstacle_data['position'], obstacle_data['velocity'])

    

    controller.run()

    signal.signal(signal.SIGINT, sigint_handler, chosen_scenario)
