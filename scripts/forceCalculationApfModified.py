#!/usr/bin/env python3

import numpy as np
import math
import rospy
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dynamic_obstacle_avoidance.msg import CustomInfo  
from std_msgs.msg import Float64

class ObstacleAvoidance:

    def __init__(self):
        # Initialize parameters from ROS
        self.attraction_scaling_factor = rospy.get_param('~attraction_scaling_factor', 3)
        self.obstacle_scaling_factor_dynamic = rospy.get_param('~obstacle_scaling_factor_dynamic', 20000)
        self.obstacle_scaling_factor_static = rospy.get_param('~obstacle_scaling_factor_static', 300000)
        self.scaling_factor_emergency = rospy.get_param('~scaling_factor_emergency', 20000)
        self.safety_margin_radius = rospy.get_param('~safety_margin_radius', 0.6)
        self.robot_domain_radius = rospy.get_param('~robot_domain_radius', 1.4)
        self.safe_distance = rospy.get_param('~safe_distance', 10)
        self.obstacle_influence_range = 3*self.safe_distance
        # self.distance_to_goal = rospy.get_param('~distance_to_goal', 0.1)

        self.custom_info_pub = rospy.Publisher('/obstacle_avoidance/custom_info', CustomInfo, queue_size=10)
        self.distance_to_goal_pub = rospy.Publisher('/obstacle_avoidance/distance_to_goal', Float64, queue_size=10)
        self.avoidance_type = "none"
        self.tau = None
        self.dm = None
        self.CR = None
    


    def set_parameters(self, **kwargs):
        """
        Define os parâmetros para o campo potencial modificado.

        :param kwargs: Dicionário contendo os parâmetros e seus respectivos valores.
        obstacle_avoidance.set_parameters(attraction_scaling_factor=5000, safe_distance=1.5, ... )
        """
        if 'attraction_scaling_factor' in kwargs:
            self.attraction_scaling_factor = kwargs['attraction_scaling_factor']

        if 'obstacle_scaling_factor_dynamic' in kwargs:
            self.obstacle_scaling_factor_dynamic = kwargs['obstacle_scaling_factor_dynamic']

        if 'obstacle_scaling_factor_static' in kwargs:
            self.obstacle_scaling_factor_static = kwargs['obstacle_scaling_factor_static']

        if 'scaling_factor_emergency' in kwargs:
            self.scaling_factor_emergency = kwargs['scaling_factor_emergency']

        if 'safety_margin_radius' in kwargs:
            self.safety_margin_radius = kwargs['safety_margin_radius']

        if 'robot_domain_radius' in kwargs:
            self.robot_domain_radius = kwargs['robot_domain_radius']

        if 'safe_distance' in kwargs:
            self.safe_distance = kwargs['safe_distance']

        if 'obstacle_influence_range' in kwargs:
            self.obstacle_influence_range = kwargs['obstacle_influence_range']


    def modified_attractive_force(self,pos, vector_to_goal, attraction_scaling_factor,distance_to_goal,normalized_vector_to_goal):
        attractive_force = attraction_scaling_factor * distance_to_goal *normalized_vector_to_goal
        #rospy.loginfo(f"Attractive Force: {attractive_force}")
        
        return attractive_force


    def calculate_Fre(self,distance_to_obstacle, safety_margin_radius, center_to_center_safe_distance, distance_to_goal, scaling_factor_emergency, obstacle_domain_radius, unit_vector_to_obstacle, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal,perpendicular_unit_vector_to_obstacle):
        var1 = (1./(distance_to_obstacle-safety_margin_radius)-1/center_to_center_safe_distance)
        var2 = (distance_to_goal**2)/((distance_to_obstacle-safety_margin_radius)**2)
        Fre1 = -2 * scaling_factor_emergency * obstacle_domain_radius * var1 * var2 * unit_vector_to_obstacle
        Fre2 = 20 * scaling_factor_emergency * obstacle_domain_radius * distance_to_goal / distance_to_obstacle * np.linalg.norm(relative_speed_vector)**2 * (np.cos(angle_between_direction_and_velocity) * np.sin(angle_between_direction_and_velocity)) * perpendicular_unit_vector_to_obstacle
        Fre3 = 2 * scaling_factor_emergency * obstacle_domain_radius * distance_to_goal * (var1**2 + np.linalg.norm(relative_speed_vector)**2 * np.cos(angle_between_direction_and_velocity)**2) * normalized_vector_to_goal
        Fre = Fre1 + Fre2 + Fre3
        return Fre


    def calculate_Frs(self,distance_to_obstacle, safety_margin_radius, distance_to_goal, obstacle_influence_range, obstacle_scaling_factor_static, obstacle_domain_radius, unit_vector_to_obstacle, normalized_vector_to_goal):
        var1 = 1/(distance_to_obstacle-safety_margin_radius) - 1/obstacle_influence_range
        var2 = (distance_to_goal**2)/(distance_to_obstacle**2)
        Frs1 = -obstacle_scaling_factor_static * obstacle_domain_radius * var1 * var2 * unit_vector_to_obstacle
        Frs3 = obstacle_scaling_factor_static * obstacle_domain_radius * distance_to_goal * var1**2 * normalized_vector_to_goal
        return Frs1 + Frs3  
        
    def calculate_Frd(self,distance_to_obstacle,center_to_center_safe_distance,obstacle_influence_range,angle_between_direction_and_velocity,relative_speed_vector,theta_m_degree,angle_difference_for_safety,vector_to_obstacle,obstacle_scaling_factor_dynamic,obstacle_domain_radius,distance_to_goal,unit_vector_to_obstacle,perpendicular_unit_vector_to_obstacle,normalized_vector_to_goal):
        var1 = ((1./(distance_to_obstacle-center_to_center_safe_distance))-(1/obstacle_influence_range))
        var2 = (center_to_center_safe_distance/(distance_to_obstacle*np.sqrt(distance_to_obstacle**2-center_to_center_safe_distance**2)))
        var3 = (np.sin(np.radians(angle_between_direction_and_velocity))/np.linalg.norm(relative_speed_vector))
        var4 = (np.sin(np.radians(theta_m_degree))/np.linalg.norm(relative_speed_vector))
        var5 = ((np.exp(angle_difference_for_safety)-1)/((distance_to_obstacle-center_to_center_safe_distance)**2))
        Fto0 = var1*(var2 + var4)
        var6 = (1/np.linalg.norm(vector_to_obstacle)+np.cos(np.radians(angle_between_direction_and_velocity))/np.linalg.norm(relative_speed_vector))
        var7 = np.linalg.norm(relative_speed_vector)*(np.exp(angle_difference_for_safety)-1)/(distance_to_obstacle*(distance_to_obstacle-center_to_center_safe_distance)**2)
        Ftop = var1*(1/np.linalg.norm(vector_to_obstacle)+np.cos(np.radians(theta_m_degree))/np.linalg.norm(relative_speed_vector)) 
        Frd1 = -obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal**2*(var1*np.exp(angle_difference_for_safety)*(var2 + var3)+var5-Fto0)*unit_vector_to_obstacle
        Frd3 = obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal**2*var1*(np.exp(angle_difference_for_safety)-1)*normalized_vector_to_goal 
        Frd2 = -obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal**2*(var1*np.exp(angle_difference_for_safety)*var6*var7-Ftop)*perpendicular_unit_vector_to_obstacle
        Frd = Frd1 + Frd2 + Frd3
        return Frd

    def iniciation(self,obstacle_position, current_robot_position, current_robot_velocity, obstacle_velocitiy, obstacle_radii, safe_distance, obstacle_influence_range, robot_domain_radius,normalized_vector_to_goal,numero_do_obstaculo):
        
        distance_to_obstacle = np.linalg.norm(obstacle_position - np.array(current_robot_position))
        center_to_center_safe_distance = robot_domain_radius + safe_distance + obstacle_radii #dm​=Ros​+dsafe​+Rts  [1]
        #print("robot_domain_radius = ",robot_domain_radius,"current_robot_position =", current_robot_position,"obstacle_position = ",obstacle_position, "obstáculo de número: ",numero_do_obstaculo)
        #collision_avoidance_radius: The collision_avoidance_radiusitical distance from the i-th obstacle. It is calculated as the sum of center_to_center_safe_distance aobstacle_scaling_factor_dynamic obstacle_influence_range.
        collision_avoidance_radius = center_to_center_safe_distance + obstacle_influence_range #CR=dm​+ρo​ [2]
        
        #vector_to_obstacle: The vector pointing from the current position of the boat to the i-th obstacle.
        vector_to_obstacle = np.array(obstacle_position) - np.array(current_robot_position)


        # Obter ângulo do obstáculo em relação à frente.
        # angle_to_obstacle = self.obstacle_angle(current_robot_position, current_robot_velocity, obstacle_position)
        # cross_z = self.cross_product_2d(current_robot_velocity, obstacle_velocitiy)
        relative_vel = np.array(current_robot_velocity) - np.array(obstacle_velocitiy)
        # Vetor_posição_relativo= np.array(obstacle_position) - np.array(current_robot_position)
        # cross_relative = self.cross_product_2d(relative_vel, Vetor_posição_relativo)


        # Normalizar vetor relativo
        rel_unit = relative_vel / np.linalg.norm(relative_vel)

        # Ângulo entre o vetor posição relativo e o vetor direção relativa
        dot_product = np.dot(vector_to_obstacle, rel_unit)
        angle_between_direction_and_velocity2 = np.arccos(np.clip(dot_product / np.linalg.norm(vector_to_obstacle), -1.0, 1.0))
        angle_between_direction_and_velocity = np.degrees(angle_between_direction_and_velocity2)
        # Ângulo limite para evitar colisão
        angle_for_safe_distance2 = np.arcsin(np.clip(center_to_center_safe_distance / np.linalg.norm(vector_to_obstacle), -1.0, 1.0))
        angle_for_safe_distance = np.degrees(angle_for_safe_distance2)  #θm​=arctan(ρ2(pos​,pts​)−dm2​​dm​  [3]​) 
        
        
        relative_speed_vector = np.array(current_robot_velocity) - np.array(obstacle_velocitiy)

 
        #unit_vector_to_obstacle is a unit vector pointing from the current position to the obstacle

        #unit_vector_to_obstacle = (obstacle_position - current_robot_position) / distance_to_bstacle
        unit_vector_to_obstacle = (np.array(obstacle_position) - np.array(current_robot_position)) / distance_to_obstacle
        angle_difference_for_safety = angle_for_safe_distance-angle_between_direction_and_velocity
        # angulacao = np.degrees(np.arccos(np.dot(unit_vector_to_obstacle, normalized_vector_to_goal)/(np.linalg.norm(unit_vector_to_obstacle) * np.linalg.norm(normalized_vector_to_goal))))
        #perpendicular_unit_vector_to_obstacle = comparar_vetores(vector_to_obstacle, relative_speed_vector, obstacle_velocitiy,unit_vector_to_obstacle,histerese)
        obstacle_domain_radius = obstacle_radii
        perpendicular_unit_vector_to_obstacle, sentido, avoidance_type = self.determine_avoidance_direction(current_robot_velocity,obstacle_velocitiy,unit_vector_to_obstacle,obstacle_position,current_robot_position,obstacle_domain_radius,robot_domain_radius)
        #perpendicular_unit_vector_to_obstacle = determiscaling_factor_emergency_side(vector_to_obstacle, relative_speed_vector,unit_vector_to_obstacle)
        
        
        return distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius, sentido, avoidance_type

    def determine_avoidance_direction(self,vr, vo,unit_vector_to_obstacle,obstacle_position,current_robot_position,obstacle_domain_radius,robot_domain_radius):
        """
        Decide a direção da rotação para desviar do obstáculo.

        :param vr: tuple (robot_velocity_x, robot_velocity_y), vetor velocidade do robô
        :param vo: tuple (obstacle_velocity_x, obstacle_velocity_y), vetor velocidade do obstáculo
        :return: string, "anti-horário", "horário" ou "paralelo"
        """
        #Parâmetros
        #self.safe_distance
        sentido = "None"  # Inicialização segura
        raio_barco = self.robot_domain_radius
        raio_obstaculo = obstacle_domain_radius
        dm = raio_barco + raio_obstaculo + self.safe_distance
        tau = self.safety_margin_radius
        self.CR = dm + 3 * self.safe_distance


        # Obter ângulo do obstáculo em relação à frente do robô.
        angle_to_obstacle = self.obstacle_angle(current_robot_position, vr, obstacle_position)

        # Produto vetorial para determinar direita/esquerda entre vetores de velocidade
        cross_z = self.cross_product_2d(vr, vo)

        # Velocidade relativa (robô em relação ao obstáculo)
        relative_vel = np.array(vr) - np.array(vo)

        # Vetor posição relativo (do robô até o obstáculo)
        d = np.array(obstacle_position) - np.array(current_robot_position)
        
        # Produto vetorial entre velocidade relativa e posição relativa
        cross_relative = self.cross_product_2d(relative_vel, d)

        # Verificar risco de colisão
        risk_of_collision = self.check_collision_risk(current_robot_position, vr, obstacle_position, vo, dm)

        # Determinar sentido (anti-horário ou horário) baseado em cross_relative
        # sentido = "horario" if cross_relative > 0 else "anti horario"

        # Classificar tipo de encontro se houver risco de colisão PARA MEU CÓDIGO
        # if risk_of_collision:
        #     if (0 <= angle_to_obstacle <= 15) or (345 <= angle_to_obstacle <= 360):
        #         avoidance_type = "HeadsOn"
        #         sentido = "horario" if cross_z > 0 else "anti horario"
        #     elif 15 < angle_to_obstacle <= 112.5:
        #         avoidance_type = "Crossing A"
        #     elif 247.5 <= angle_to_obstacle < 345:
        #         avoidance_type = "Crossing B"
        #     elif angle_to_obstacle > 112.5 or angle_to_obstacle <= 247.5:
        #         avoidance_type = "Overtaking"
        #         sentido = "horario" if cross_relative > 0 else "anti horario"
        #     else:
        #         avoidance_type = "No avoidance"
        # else:
        #     avoidance_type = "No avoidance"

        #CÓDIGO DO ARTIGO LIU
        if risk_of_collision:
            if (0 <= angle_to_obstacle <= 15) or (345 <= angle_to_obstacle <= 360):
                avoidance_type = "HeadsOn"
                sentido = "horario" 
            elif 15 < angle_to_obstacle <= 112.5:
                avoidance_type = "Crossing A"
                sentido = "horario" 
            elif 247.5 <= angle_to_obstacle < 345:
                avoidance_type = "Crossing B"
                sentido = "nada"
            elif angle_to_obstacle > 112.5 or angle_to_obstacle <= 247.5:
                avoidance_type = "Overtaking"
                sentido = "anti horario"
            else:
                avoidance_type = "No avoidance"
        else:
            avoidance_type = "No avoidance"


        if np.linalg.norm(d) < dm:
            sentido = "horario" if cross_relative < 0 else "anti horario"

        if sentido == "anti horario":
            avoidance_vector = np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]])
        elif sentido == "horario":
            avoidance_vector = -np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]])
        else:
            sentido == "horario"
            avoidance_vector = -np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]])
        


        return avoidance_vector, sentido, avoidance_type



        

    def obstacle_angle(self,barco_pos, barco_vel, obst_pos):
        # Vetor posição relativo
        d = np.array(obst_pos) - np.array(barco_pos)
        
        # Normalizar direção da velocidade do barco
        barco_vel_unit = barco_vel / np.linalg.norm(barco_vel)

        # Ângulo entre o vetor relativo e a frente do barco
        dot_product = np.dot(d, barco_vel_unit)
        d_magnitude = np.linalg.norm(d)
        angle_rad = np.arccos(np.clip(dot_product / d_magnitude, -1.0, 1.0))

        
        # Determinar o sentido do ângulo (horário/anti-horário)
        cross_z = d[0] * barco_vel_unit[1] - d[1] * barco_vel_unit[0]
        angle_deg = np.degrees(angle_rad)
        
        if cross_z < 0:
            angle_deg = 360 - angle_deg  # Ajuste para sentido horário
        
        return angle_deg

    def cross_product_2d(self,v1, v2):
        # Calcula o produto vetorial 2D (apenas x e y)
        return v1[0] * v2[1] - v1[1] * v2[0]

    def check_collision_risk(self,barco_pos, barco_vel, obst_pos, obst_vel, dm):
        """
        Verifica o risco de colisão com base no ângulo entre o vetor relativo e o vetor posição.
        """
        # Vetor posição relativo entre obstáculo e barco
        d = np.array(obst_pos) - np.array(barco_pos)
        
        # Velocidade relativa entre barco e obstáculo
        relative_vel = np.array(barco_vel) - np.array(obst_vel)
        
        if np.linalg.norm(relative_vel) < 1e-8:
            return False  # Sem velocidade relativa, sem risco de colisão
        
        d_magnitude = np.linalg.norm(d)
        if d_magnitude < 1e-8:
            return False  # Barco e obstáculo estão na mesma posição
        
        # Normalizar vetor relativo
        rel_unit = relative_vel / np.linalg.norm(relative_vel)
        
        # Ângulo entre o vetor posição relativo e o vetor direção relativa
        dot_product = np.dot(d, rel_unit)
        angle_between = np.arccos(np.clip(dot_product / (d_magnitude), -1.0, 1.0))
        
        # Ângulo limite para evitar colisão
        angle_limit = np.arcsin(np.clip(dm / d_magnitude, -1.0, 1.0))
        
        return angle_between <= angle_limit
    
    def getDistanceToGoal(self):
        if self.distance_to_goal is None:
            raise ValueError("distance_to_goal ainda não foi definido. Certifique-se de chamar modified_potential_field primeiro.")
        return self.distance_to_goal
    
    def plot_vector(self, x, y, fx, fy):
        # Função para atualizar os dados do gráfico
        def update(frame):
            quiver.set_UVC(fx, fy)
            return quiver,

        # Cria uma nova figura
        fig, ax = plt.subplots()
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)

        # Adiciona um quiver plot
        quiver = ax.quiver(x, y, fx, fy, angles='xy', scale_units='xy', scale=1, color='r')

        # Cria uma animação
        ani = animation.FuncAnimation(fig, update, blit=False, interval=1000)

        # Mostra o gráfico
        plt.show()

    def modified_potential_field(self,goal_position,list_of_obstacle_positions,list_of_obstacle_radii,list_of_obstacle_velocities,current_robot_position,current_robot_velocity):
        # goal_position - > vetor com posição x e y
        # list_of_obstacle_positions -> uma lista de vetores da posição do obstáculo.
        # list_of_obstacle_radii -> uma lista com os raios do obstáculo (correspondente com a posição na list_of_obstacle_position
        # list_of_obstacle_velocities -> uma lista de vetores da velocidade do obstáculo (correspondente com a posição e o raio da list_of_obstacle)
        # current_robot_position -> posição do robô no mundo
        # attraction_scaling_factor -> valor definido pelo projetista. Na simulação foi igual a 6000. No artigo de Liu foi igual a 3000
        # obstacle_scaling_factor_dynamic -> Seria o Nd no artigo. Valor definido pelo projetista. Simulação = 20000. Artigo = 2000
        # obstacle_scaling_factor_static -> Ns no artigo. Valor definido pelo projetista. Simulação = 3000000. Artigo = 300000
        # scaling_factor_emergency -> Ne no artigo. Valor definido pelo projetista. Simulação = 40000. Artigo = 2000
        # safety_margin_radius -> No artigo, chamado de tau, e na simulaçõa com valor de 0.3. área proibida. representa um pequeno raio de uma margem de segurança artificial para o robô (OS), garantindo que a superfície dos obstáculos tenha um potencial repulsivo suficientemente grande mas limitado
        # robot_domain_radius -> Raio do robô 
        # safe_distance -> representa a distância segura permissível entre o robô e um obstáculo
        # obstacle_influence_range -> representa o alcance de influência do conjunto de obstáculos (TS) e pode variar com base de um operador/projetista para condições como visibilidade baixa ou águas abertas. (variar entre 3 a 5). Foi usado 5 na simulação
        # current_robot_velocity -> vetor de velocidade atual do robô em relação ao mundo.

        
        self.tau = self.safety_margin_radius + self.robot_domain_radius
        vector_to_goal = [a - b for a, b in zip(goal_position, current_robot_position)]
        count = 0 #Inutilizado
        distance_to_goal = np.linalg.norm(vector_to_goal) 
        normalized_vector_to_goal = vector_to_goal/distance_to_goal
        
        self.distance_to_goal_pub.publish(distance_to_goal)

        attractive_force = np.zeros_like(current_robot_position)
        repulsive_force = np.zeros_like(current_robot_position)
        
        attractive_force = self.modified_attractive_force(current_robot_position, vector_to_goal, self.attraction_scaling_factor,distance_to_goal,normalized_vector_to_goal)
        # [6]
        Frd_total = 0
        Fre_total= 0
        Frs_total = 0
        for i,obstacle in enumerate(list_of_obstacle_positions):
            #initiate Frd,Frs,Fre
            Frd = np.zeros_like(current_robot_position)
            Fre = np.zeros_like(current_robot_position)
            Frs = np.zeros_like(current_robot_position)		
            numero_do_obstaculo = i+1 

            distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius, sentido, avoidance_type = self.iniciation(list_of_obstacle_positions[i], current_robot_position, current_robot_velocity, list_of_obstacle_velocities[i], list_of_obstacle_radii[i], self.safe_distance, self.obstacle_influence_range, self.robot_domain_radius,normalized_vector_to_goal,numero_do_obstaculo)
            
            if distance_to_obstacle <= collision_avoidance_radius:
                # Criação da mensagem CustomInfo
                custom_info_msg = CustomInfo()
                custom_info_msg.distance_to_obstacle = distance_to_obstacle
                custom_info_msg.center_to_center_safe_distance = center_to_center_safe_distance
                custom_info_msg.collision_avoidance_radius = collision_avoidance_radius
                custom_info_msg.vector_to_obstacle = vector_to_obstacle.tolist()
                custom_info_msg.angle_for_safe_distance = angle_for_safe_distance
                custom_info_msg.relative_speed_vector = relative_speed_vector.tolist()
                custom_info_msg.angle_between_direction_and_velocity = angle_between_direction_and_velocity
                custom_info_msg.unit_vector_to_obstacle = unit_vector_to_obstacle.tolist()
                custom_info_msg.angle_difference_for_safety = angle_difference_for_safety
                custom_info_msg.perpendicular_unit_vector_to_obstacle = perpendicular_unit_vector_to_obstacle.tolist()
                custom_info_msg.obstacle_domain_radius = obstacle_domain_radius
                custom_info_msg.distance_to_goal = distance_to_goal
                custom_info_msg.action_type = "rotacao " + sentido
                custom_info_msg.avoidance_type = avoidance_type
                custom_info_msg.CR = collision_avoidance_radius  #Raio de detecção
                custom_info_msg.dm = center_to_center_safe_distance  #raio de emergência
                custom_info_msg.angle_for_safe_distance = angle_for_safe_distance
                custom_info_msg.angle_between_direction_and_velocity = angle_between_direction_and_velocity

                # Publicação da mensagem em um tópico
                self.custom_info_pub.publish(custom_info_msg)

                tolerance = 1e-10         
                if distance_to_obstacle <= collision_avoidance_radius and angle_between_direction_and_velocity < angle_for_safe_distance and center_to_center_safe_distance < distance_to_obstacle:
                    # rospy.loginfo("PASSOU AKI: ")
                    if np.linalg.norm(list_of_obstacle_velocities[i]) > tolerance:
                        # rospy.loginfo("Dinamico: ")
                        custom_info_msg.action_type = "Obstáculo Dinâmico"
                        Frd = self.calculate_Frd(distance_to_obstacle,center_to_center_safe_distance,self.obstacle_influence_range,angle_between_direction_and_velocity,relative_speed_vector,angle_for_safe_distance,angle_difference_for_safety,vector_to_obstacle,self.obstacle_scaling_factor_dynamic,obstacle_domain_radius,distance_to_goal,unit_vector_to_obstacle,perpendicular_unit_vector_to_obstacle,normalized_vector_to_goal)
                    else:
                        # rospy.loginfo("Estatico: ")
                        custom_info_msg.action_type = "Obstáculo Estático"
                        Frs = self.calculate_Frs(distance_to_obstacle, self.safety_margin_radius, distance_to_goal, self.obstacle_influence_range, self.obstacle_scaling_factor_static, obstacle_domain_radius, unit_vector_to_obstacle, normalized_vector_to_goal)
                else:
                    if distance_to_obstacle <= center_to_center_safe_distance :
                        # rospy.loginfo("Emergencia: ")
                        custom_info_msg.action_type = "Obstáculo Emergência"
                        Fre = self.calculate_Fre(distance_to_obstacle, self.safety_margin_radius, center_to_center_safe_distance, distance_to_goal, self.scaling_factor_emergency, obstacle_domain_radius, unit_vector_to_obstacle, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal,perpendicular_unit_vector_to_obstacle)
                    else:
                        custom_info_msg.action_type = "Sem ação"

                # rospy.loginfo(f"Obstaculo numero {i+1}")
                # rospy.loginfo(f"Frd = {Frd}")#print("distance_to_obstacle =",distance_to_obstacle,"< collision_avoidance_radius =",collision_avoidance_radius," dinamic --------------  Frd = ",Frd)
                # rospy.loginfo(f"Frs = {Frs}")#print("distance_to_obstacle =",distance_to_obstacle,"< collision_avoidance_radius =",collision_avoidance_radius," static --------------  Frs = ",Frs)
                # rospy.loginfo(f"Fre = {Fre}")#print("distance_to_obstacle =",distance_to_obstacle,"< center_to_center_safe_distance =", center_to_center_safe_distance," --------------  Fre = ",Fre)
                
                Frd_total += Frd
                Fre_total += Fre
                Frs_total += Frs
                #print("obstaculo numero: ",i,"definicao do obst =",list_of_obstacle_positions[i],"velocidade do obstaculo",list_of_obstacle_velocities[i],"repulsive_force = ",repulsive_force,"angle_between_direction_and_velocity = ",angle_between_direction_and_velocity,"theta_m = ",angle_for_safe_distance)
        
        repulsive_force = Frd_total+Fre_total+Frs_total
        # rospy.loginfo(f"repulsive_force: { repulsive_force }  N")
        total_force = attractive_force + repulsive_force
        #self.plot_vector(current_robot_position[0], current_robot_position[1], total_force[0], total_force[1])
        #print("FORCA TOTAL = ",total_force)    
        return total_force, attractive_force, repulsive_force, Frd_total, Frs_total, Fre_total
