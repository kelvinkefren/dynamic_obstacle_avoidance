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
        self.attraction_scaling_factor = rospy.get_param('~attraction_scaling_factor', 3000)
        self.obstacle_scaling_factor_dynamic = rospy.get_param('~obstacle_scaling_factor_dynamic', 2000)
        self.obstacle_scaling_factor_static = rospy.get_param('~obstacle_scaling_factor_static', 300000)
        self.scaling_factor_emergency = rospy.get_param('~scaling_factor_emergency', 2000)
        self.safety_margin_radius = rospy.get_param('~safety_margin_radius', 0.6)
        self.robot_domain_radius = rospy.get_param('~robot_domain_radius', 1.4)
        self.safe_distance = rospy.get_param('~safe_distance', 10)
        self.obstacle_influence_range = 3*self.safe_distance
        # self.distance_to_goal = rospy.get_param('~distance_to_goal', 0.1)

        self.custom_info_pub = rospy.Publisher('/obstacle_avoidance/custom_info', CustomInfo, queue_size=10)
        self.distance_to_goal_pub = rospy.Publisher('/obstacle_avoidance/distance_to_goal', Float64, queue_size=10)
        self.avoidance_type = "none"
        
        self.dm = None
        self.CR = None
        self.tecnica =1
        self.crossing_b = 0
        self.previous = "none"
        self.state = "none"
        self.collision_pub = rospy.Publisher('/obstacle_avoidance/collision', Bool, queue_size=10)
    


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
        Fre2 = 2 * scaling_factor_emergency * obstacle_domain_radius * distance_to_goal / distance_to_obstacle * np.linalg.norm(relative_speed_vector)**2 * (np.cos(angle_between_direction_and_velocity) * np.sin(angle_between_direction_and_velocity)) * perpendicular_unit_vector_to_obstacle
        Fre3 = 2 * scaling_factor_emergency * obstacle_domain_radius * distance_to_goal * (var1**2 + np.linalg.norm(relative_speed_vector)**2 * np.cos(angle_between_direction_and_velocity)**2) * normalized_vector_to_goal
        Fre = Fre1 + Fre2 + Fre3
        return Fre


    def calculate_Frs(self,distance_to_obstacle, safety_margin_radius, distance_to_goal, obstacle_influence_range, obstacle_scaling_factor_static, obstacle_domain_radius, unit_vector_to_obstacle, normalized_vector_to_goal):
        var1 = 1/(distance_to_obstacle-safety_margin_radius) - 1/obstacle_influence_range
        var2 = (distance_to_goal**2)/(distance_to_obstacle**2)
        Frs1 = -obstacle_scaling_factor_static * obstacle_domain_radius * var1 * var2 * unit_vector_to_obstacle
        Frs3 = obstacle_scaling_factor_static * obstacle_domain_radius * distance_to_goal * var1**2 * normalized_vector_to_goal
        return Frs1 + Frs3  
        
    def calculate_Frd(self,distance_to_obstacle,center_to_center_safe_distance,obstacle_influence_range,angle_between_direction_and_velocity,relative_speed_vector,theta_m_degree,angle_difference_for_safety,vector_to_obstacle,obstacle_scaling_factor_dynamic,obstacle_domain_radius,distance_to_goal,unit_vector_to_obstacle,perpendicular_unit_vector_to_obstacle,normalized_vector_to_goal,avoidance_type):
        var1 = ((1./(distance_to_obstacle-center_to_center_safe_distance))-(1/obstacle_influence_range))
        var2 = (center_to_center_safe_distance/(distance_to_obstacle*np.sqrt(distance_to_obstacle**2-center_to_center_safe_distance**2)))
        var3 = (np.sin(np.radians(angle_between_direction_and_velocity))/np.linalg.norm(relative_speed_vector))
        var4 = (np.sin(np.radians(theta_m_degree))/np.linalg.norm(relative_speed_vector))
        var5 = ((np.exp(angle_difference_for_safety)-1)/((distance_to_obstacle-center_to_center_safe_distance)**2))
        Fto0 = var1*(var2 + var4)
        var6 = (1/np.linalg.norm(vector_to_obstacle)+np.cos(np.radians(angle_between_direction_and_velocity))/np.linalg.norm(relative_speed_vector))
        var7 = np.linalg.norm(relative_speed_vector)*(np.exp(angle_difference_for_safety)-1)/(distance_to_obstacle*(distance_to_obstacle-center_to_center_safe_distance)**2)
        Ftop = var1*(1/np.linalg.norm(vector_to_obstacle)+np.cos(np.radians(theta_m_degree))/np.linalg.norm(relative_speed_vector)) 
        Frd1 = -obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal*(var1*np.exp(angle_difference_for_safety)*(var2 + var3)+var5-Fto0)*unit_vector_to_obstacle
        # Frd1 = np.zeros_like(relative_speed_vector)    
        Frd3 = obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal*var1*(np.exp(angle_difference_for_safety)-1)*normalized_vector_to_goal 
        Frd2 = obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal**2*(var1*np.exp(angle_difference_for_safety)*var6*var7-Ftop)*perpendicular_unit_vector_to_obstacle
        if self.crossing_b==1:
            Frd1 = np.zeros_like(relative_speed_vector) 
            Frd2 = np.zeros_like(relative_speed_vector)    
            Frd3 = np.zeros_like(relative_speed_vector)   
         
        # if self.tecnica == 1 and avoidance_type =="crossing A":
            # Frd1 = np.zeros_like(relative_speed_vector)
            

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

        perpendicular_unit_vector_to_obstacle, sentido, avoidance_type,avoidance_type_robot_from_obst, manobra_desvio = self.determine_avoidance_direction(current_robot_velocity,obstacle_velocitiy,unit_vector_to_obstacle,obstacle_position,current_robot_position,obstacle_domain_radius,normalized_vector_to_goal)
        #perpendicular_unit_vector_to_obstacle = determiscaling_factor_emergency_side(vector_to_obstacle, relative_speed_vector,unit_vector_to_obstacle)
        
        
        return distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius, sentido, avoidance_type, avoidance_type_robot_from_obst, manobra_desvio

    def angle_robot_from_obstacle(self,obstacle_position, obstacle_velocity, current_robot_position):
        """
        Retorna o ângulo (em graus) em que o robô se encontra em relação ao obstáculo.
        O vetor 'obstacle_velocity' define o ângulo 0°, e ele cresce no sentido horário (clockwise).

        :param obstacle_position: np.array([ox, oy]) -> Posição (x, y) do obstáculo
        :param obstacle_velocity: np.array([vx, vy]) -> Velocidade (vx, vy) do obstáculo (definindo 0°)
        :param current_robot_position: np.array([rx, ry]) -> Posição (x, y) do robô
        :return: Ângulo em graus no intervalo [0, 360).
        """
        # Vetor do obstáculo até o robô
        d = np.array(current_robot_position) - np.array(obstacle_position)
        
        # Magnitude da velocidade do obstáculo
        vo_mag = np.linalg.norm(obstacle_velocity)
        
        # Se o obstáculo estiver parado (ou quase parado), 
        # não há "frente" bem definida => retorne 0° ou o que fizer mais sentido
        if vo_mag < 1e-8:
            return 0.0

        # Normaliza a velocidade do obstáculo
        vo_unit = obstacle_velocity / vo_mag
        
        # Produto escalar para obter o ângulo em módulo (0° a 180°)
        dot_prod = np.dot(d, vo_unit)
        d_mag = np.linalg.norm(d)
        
        # Se o robô estiver exatamente na mesma posição do obstáculo, 
        # devolva 0.0 ou trate de outra forma
        if d_mag < 1e-8:
            return 0.0
        
        # Ângulo em radianos entre d e vo_unit (sem considerar sentido ainda)
        angle_rad = np.arccos(np.clip(dot_prod / d_mag, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        
        # Produto vetorial 2D para determinar se está "à direita" (horário) ou "à esquerda" (anti-horário)
        # cross_z > 0 => d está "à esquerda" de vo_unit  (em termos do padrão matemático CCW)
        # cross_z < 0 => d está "à direita"  de vo_unit
        cross_z = (vo_unit[0] * d[1]) - (vo_unit[1] * d[0])
        
        # Queremos que o ângulo cresça no sentido HORÁRIO.
        # Pela convenção geométrica usual, cross_z > 0 significaria d acima de vo (CCW).
        # Para inverter para clockwise, fazemos:
        if cross_z > 0:
            # Se cross_z for positivo, ao invés de  angle_deg, use 360 - angle_deg
            angle_deg = 360.0 - angle_deg
        
        # Ajustar para garantir dentro de [0, 360)
        angle_deg = angle_deg % 360.0
        
        return angle_deg
    def determine_avoidance_direction(self,vr, vo,unit_vector_to_obstacle,obstacle_position,current_robot_position,obstacle_domain_radius,normalized_vector_to_goal):
        """
        Decide a direção da rotação para desviar do obstáculo.

        :param vr: tuple (robot_velocity_x, robot_velocity_y), vetor velocidade do robô
        :param vo: tuple (obstacle_velocity_x, obstacle_velocity_y), vetor velocidade do obstáculo
        :return: string, "anti-horário", "horário" ou "paralelo"
        """
        #Parâmetros
        #self.safe_distance        # Definir a variável tecnica
        sentido = "None"  # Inicialização segura
        avoidance_type_robot_from_obst="None"
        raio_barco = self.robot_domain_radius
        raio_obstaculo = obstacle_domain_radius
        dm = raio_barco + raio_obstaculo + self.safe_distance
        self.CR = dm + 3 * self.safe_distance
        distancia_obst = np.linalg.norm(np.array(obstacle_position) - np.array(current_robot_position))
        # Obter ângulo do obstáculo em relação à frente do robô.
        angle_to_obstacle = self.obstacle_angle(current_robot_position, vr, obstacle_position)
        if distancia_obst < self.CR:
            angle_of_robot_from_obstacle = self.angle_robot_from_obstacle(obstacle_position, vo, current_robot_position)
            if (0 <= angle_of_robot_from_obstacle <= 15) or (345 <= angle_of_robot_from_obstacle <= 360):
                avoidance_type_robot_from_obst="HeadsOn"
            elif 15 < angle_of_robot_from_obstacle <= 112.5:
                avoidance_type_robot_from_obst = "Crossing A"
            elif 247.5 <= angle_of_robot_from_obstacle < 345:  
                avoidance_type_robot_from_obst = "Crossing B"
            elif angle_of_robot_from_obstacle > 112.5 or angle_of_robot_from_obstacle <= 247.5:
                avoidance_type_robot_from_obst = "Overtaking"
            print(f"angle_of_robot_from_obstacle = {angle_of_robot_from_obstacle} e vo = {vo} e avoidance_type_robot_from_obst={avoidance_type_robot_from_obst}")
        # print(f"angle_of_robot_from_obstacle= {angle_of_robot_from_obstacle}")
        
        manobra_desvio = 0
        #verificar o tipo de encontro do barco no ponto de vista do obstaculo

        # Velocidade relativa (robô em relação ao obstáculo)
        relative_vel = np.array(vr) - np.array(vo)
        # Produto vetorial para determinar direita/esquerda entre vetores de velocidade
        cross_z = self.cross_product_2d(vr, vo)
        z_dot_product = vr[0] * vo[0] + vr[1] * vo[1]
        cross_relative_vo = self.cross_product_2d(relative_vel, vo)

        # Vetor posição relativo (do robô até o obstáculo)
        d = np.array(obstacle_position) - np.array(current_robot_position)
        
        # Produto vetorial entre velocidade relativa e posição relativa
        cross_relative = self.cross_product_2d(relative_vel, d)

        # Verificar risco de colisão
        risk_of_collision = self.check_collision_risk(current_robot_position, vr, obstacle_position, vo, dm)
        avoidance_type="nothing"
        # Determinar sentido (anti-horário ou horário) baseado em cross_relative
        # sentido = "horario" if cross_relative > 0 else "anti horario"
        if distancia_obst <= self.CR:
            print(f"Cross_z ={cross_z} cross_relative = {cross_relative}, z_dot_product = {z_dot_product}, cross_relative_vo = {cross_relative_vo}")
        # Verificar qual técnica usar
        if self.tecnica == 1:
            self.crossing_b=0
            # Classificar tipo de encontro se houver risco de colisão PARA MEU CÓDIGO
            if (0 <= angle_to_obstacle <= 15) or (345 <= angle_to_obstacle <= 360):
                avoidance_type = "HeadsOn"             
            elif 15 < angle_to_obstacle <= 112.5:
                avoidance_type = "Crossing A"
            elif 247.5 <= angle_to_obstacle < 345:  
                avoidance_type = "Crossing B"
            elif angle_to_obstacle > 112.5 or angle_to_obstacle <= 247.5:
                avoidance_type = "Overtaking"
            if risk_of_collision:
                manobra_desvio = 1
                if avoidance_type == "HeadsOn":
                    #verificar a posição do robô em relação ao obstáculo
                    if avoidance_type_robot_from_obst=="HeadsOn":
                        sentido="cross_relative"
                    if avoidance_type_robot_from_obst == "Crossing A":
                        sentido="cross_relative"
                    if avoidance_type_robot_from_obst == "Crossing B":
                        sentido="cross_relative"
                    if avoidance_type_robot_from_obst == "Overtaking":
                        sentido="goal"
                    # if cross_relative >= 0:
                    #     sentido="horario"
                    #     self.previous = sentido                        
                    # if cross_relative < 0:
                    #     sentido="anti horario"
                    #     self.previous = sentido                
                elif avoidance_type == "Crossing A":
                    #verificar a posição do robô em relação ao obstáculo
                    if avoidance_type_robot_from_obst=="HeadsOn":
                        sentido="cross_relative"
                    if avoidance_type_robot_from_obst == "Crossing A":
                        if z_dot_product > 0:
                            sentido="horario"
                        if z_dot_product <=0:
                            sentido="cross_relative"
                    if avoidance_type_robot_from_obst == "Crossing B":
                        sentido="horario"
                    if avoidance_type_robot_from_obst == "Overtaking":
                        sentido="goal"
                    # sentido="horario"
                    # if z_dot_product<0 and cross_relative<0:
                    #     sentido="anti horario"
                elif avoidance_type == "Crossing B":  
                    #verificar a posição do robô em relação ao obstáculo
                    if avoidance_type_robot_from_obst=="HeadsOn":
                        sentido="cross_relative"
                    if avoidance_type_robot_from_obst == "Crossing A":
                        sentido="anti horario"
                    if avoidance_type_robot_from_obst == "Crossing B":
                        sentido="cross_relative"
                    if avoidance_type_robot_from_obst == "Overtaking":
                        sentido="goal"
                    # if cross_relative >= 0:
                    #     sentido="horario"
                    # if cross_relative < 0:
                    #     sentido="anti horario"
                elif avoidance_type == "Overtaking":  
                    if np.linalg.norm(np.array(vr)) < np.linalg.norm(np.array(vo)):
                        sentido = "horario" if cross_relative < 0  else "anti horario"
                    else:
                        self.crossing_b=1
            else:
                manobra_desvio = 0

        elif self.tecnica == 2:
            # CÓDIGO DO ARTIGO LIU
            if risk_of_collision:
                manobra_desvio = 1
                # Situação de "Heads On" - Ambas manobram para a direita (starboard)
                if (0 <= angle_to_obstacle <= 5) or (355 <= angle_to_obstacle <= 360):
                    avoidance_type = "HeadsOn"
                    sentido = "horario"  # Manobra para a direita
                    self.crossing_b=0
                
                # Situação de "Crossing A" - Obstáculo à direita, ceder passagem
                elif 5 < angle_to_obstacle <= 112.5:
                    avoidance_type = "Crossing A"
                    sentido = "horario"  # Manobra para a direita (Give-Way)
                    self.crossing_b=0

                # Situação de "Crossing B" - Obstáculo à esquerda, manter o curso
                elif 247.5 <= angle_to_obstacle < 355:
                    avoidance_type = "Crossing B"
                    sentido = "horario"  # Stand-On Vessel
                    self.crossing_b=1

                # Situação de "Overtaking" - Ultrapassagem, manobra segura
                elif 112.5 < angle_to_obstacle <= 247.5:
                    avoidance_type = "Overtaking"
                    sentido = "manter curso"  # Ultrapassagem pela direita
                    if np.linalg.norm(np.array(vr)) < np.linalg.norm(np.array(vo)):
                        self.crossing_b=0
                    else:
                        self.crossing_b=1


                # Caso padrão em risco de colisão
                else:
                    avoidance_type = "No avoidance"
                    sentido = "manter curso"
                    self.crossing_b=0
            else:
                # Sem risco de colisão
                avoidance_type = "No avoidance"
                sentido = "manter curso"
                self.crossing_b=0
        elif self.tecnica == 3:
            # Não aplicamos lógica diferenciada de desvio.
            # Apenas consideramos que não há mudança no sentido (manter curso)
            # e não há classificação complexa (avoidance_type = "No avoidance")
            avoidance_type = "APF normal"
            sentido = "APF normal"


        if np.linalg.norm(d) < dm:
            sentido = "horario" if cross_relative < 0 else "anti horario"
            avoidance_type = "emergência"
        perp1 = np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]])
        perp2 = -np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]])
        cross_1 = 0
        cross_2 = 0
        if distancia_obst <= self.CR:
            cross_1 = self.cross_product_2d(vr, perp1)
            cross_2 = self.cross_product_2d(vr, perp2)
            if sentido=="goal":
                cross_g = self.cross_product_2d(vr,vo)
                if cross_g<0:
                    sentido = "anti horario"
                elif cross_g>=0:
                    sentido = "horario"
            if sentido=="cross_relative":
                if cross_relative >= 0:
                    sentido="horario"                       
                if cross_relative < 0:
                    sentido="anti horario"
            if avoidance_type == "emergência":         
                if cross_relative >= 0:
                    sentido="horario"                       
                if cross_relative < 0:
                    sentido="anti horario"
        if sentido == "anti horario":
            avoidance_vector = perp1 if cross_1 > 0 else perp2
        elif sentido == "horario":
            avoidance_vector = perp1 if cross_1 <= 0 else perp2
        else:
            sentido == "manter curso"
            avoidance_vector = np.array([0, 0])
        


        return avoidance_vector, sentido, avoidance_type, avoidance_type_robot_from_obst, manobra_desvio



        

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
        vector_to_goal = [a - b for a, b in zip(goal_position, current_robot_position)]
        count = 0 #Inutilizado
        distance_to_goal = np.linalg.norm(vector_to_goal) 
        normalized_vector_to_goal = vector_to_goal/distance_to_goal
        
        self.distance_to_goal_pub.publish(distance_to_goal)

        attractive_force = np.zeros_like(current_robot_position)
        repulsive_force = np.zeros_like(current_robot_position)
        # Força de atração padrão
        attractive_force = self.modified_attractive_force(current_robot_position, vector_to_goal, self.attraction_scaling_factor, distance_to_goal, normalized_vector_to_goal)
        # Publicar o estado de colisão
        collision_state = Bool() 
        collision_state.data  = False
        # rospy.loginfo(f"attractive_force : {attractive_force}")
        if self.tecnica == 3:
            factor_att = 1
            factor_rep = 10*distance_to_goal/2
            attractive_force = self.modified_attractive_force(current_robot_position, vector_to_goal, factor_att, distance_to_goal, normalized_vector_to_goal)
        
            # Força repulsiva padrão (uma soma das forças repulsivas de todos os obstáculos)
            repulsive_force = np.zeros_like(current_robot_position)
            
            for i,obstacle in enumerate(list_of_obstacle_positions):
                distance_to_obstacle = np.linalg.norm(np.array(obstacle) - np.array(current_robot_position))
                
                previous_collision_state = collision_state.data
                collision_state.data = distance_to_obstacle < (self.safety_margin_radius + self.robot_domain_radius)
                if previous_collision_state != collision_state.data:
                    self.collision_pub.publish(collision_state)

                if distance_to_obstacle < self.obstacle_influence_range:
                    # Vetor unitário do robô ao obstáculo
                    unit_vector_to_obstacle = (np.array(obstacle) - np.array(current_robot_position)) / distance_to_obstacle
                    # Força repulsiva padrão:
                    # Fr = η * (1/d - 1/ρ0)* (1/d²)* (unit_vector_to_obstacle)
                    repulsive_component = -factor_rep * ( (1 - distance_to_obstacle / self.obstacle_influence_range) ) * unit_vector_to_obstacle
                    repulsive_force += repulsive_component
                    rospy.loginfo(f"Obstaculo:{i}, repulsive_force : {repulsive_force}")
            
            total_force = attractive_force + repulsive_force

            # rospy.loginfo(f"total_force : {total_force}")
            return total_force, attractive_force, repulsive_force, 0, 0, 0

        else:
            
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



                distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius, sentido, avoidance_type,avoidance_type_robot_from_obst, manobra_desvio = self.iniciation(list_of_obstacle_positions[i], current_robot_position, current_robot_velocity, list_of_obstacle_velocities[i], list_of_obstacle_radii[i], self.safe_distance, self.obstacle_influence_range, self.robot_domain_radius,normalized_vector_to_goal,numero_do_obstaculo)
                


                self.tau = self.safety_margin_radius + self.robot_domain_radius

                previous_collision_state = collision_state.data
                collision_state.data = distance_to_obstacle < self.tau
                if previous_collision_state != collision_state.data:
                    self.collision_pub.publish(collision_state)
                self.collision_pub.publish(collision_state)


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
                    # custom_info_msg.action_type = "rotacao " + sentido
                    custom_info_msg.avoidance_type = "b="+avoidance_type + " e obst=" + avoidance_type_robot_from_obst + " rotação:" + sentido
                    custom_info_msg.CR = collision_avoidance_radius  #Raio de detecção
                    custom_info_msg.dm = center_to_center_safe_distance  #raio de emergência
                    custom_info_msg.angle_for_safe_distance = angle_for_safe_distance
                    custom_info_msg.angle_between_direction_and_velocity = angle_between_direction_and_velocity
                    print(f"boat= {avoidance_type} e obst= {avoidance_type_robot_from_obst} e sentodo = {sentido} e desvio = {manobra_desvio}")

                    tolerance = 1e-10         
                    if distance_to_obstacle <= collision_avoidance_radius and angle_between_direction_and_velocity < angle_for_safe_distance and center_to_center_safe_distance < distance_to_obstacle:
                        # rospy.loginfo("PASSOU AKI: ")
                        if np.linalg.norm(list_of_obstacle_velocities[i]) > tolerance:
                            # rospy.loginfo("Dinamico: ")
                            custom_info_msg.action_type = "Obstáculo Dinâmico"
                            Frd = self.calculate_Frd(distance_to_obstacle,center_to_center_safe_distance,self.obstacle_influence_range,angle_between_direction_and_velocity,relative_speed_vector,angle_for_safe_distance,angle_difference_for_safety,vector_to_obstacle,self.obstacle_scaling_factor_dynamic,obstacle_domain_radius,distance_to_goal,unit_vector_to_obstacle,perpendicular_unit_vector_to_obstacle,normalized_vector_to_goal,avoidance_type)
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

                    rospy.loginfo(f"Obstaculo numero {i+1}")
                    
                    # Publicação da mensagem em um tópico
                    self.custom_info_pub.publish(custom_info_msg)

                    Frd_total += Frd
                    Fre_total += Fre
                    Frs_total += Frs
            
            repulsive_force = Frd_total+Fre_total+Frs_total
            # rospy.loginfo(f"repulsive_force: { repulsive_force }  N")
            total_force = attractive_force + repulsive_force
            #self.plot_vector(current_robot_position[0], current_robot_position[1], total_force[0], total_force[1])
            #print("FORCA TOTAL = ",total_force)    
            return total_force, attractive_force, repulsive_force, Frd_total, Frs_total, Fre_total
