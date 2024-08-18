#!/usr/bin/env python3

import numpy as np
import math
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dynamic_obstacle_avoidance.msg import CustomInfo  

class ObstacleAvoidance:

    def __init__(self):
        # Initialize parameters from ROS
        self.attraction_scaling_factor = rospy.get_param('~attraction_scaling_factor', 6000)
        self.obstacle_scaling_factor_dynamic = rospy.get_param('~obstacle_scaling_factor_dynamic', 20000)
        self.obstacle_scaling_factor_static = rospy.get_param('~obstacle_scaling_factor_static', 3000000)
        self.scaling_factor_emergency = rospy.get_param('~scaling_factor_emergency', 40000)
        self.safety_margin_radius = rospy.get_param('~safety_margin_radius', 0.3)
        self.robot_domain_radius = rospy.get_param('~robot_domain_radius', 0.5)
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)
        self.obstacle_influence_range = rospy.get_param('~obstacle_influence_range', 5)
        # self.distance_to_goal = rospy.get_param('~distance_to_goal', 0.1)

        self.custom_info_pub = rospy.Publisher('/obstacle_avoidance/custom_info', CustomInfo, queue_size=10)
    


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
		#vector_to_obstacle = obstacle_position - current_robot_position
        if distance_to_obstacle >= center_to_center_safe_distance:
            angle_for_safe_distance2 = np.arctan2(center_to_center_safe_distance, np.sqrt(distance_to_obstacle**2 - center_to_center_safe_distance**2))
            angle_for_safe_distance = np.degrees(angle_for_safe_distance2)  #θm​=arctan(ρ2(pos​,pts​)−dm2​​dm​  [3]​) 
        else:
            angle_for_safe_distance = 0
        
        relative_speed_vector = np.array(current_robot_velocity) - np.array(obstacle_velocitiy)
        #relative_speed_vector = current_robot_velocity - obstacle_velocitiy
        #print("VOS = ",current_robot_velocity,"VTs = ",obstacle_velocitiy)
        
        #angle_between_direction_and_velocity: The angle between the vector vector_to_obstacle aobstacle_scaling_factor_dynamic the relative velocity vector relative_speed_vector. 
        extra = 1e-8  # You can adjust this value as scaling_factor_emergencyeded
        angle_between_direction_and_velocity2 = np.arccos(np.dot(vector_to_obstacle, relative_speed_vector) / (np.linalg.norm(vector_to_obstacle) * np.linalg.norm(relative_speed_vector) + extra))
        angle_between_direction_and_velocity = np.degrees(angle_between_direction_and_velocity2)
        #angle_between_direction_and_velocity = adjust_angle(angle_between_direction_and_velocity)
        #print("angulo = ",math.degrees(angle_between_direction_and_velocity),"angulo_obst = ",math.degrees(angle_for_safe_distance) )
        # Pegar o menor ângulo
        if angle_between_direction_and_velocity > 90:
            angle_between_direction_and_velocity = 180 - angle_between_direction_and_velocity

 
        #unit_vector_to_obstacle is a unit vector pointing from the current position to the obstacle

        #unit_vector_to_obstacle = (obstacle_position - current_robot_position) / distance_to_bstacle
        unit_vector_to_obstacle = (np.array(obstacle_position) - np.array(current_robot_position)) / distance_to_obstacle
        angle_difference_for_safety = angle_for_safe_distance-angle_between_direction_and_velocity
        angulacao = np.degrees(np.arccos(np.dot(unit_vector_to_obstacle, normalized_vector_to_goal)/(np.linalg.norm(unit_vector_to_obstacle) * np.linalg.norm(normalized_vector_to_goal))))
        histerese = 0
        if angulacao < 5 or angulacao > 355:
            histerese = 0.1
        #perpendicular_unit_vector_to_obstacle = comparar_vetores(vector_to_obstacle, relative_speed_vector, obstacle_velocitiy,unit_vector_to_obstacle,histerese)
        perpendicular_unit_vector_to_obstacle = self.determine_avoidance_direction(current_robot_velocity,obstacle_velocitiy,unit_vector_to_obstacle)
        #perpendicular_unit_vector_to_obstacle = determiscaling_factor_emergency_side(vector_to_obstacle, relative_speed_vector,unit_vector_to_obstacle)
        obstacle_domain_radius = obstacle_radii
        
        return distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius

    def determine_avoidance_direction(self,vr, vo,unit_vector_to_obstacle):
        """
        Decide a direção da rotação para desviar do obstáculo.

        :param vr: tuple (robot_velocity_x, robot_velocity_y), vetor velocidade do robô
        :param vo: tuple (obstacle_velocity_x, obstacle_velocity_y), vetor velocidade do obstáculo
        :return: string, "anti-horário", "horário" ou "paralelo"
        """
        robot_velocity_x, robot_velocity_y = vr
        obstacle_velocity_x, obstacle_velocity_y = vo

        # Calcula a componente z do produto vetorial
        cross_product_z_component = robot_velocity_x * obstacle_velocity_y - robot_velocity_y * obstacle_velocity_x
        
        # Calcula o produto escalar e as magnitudes dos vetores de velocidade
        velocity_dot_product = robot_velocity_x * obstacle_velocity_x + robot_velocity_y * obstacle_velocity_y
        robot_velocity_magnitude = math.sqrt(robot_velocity_x ** 2 + robot_velocity_y ** 2)
        obstacle_velocity_magnitude = math.sqrt(obstacle_velocity_x ** 2 + obstacle_velocity_y ** 2)

        # Calcula o ângulo entre os vetores de velocidade em graus
		
        if robot_velocity_magnitude * obstacle_velocity_magnitude == 0:
            angle_between_velocities_rad = 0
        else:
            angle_between_velocities_rad = math.acos(velocity_dot_product / (robot_velocity_magnitude * obstacle_velocity_magnitude))        
        angle_between_velocities_deg = math.degrees(angle_between_velocities_rad)

        # Se o ângulo estiver entre 175 e 185 graus, escolha uma direção de rotação padrão (anti-horário, por exemplo)
        if 165 <= angle_between_velocities_deg <= 195:
            return -np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]])  #"anti-horário"
        elif cross_product_z_component > 0:
            return -np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]])  #"anti-horário"
        elif cross_product_z_component < 0:
            return -np.array([-unit_vector_to_obstacle[1], unit_vector_to_obstacle[0]]) #"horário"
        else:
            return -np.array([unit_vector_to_obstacle[1], -unit_vector_to_obstacle[0]]) #"paralelo"  


        # caso não tenha retornado ainda, não há comparação possível
        #print("Não há comparação possível")
        return np.array([-unit_vector_to_obstacle[1], unit_vector_to_obstacle[0]])

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

            distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius = self.iniciation(list_of_obstacle_positions[i], current_robot_position, current_robot_velocity, list_of_obstacle_velocities[i], list_of_obstacle_radii[i], self.safe_distance, self.obstacle_influence_range, self.robot_domain_radius,normalized_vector_to_goal,numero_do_obstaculo)

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

            # Publicação da mensagem em um tópico
            self.custom_info_pub.publish(custom_info_msg)

            tolerance = 1e-10         
            if distance_to_obstacle <= collision_avoidance_radius and angle_between_direction_and_velocity < angle_for_safe_distance and center_to_center_safe_distance < distance_to_obstacle:
                if np.linalg.norm(list_of_obstacle_velocities[i]) > tolerance:
                    rospy.loginfo("Dinamico: ")
                    Frd = self.calculate_Frd(distance_to_obstacle,center_to_center_safe_distance,self.obstacle_influence_range,angle_between_direction_and_velocity,relative_speed_vector,angle_for_safe_distance,angle_difference_for_safety,vector_to_obstacle,self.obstacle_scaling_factor_dynamic,obstacle_domain_radius,distance_to_goal,unit_vector_to_obstacle,perpendicular_unit_vector_to_obstacle,normalized_vector_to_goal)
                else:
                    rospy.loginfo("Estatico: ")
                    Frs = self.calculate_Frs(distance_to_obstacle, self.safety_margin_radius, distance_to_goal, self.obstacle_influence_range, self.obstacle_scaling_factor_static, obstacle_domain_radius, unit_vector_to_obstacle, normalized_vector_to_goal)
            else:
                if distance_to_obstacle < center_to_center_safe_distance :
                    rospy.loginfo("Emergencia: ")
                    Fre = self.calculate_Fre(distance_to_obstacle, self.safety_margin_radius, center_to_center_safe_distance, distance_to_goal, self.scaling_factor_emergency, obstacle_domain_radius, unit_vector_to_obstacle, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal,perpendicular_unit_vector_to_obstacle)
                else:
                    None
            rospy.loginfo(f"Obstaculo numero {i+1}")
            rospy.loginfo(f"Frd = {Frd}")#print("distance_to_obstacle =",distance_to_obstacle,"< collision_avoidance_radius =",collision_avoidance_radius," dinamic --------------  Frd = ",Frd)
            rospy.loginfo(f"Frs = {Frs}")#print("distance_to_obstacle =",distance_to_obstacle,"< collision_avoidance_radius =",collision_avoidance_radius," static --------------  Frs = ",Frs)
            rospy.loginfo(f"Fre = {Fre}")#print("distance_to_obstacle =",distance_to_obstacle,"< center_to_center_safe_distance =", center_to_center_safe_distance," --------------  Fre = ",Fre)
               
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
