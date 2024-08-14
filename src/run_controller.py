#!/usr/bin/env python3

import rospy
from migbot_controller import MigbotController, Migbot, Obstacle
from obstacle_avoidance import ObstacleAvoidance

def main():
    rospy.init_node('dynamic_obstacle_avoidance_node')

    # Carregando parâmetros do YAML
    params = rospy.get_param('/obstacle_avoidance')

    # Inicializando a lógica de desvio de obstáculos
    obstacle_avoidance = ObstacleAvoidance(params)

    # Inicializando o robô e obstáculos
    migbot = Migbot()
    obstacle_names = ['vegetation1_buoy', 'vegetation2_buoy', 'vegetation3_buoy', 'branche1_buoy', 'branche2_buoy']
    obstacle_radii = [1, 1, 1, 9, 7]
    obstacles = [Obstacle(name, radius) for name, radius in zip(obstacle_names, obstacle_radii)]

    # Inicializando o controlador do robô
    controller = MigbotController(obstacle_avoidance, migbot, obstacles)

    # Executando o loop principal
    controller.run()

if __name__ == '__main__':
    main()
