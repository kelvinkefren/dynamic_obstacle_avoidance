cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
)

catkin_package()

# Instala os scripts Python do diretório src
install(PROGRAMS
  src/obstacle_avoidance.py
  src/migbot_controller.py
  src/run_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instala o arquivo de lançamento
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Instala o arquivo de configuração (se necessário)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
