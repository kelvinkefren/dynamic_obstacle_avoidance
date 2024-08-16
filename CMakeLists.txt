cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_avoidance)

## Find catkin macros and libraries
## Also find other catkin packages that are needed
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  message_generation  # Needed for custom message generation
)


## Declare ROS messages
add_message_files(
  FILES
  RobotState.msg
  ObstacleState.msg
  ObstacleArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
install(PROGRAMS
  src/obstacle_avoidance.py
  src/migbot_controller.py
  src/run_controller.py
  src/forceCalculationApfModified.py
  src/migbotAvoidance.py
  scripts/scenario_controller.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install the configuration files (if any)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)