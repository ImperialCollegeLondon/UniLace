cmake_minimum_required(VERSION 2.8.12)

project(robot_driver_unity)

find_package(catkin REQUIRED COMPONENTS 
  roscpp std_msgs 
  sensor_msgs 
  control_msgs 
  trajectory_msgs 
  simple_message 
  actionlib_msgs 
  actionlib 
  urdf)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/joint_trajectory_action.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)