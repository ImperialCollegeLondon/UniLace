cmake_minimum_required(VERSION 2.8.12)

project(uni_lace)

find_package(catkin REQUIRED COMPONENTS 
  roscpp std_msgs 
  sensor_msgs 
  control_msgs 
  trajectory_msgs 
  simple_message 
  actionlib_msgs 
  actionlib 
  urdf)

catkin_python_setup()

catkin_package()