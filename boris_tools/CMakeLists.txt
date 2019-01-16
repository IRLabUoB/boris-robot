cmake_minimum_required(VERSION 2.8.3)
project(boris_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_python_setup()
