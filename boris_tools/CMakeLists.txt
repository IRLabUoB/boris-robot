cmake_minimum_required(VERSION 2.8.3)
project(boris_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/BorisPositionFFJointTrajectoryActionServer.cfg
  
)

# add_dependencies(${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(boris_tools_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


catkin_package(
  # CATKIN_DEPENDS
  # rospy
  # actionlib
  # sensor_msgs
  # std_msgs
  # control_msgs
  # trajectory_msgs
  # dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)