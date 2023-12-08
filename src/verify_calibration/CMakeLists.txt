cmake_minimum_required(VERSION 3.0.2)
project(verify_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  dynamic_reconfigure
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

add_service_files(
  FILES
  CallPickPlaceDemo.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/SetTFOffset.cfg
# )


catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs dynamic_reconfigure message_runtime
  DEPENDS system_lib EIGEN3
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

# Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/verify_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)