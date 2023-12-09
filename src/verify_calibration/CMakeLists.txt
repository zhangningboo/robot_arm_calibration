cmake_minimum_required(VERSION 3.0.2)
project(verify_calibration)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
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

find_package(OpenCV)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  CallPickPlaceDemo.srv
)

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/HSVColorDetection.cfg
# cfg/SetTFOffset.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES verify_calibration
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs dynamic_reconfigure message_runtime
  DEPENDS system_lib EIGEN3
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(moveit_pose_demo src/moveit_pose_demo.cpp)
target_link_libraries(moveit_pose_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(moveit_pose_demo ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(verify_calibration_server src/verify_calibration_server.cpp)
target_link_libraries(verify_calibration_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(verify_calibration_server ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



catkin_install_python(PROGRAMS
  scripts/verify_calibration_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
