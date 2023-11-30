cmake_minimum_required(VERSION 3.1.3)
project(fr5_real_moveit)

set(CMAKE_BUILD_TYPE debug) #debug模式，程序不会被优化，速度非常慢
# set(CMAKE_BUILD_TYPE "RelWithDebInfo") #debug模式，程序不会被优化，速度非常慢
# set(CMAKE_BUILD_TYPE "Release") 

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ecl_build
  ecl_devices
  ecl_geometry
  ecl_time
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  serial
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
)


# ## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   MotorStatus.msg
#   SingleJointControl.msg
#   JointLocation.msg
# )

## Generate services in the 'srv' folder
#  add_service_files(
#      FILES
#      CommandPose.srv
#      CommandJoint.srv
#      CallVersion.srv
#  )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   actionlib_msgs
#   geometry_msgs
#   sensor_msgs
# )

catkin_package(
	#  LIBRARIES xarm_driver
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

###########
## Build ##
###########


include_directories(
  include
  /usr/local/include/frcobot_hw
  /usr/local/include/xmlrpc-c
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
  ${catkin_LIB_DIRS}
)

add_executable(fr_driver src/fr_driver.cpp)
target_link_libraries(fr_driver  ${catkin_LIBRARIES}
  robotAPI
  xmlrpc
  xmlrpc_client
  xmlrpc_util
  xmlrpc_xmlparse
  xmlrpc_xmltok
  z
)

install(TARGETS fr_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(fr5_joint_states_pub src/fr5_joint_states_pub.cpp)
target_link_libraries(fr5_joint_states_pub  ${catkin_LIBRARIES}
  robotAPI
)

install(TARGETS fr5_joint_states_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(fr5_query_state_and_pub src/fr5_query_state_and_pub.cpp)
target_link_libraries(fr5_query_state_and_pub  ${catkin_LIBRARIES}
  robotAPI
  xmlrpc
  xmlrpc_client
  xmlrpc_util
  xmlrpc_xmlparse
  xmlrpc_xmltok
  z
)

install(TARGETS fr5_query_state_and_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
