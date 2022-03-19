cmake_minimum_required(VERSION 3.0.2)
project(akros_teleop)

# set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  ds4_driver
  std_msgs
  nodelet
  rospy
)


catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs ds4_driver
 DEPENDS
 )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/ds4_feedback_node.py
   scripts/ds4_twist_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

