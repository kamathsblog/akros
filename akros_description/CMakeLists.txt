cmake_minimum_required(VERSION 3.0.2)
project(akros_description)

# set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs visualization_msgs)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs visualization_msgs
 DEPENDS
 )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/mesh_publisher.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)