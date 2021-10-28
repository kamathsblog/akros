cmake_minimum_required(VERSION 3.0.2)
project(akros_bringup)

# set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  vision_msgs
  akros_msgs
  depthai_bridge
  depthai_ros_msgs
  image_transport
  nodelet
  rospy
)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()
#find_package(depthai REQUIRED)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs depthai_bridge vision_msgs cv_bridge depthai_ros_msgs
 DEPENDS
 OpenCV
 )

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

macro(dai_add_node node_name node_src)
    add_executable("${node_name}" "${node_src}")

    target_link_libraries("${node_name}" 
      ${catkin_LIBRARIES}
      depthai::core
      opencv_imgproc
      opencv_highgui
    )
    
endmacro()

## DEPTHAI DETECTION CONVERTERS
## dai_add_node(rgb_converter_node src/mono_detections.cpp)
## dai_add_node(rgbd_converter_node src/spatial_detections.cpp)


catkin_install_python(PROGRAMS
   scripts/twist_mixer.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

