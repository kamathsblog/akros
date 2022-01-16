cmake_minimum_required(VERSION 3.0.2)
project(akros_test)

# set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  akros_msgs
  nodelet
  rospy
  camera_info_manager
  depthai_ros_msgs
  cv_bridge
  vision_msgs
  depthai_bridge
  message_filters
  nodelet
)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()
find_package(depthai REQUIRED)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs depthai_ros_msgs camera_info_manager depthai_bridge vision_msgs cv_bridge message_filters
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

macro(dai_add_nodelet nodelet_name nodelet_src)
    add_library("${nodelet_name}" "${nodelet_src}")
    
    add_dependencies("${nodelet_name}"
      ${catkin_EXPORTED_TARGETS} 
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
    
    target_link_libraries("${nodelet_name}" 
      ${catkin_LIBRARIES}
      depthai::core
      opencv_imgproc
      opencv_highgui
    )
    
    target_link_options("${nodelet_name}" PRIVATE
      -Wl,--no-undefined
    )
endmacro() 

macro(dai_add_node node_name node_src)
    add_executable("${node_name}" "${node_src}")

    target_link_libraries("${node_name}" 
      ${catkin_LIBRARIES}
      depthai::core
      opencv_imgproc
      opencv_highgui
    )   
endmacro() 

dai_add_nodelet(rgbd_inertial_nodelet src/oakd_rgbd_inertial_nodelet.cpp)
dai_add_node(oakd_rgbd_imu_node src/oakd_rgbd_imu_node.cpp)
dai_add_node(oakd_imu_node src/oakd_imu_node.cpp)

