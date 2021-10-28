cmake_minimum_required(VERSION 3.5)
project(akros_behaviors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    actionlib 
    sensor_msgs
    move_base_msgs 
    behaviortree_cpp_v3
    rospy)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
## catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS 
   DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
set(NAVIGATION_SOURCES
    src/behaviors/navigation_behaviors.cpp 
)

add_executable(
    waypoint_sequence_node src/waypoint_sequence.cpp ${NAVIGATION_SOURCES}
)
target_link_libraries(waypoint_sequence_node
    ${catkin_LIBRARIES} yaml-cpp
)
target_compile_definitions(waypoint_sequence_node PRIVATE BT_XML_PATH="${PROJECT_SOURCE_DIR}/trees/waypoint_sequence_bt.xml")

catkin_install_python(PROGRAMS
   scripts/waypoint_sequence_store.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
