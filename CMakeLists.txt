cmake_minimum_required(VERSION 2.8.3)
project(reach_driver)
add_definitions(-DERB_DEBUGGING)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(BUILD_WITH_ROS 0)
if(EXISTS ${CATKIN_ENV})
  set(BUILD_WITH_ROS 1)
  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
  )
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES reach_driver
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs
  #  DEPENDS system_lib
  )
else()
  find_package(catkin REQUIRED)
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES reach_driver
  #  DEPENDS system_lib
  )
endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_parser
  src/reach_parser.cpp
)

if(BUILD_WITH_ROS)
  ## Declare a ROS C++ executable

  add_executable(${PROJECT_NAME}_node src/reach_driver_node.cpp)

  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES} ${PROJECT_NAME}_parser
  )
endif()