cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

###################################
## Declare catkin package ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(listener src/listener.cpp)
add_executable(talker src/talker.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(listener
   ${catkin_LIBRARIES}
 )
 target_link_libraries(talker
   ${catkin_LIBRARIES}
 )

