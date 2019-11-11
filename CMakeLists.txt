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
  message_generation
  tf
)

## Generate services in the 'srv' folder
add_service_files(FILES changeBaseString.srv)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs)

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

add_dependencies(talker beginner_tutorials_generate_messages_cpp)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)


## Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
  talkertest
  test/talkertest.launch
  test/main.cpp
  test/talkertest.cpp
  )
  target_link_libraries(
  talkertest
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  talker
  talkertest
  ${catkin_EXPORTED_TARGETS}
  )
endif()
