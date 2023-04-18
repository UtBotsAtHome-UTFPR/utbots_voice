cmake_minimum_required(VERSION 3.0.2)
project(whisper_cpp_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  roscpp
  rospy
  std_msgs
  roslib
)

catkin_package()

add_subdirectory(whisper.cpp)

## Specify additional locations of header files (your package locations should be listed before other locations)
include_directories(whisper.cpp/examples/ ${catkin_INCLUDE_DIRS})

# Whisper node
message(STATUS "*** Whisper node ***")
set(TARGET whisper_node)
add_executable(${TARGET} src/whisper_node.cpp)
target_link_libraries(${TARGET} PRIVATE whisper ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})