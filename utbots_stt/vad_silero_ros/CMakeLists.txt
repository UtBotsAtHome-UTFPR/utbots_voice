cmake_minimum_required(VERSION 3.0.2)
project(vad_silero_ros)

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

catkin_package(
  INCLUDE_DIRS include
)

## Specify additional locations of header files (your package locations should be listed before other locations)
include_directories(include/ ${catkin_INCLUDE_DIRS})

# ONNX library
message(STATUS "*** Adding ONNX lib ***")
add_library(lib_onx SHARED IMPORTED GLOBAL)
set_target_properties(lib_onx PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libonnxruntime.so)

# miniaudio library
message(STATUS "*** Adding miniaudio lib ***")
add_compile_definitions(MINIAUDIO_IMPLEMENTATION)
add_library(lib_miniaudio "src/miniaudio.c")

# VAD node
message(STATUS "*** VAD node ***")
set(TARGET vad_node)
add_executable(${TARGET} src/vad_node.cpp)
target_link_libraries(${TARGET} ${catkin_LIBRARIES} lib_onx ${CMAKE_DL_LIBS} lib_miniaudio)
