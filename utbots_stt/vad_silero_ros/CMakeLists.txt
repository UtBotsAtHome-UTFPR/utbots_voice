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

# # RNNoise library
message(STATUS "*** Adding RNNoise lib ***")
#add_library(librnnoise SHARED IMPORTED GLOBAL)
#set_target_properties(librnnoise PROPERTIES IMPORTED_LOCATION ${RNNOISE_DIR}/.lib/librnnoise.so)
#include(src/cmake/rnnoise.cmake)

include(ExternalProject)

set(RNNOISE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rnnoise)
# set(RNNOISE_BIN ${CMAKE_CURRENT_BINARY_DIR}/rnnoise)
set(RNNOISE_BIN  ${CMAKE_CURRENT_BINARY_DIR}/rnnoise)

set(RNNOISE_STATIC_LIB ${RNNOISE_BIN}/lib/librnnoise.a)
#set(RNNOISE_STATIC_LIB ${RNNOISE_DIR}/.libs/librnnoise.so)

set(RNNOISE_INCLUDES ${RNNOISE_BIN}/include)

file(MAKE_DIRECTORY ${RNNOISE_INCLUDES})

ExternalProject_Add(
    librnnoise
    # PREFIX ${RNNOISE_BIN}
    SOURCE_DIR ${RNNOISE_DIR}
    DOWNLOAD_COMMAND cd ${RNNOISE_DIR} && git clean -dfX && ${RNNOISE_DIR}/autogen.sh
    CONFIGURE_COMMAND ${RNNOISE_DIR}/configure --srcdir=${RNNOISE_DIR} --prefix=${RNNOISE_BIN} --enable-static=yes  --enable-x86-rtcd --disable-shared
    BUILD_COMMAND cd ${RNNOISE_DIR} && make 
    BUILD_IN_SOURCE 1 
    #INSTALL_COMMAND # make install
    # BUILD_BYPRODUCTS ${RNNOISE_STATIC_LIB}
)
add_library(rnnoise STATIC IMPORTED GLOBAL)
add_dependencies(rnnoise librnnoise)
set_target_properties(rnnoise PROPERTIES IMPORTED_LOCATION ${RNNOISE_STATIC_LIB})
set_target_properties(rnnoise PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${RNNOISE_INCLUDES})


# VAD node
message(STATUS "*** VAD node ***")
set(TARGET vad_node)
add_executable(${TARGET} src/vad_node.cpp)
target_link_libraries(${TARGET} ${catkin_LIBRARIES} lib_onx ${CMAKE_DL_LIBS} lib_miniaudio rnnoise)



