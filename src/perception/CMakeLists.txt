cmake_minimum_required(VERSION 3.1)
project(perception_node)
enable_language(CUDA)

set(CMAKE_CXX_COMPILIER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -Wno-write-strings")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")
set(OpenCV_DIR "/home/cy/apps/opencv-4.0.0/build")
set(SRC_DIR src)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    image_transport
    cv_bridge
    OpenCV
)

include_directories(
    ${SRC_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#detector lib (cuda)
file(GLOB_RECURSE sources src/modules/*.hpp src/modules/*.cpp src/modules/*.h src/modules/*.cu src/extra/*.h)
add_library(detector SHARED ${sources})
target_include_directories(detector PRIVATE src/extra/ src/modules/ ${OpenCV_INCLUDE_DIRS} ${CUDA_TOOLKIT_ROOT_DIR}/include)
target_link_libraries(detector nvinfer nvinfer_plugin nvparsers "stdc++fs")

## Declare a C++ executable
add_executable(perception_node ${SRC_DIR}/perception_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(perception_node PRIVATE src/modules/ src/extra/)
## Specify libraries to link a library or executable target against
target_link_libraries(perception_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    detector
    opencv_dnn
    )



