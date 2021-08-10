cmake_minimum_required(VERSION 3.1)
project(perception_node)
enable_language(CUDA)



set(CMAKE_CXX_COMPILIER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -Wno-write-strings")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")
set(OpenCV_DIR "/home/cy/apps/opencv-4.0.0/build")
set(SRC_DIR src)
include_directories("./src")

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    image_transport
    cv_bridge
    OpenCV
)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# include_directories("/home/cy/apps/opencv-4.0.0/build")
# include_directories("/home/cy/apps/opencv-4.0.0/modules/dnn/include")


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS roscpp rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
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

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ros_boilerplate ros_boilerplate_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY ${SRC_DIR}/include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



