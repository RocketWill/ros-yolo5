cmake_minimum_required(VERSION 2.8.3)
project(camera_driver)

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)
use_cxx11()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    image_transport
    cv_bridge
)

include_directories("/home/cy/apps/opencv-4.0.0/build")
find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp rospy
)

include_directories(
    src/include
    ${catkin_INCLUDE_DIRS}
)

set(SRC_DIR src)
add_executable(camera_driver ${SRC_DIR}/camera_driver.cpp)
add_dependencies(camera_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_driver
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

install(DIRECTORY ${SRC_DIR}/include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    )




