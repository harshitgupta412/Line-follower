cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
find_package(gazebo REQUIRED)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${GAZEBO_INCLUDE_DIRS}  ${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(controller SHARED controller.cc)

target_link_libraries(controller ${GAZEBO_libraries})
target_link_libraries(controller ${OpenCV_LIBS})

