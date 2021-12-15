cmake_minimum_required(VERSION 3.5)

project(task3)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(task3 fisheye.cpp)
target_link_libraries(task3 ${OpenCV_LIBS})