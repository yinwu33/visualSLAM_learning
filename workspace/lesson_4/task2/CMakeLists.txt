cmake_minimum_required(VERSION 3.5)

project(task2)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(task2 undistort_image.cpp)
target_link_libraries(task2 ${OpenCV_LIBS})