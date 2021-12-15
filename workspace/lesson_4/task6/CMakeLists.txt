cmake_minimum_required(VERSION 3.5)

project(task6)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} gaussnewton.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen)