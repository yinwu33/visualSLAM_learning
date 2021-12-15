cmake_minimum_required(VERSION 3.5)
project(draw)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} draw_trajectory.cpp)
target_link_libraries(${PROJECT_NAME} Sophus::Sophus Eigen3::Eigen ${Pangolin_LIBRARIES})