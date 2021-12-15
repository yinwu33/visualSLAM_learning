cmake_minimum_required(VERSION 3.5)

project(task3)


find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

add_executable(task3 E2Rt.cpp)
target_link_libraries(task3 Eigen3::Eigen Sophus::Sophus)
