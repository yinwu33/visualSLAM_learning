cmake_minimum_required(VERSION 3.5)

project(task4)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(task4 GN-BA.cpp)
target_link_libraries(task4 Sophus::Sophus Eigen3::Eigen)
