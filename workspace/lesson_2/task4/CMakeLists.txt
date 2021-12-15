cmake_minimum_required(VERSION 3.5)

project(useEigenGeometry)

find_package(Eigen3 REQUIRED)
find_package(glog 0.6.0 REQUIRED)

add_executable(${PROJECT_NAME} useEigenGeometry.cpp )

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen glog::glog)