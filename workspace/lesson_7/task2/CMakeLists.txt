cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 14)

project(task2)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

find_package(G2O REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(fmt REQUIRED)

find_package (glog 0.6.0 REQUIRED)

find_package(pybind11)

find_package(Pangolin REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} task2.cpp
                                point.hpp
                                cameraParameters.hpp
                                measurement.hpp
                                vertexCamera.hpp
                                vertexPosition.hpp
                                )

target_link_libraries(${PROJECT_NAME} 
${OpenCV_LIBS}  
g2o_core g2o_stuff g2o_types_slam3d 
Eigen3::Eigen 
fmt::fmt
glog::glog ${Pangolin_LIBRARIES})

