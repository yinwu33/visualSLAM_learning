cmake_minimum_required(VERSION 3.8)

project(task2)

# Eigen3
include_directories("/usr/include/eigen3")

# Sohpus
find_package(Sophus REQUIRED)
include_directories(${SophusINCLUDE_DIRS})

# fmt
find_package(fmt)

add_executable(task2 task2.cpp)
target_link_libraries(task2 ${Sophus_LIBRARIES} fmt::fmt)