cmake_minimum_required(VERSION 3.5)

project(sayhello)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(glog REQUIRED)

add_subdirectory(src)