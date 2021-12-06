add_executable(sayhello useHello.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(HELLO REQUIRED)

if(HELLO_FOUND)
    include_directories(${HELLO_INCLUDE_DIR})
    target_link_libraries(sayhello ${HELLO_LIBRARY})
else(HELLO_FOUND)
    message(FATAL_ERROR "HELLO lib not found")
endif(HELLO_FOUND)

target_link_libraries(sayhello glog::glog)

# test
enable_testing()

add_executable(add_test myAdd_test.cpp myAdd.cpp)
target_link_libraries(add_test gtest_main)
target_link_libraries(add_test gtest)
target_link_libraries(add_test pthread)
