set(HELLO_LIB_SRC hello.cpp)
set(HELLO_EXE_SRC useHello.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(hello ${HELLO_LIB_SRC})

install(FILES ${CMAKE_SOURCE_DIR}/include/hello.h DESTINATION /usr/local/include)
install(TARGETS hello LIBRARY DESTINATION /usr/local/lib CONFIGURATIONS Release)