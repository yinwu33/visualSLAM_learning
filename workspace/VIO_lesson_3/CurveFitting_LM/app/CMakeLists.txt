add_executable(testCurveFitting CurveFitting.cpp)
target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)

add_executable(myCurveFitting my_curve_fitting.cpp)
target_link_libraries(myCurveFitting ${PROJECT_NAME}_backend)