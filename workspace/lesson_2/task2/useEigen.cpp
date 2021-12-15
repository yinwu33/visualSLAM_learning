#include "myFunction.h"
#include <glog/logging.h>
#include <chrono>
#include <string>


void showResult(const std::chrono::nanoseconds &duration, const Eigen::Matrix<double, DIMENSION, 1> &x, std::string name) {
    LOG(INFO) << "\n-------------------------\n"
              << "# Test Name: " << name << "\n"
              << "---Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms\n"
              << "---Result(norm of x): " << x.norm() << "\n"
              << "=========================\n\n";
}


int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;

    LOG(INFO) << "Matrix: " << DIMENSION;

    Eigen::Matrix<double, DIMENSION, 1> x;

    MyFunction my_function;

    auto time_start = std::chrono::high_resolution_clock::now();
    my_function.solve_by_QR(x);
    auto time_stop = std::chrono::high_resolution_clock::now();
    auto duration = time_stop - time_start;
    showResult(duration, x, "QR");

    time_start = std::chrono::high_resolution_clock::now();
    my_function.solve_by_Cholesky(x);
    time_stop = std::chrono::high_resolution_clock::now();
    duration = time_stop - time_start;
    showResult(duration, x, "Cholesky");

    time_start = std::chrono::high_resolution_clock::now();
    my_function.solve_by_Inverse(x);
    time_stop = std::chrono::high_resolution_clock::now();
    duration = time_stop - time_start;
    showResult(duration, x, "Inverse");

    time_start = std::chrono::high_resolution_clock::now();
    my_function.solve_by_QR_HouseHolder(x);
    time_stop = std::chrono::high_resolution_clock::now();
    duration = time_stop - time_start;
    showResult(duration, x, "QR HousHolder");

    return 0;
}