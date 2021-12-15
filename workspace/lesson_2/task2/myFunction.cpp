#include "myFunction.h"

MyFunction::MyFunction() {
    A = Eigen::MatrixXd::Random(DIMENSION, DIMENSION);
    b = Eigen::MatrixXd::Random(DIMENSION, 1);
}

MyFunction::MyFunction(Eigen::Matrix<double, DIMENSION, DIMENSION> b_) : b(b_) {
    A = Eigen::MatrixXd::Random(DIMENSION, DIMENSION);
}

void MyFunction::solve_by_Inverse(Eigen::Matrix<double, DIMENSION, 1> &x) {
    x = A.inverse() * b;
}

void MyFunction::solve_by_QR(Eigen::Matrix<double, DIMENSION, 1> &x) {
    x = A.colPivHouseholderQr().solve(b);
}

void MyFunction::solve_by_QR_HouseHolder(Eigen::Matrix<double, DIMENSION, 1> &x) {
    x = A.householderQr().solve(b);
}

void MyFunction::solve_by_Cholesky(Eigen::Matrix<double, DIMENSION, 1> &x) {
    x = A.ldlt().solve(b);
}