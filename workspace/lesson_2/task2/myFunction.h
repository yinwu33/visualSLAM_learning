#ifndef MY_MATRIX_H
#define MY_MATRIX_H

#include <Eigen/Core>
#include <Eigen/Dense>


#define DIMENSION 150

class MyFunction {
public:
    MyFunction();
    MyFunction(Eigen::Matrix<double, DIMENSION, DIMENSION> b_);

    void solve_by_Inverse(Eigen::Matrix<double, DIMENSION, 1> &x);
    void solve_by_QR(Eigen::Matrix<double, DIMENSION, 1> &x);
    void solve_by_QR_HouseHolder(Eigen::Matrix<double, DIMENSION, 1> &x);
    void solve_by_Cholesky(Eigen::Matrix<double, DIMENSION, 1> &x);

private:
    // Ax = b
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;
};


#endif