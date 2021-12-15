//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Vector3d sigma_vector = svd.singularValues();
    double sigma = (sigma_vector[0] + sigma_vector[1])/2;
    Eigen::Matrix3d sigma_matrix;
    sigma_matrix << sigma, 0, 0,
                    0, sigma, 0,
                    0, 0, 0;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;

    Matrix3d R_z_pos = AngleAxisd(M_PI_2, Vector3d{0, 0, 1}).toRotationMatrix();
    Matrix3d R_z_neg = AngleAxisd(-M_PI_2, Vector3d{0, 0, 1}).toRotationMatrix();

    t_wedge1 = U * R_z_pos * sigma_matrix * U.transpose();
    t_wedge2 = U * R_z_neg * sigma_matrix * U.transpose();
    R1 = U * R_z_pos.transpose() * V.transpose();
    R2 = U * R_z_neg.transpose() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    // check scale
    auto E_a = E.array();
    auto tR_a = tR.array();
    cout << "scale: " << E_a / tR_a << endl;

    return 0;
}