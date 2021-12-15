#ifndef POINT_HPP
#define POINT_HPP

#include <Eigen/Core>


struct Point {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {
        position = Eigen::Vector3d{ x, y, z };
    }

    double x, y, z;
    Eigen::Vector3d position;
};

#endif // POINT_HPP