#ifndef VERTEX_CAMERA_HPP
#define VERTEX_CAMERA_HPP

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>

#include "cameraParameters.hpp"

class VertexCamera : public g2o::BaseVertex<6, CameraParameters> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual bool read(std::istream &is) override { return false; }

    virtual bool write(std::ostream &os) const override {
        os << id() << " ";
        Eigen::Quaterniond q = _estimate.pose.unit_quaternion();
        os << _estimate.pose.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
        
        return true;
        }

    virtual void setToOriginImpl() override {
        _estimate = CameraParameters();
    }

    virtual void oplusImpl(const double* update) override {
        Eigen::Matrix<double, 6, 1> upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate.pose = Sophus::SE3d::exp(upd) * _estimate.pose;
    }

};

#endif // VERTEX_CAMERA_HPP