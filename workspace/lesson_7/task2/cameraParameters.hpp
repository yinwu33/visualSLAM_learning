#ifndef CAMERA_PARAMETERS_HPP
#define CAMERA_PARAMETERS_HPP

#include <Eigen/Core>
#include <sophus/se3.hpp>


struct CameraParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CameraParameters() = default;
  CameraParameters(double r1_, double r2_, double r3_,
                   double t1_, double t2_, double t3_,
                   double f_, double k1_, double k2_)
                   : r1(r1_), r2(r2_), r3(r3_),
                     t1(t1_), t2(t2_), t3(t3_),
                     f(f_), k1(k1_), k2(k2_) {
    Eigen::Vector3d v{ r1, r2, r3 };
    Eigen::Vector3d t{ t1, t2, t3 };

    Sophus::SO3d rotation = Sophus::SO3d::exp(v);
    pose = Sophus::SE3d(rotation, t);
  }

  double r1, r2, r3, t1, t2, t3, f, k1, k2;
  Sophus::SE3d pose;

  Eigen::Vector2d projection(const Eigen::Vector3d& p_w) {
    Eigen::Vector3d p_c = pose.rotationMatrix() * p_w + pose.translation();

    double x_ = p_c[0] / p_c[2];
    double y_ = p_c[1] / p_c[2];

    double r2 = x_ * x_ + y_ * y_;
    double distortion = 1 + k1 * r2 + k2 * r2 * r2;
    double x_d = distortion * x_;
    double y_d = distortion * y_;

    return Eigen::Vector2d{ x_d * f + 0, y_d * f + 0 };  // todo no cx cy?
  }
};

#endif // CAMERA_PARAMETERS_HPP