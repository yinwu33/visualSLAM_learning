#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

int main(int argc, char** argv) {
    std::cout << "Running VIO Lesson 10 Task 2 ...\n======" << std::endl;

    Eigen::Vector3d w(0.01, 0.02, 0.03);

    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();

    // first method
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d update = Sophus::SO3d::exp(w);
    std::cout << "Update with Sophus: \n" << (SO3_R * update).matrix() << std::endl;

    // second method
    Eigen::Quaterniond q(R);
    Eigen::Quaterniond q_q(1, w[0]/2, w[1]/2, w[2]/2);

    std::cout << "Update q: \n" << q_q.toRotationMatrix() << std::endl;
    std::cout << "Update q normalized: \n" << q_q.normalized().toRotationMatrix() << std::endl;

    std::cout << "Update with Quaternion: \n" << (q * q_q).toRotationMatrix() << std::endl;
    std::cout << "Update with Quaternion normalized before: \n" << (q * q_q.normalized()).toRotationMatrix() << std::endl;
    std::cout << "Update with Quaternion normalized: \n" << (q * q_q).normalized().toRotationMatrix() << std::endl;

    std::cout << "======\nFinished" << std::endl;
    return EXIT_SUCCESS;
}