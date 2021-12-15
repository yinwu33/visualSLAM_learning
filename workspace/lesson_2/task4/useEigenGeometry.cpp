#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


Eigen::Isometry3d QuaterniondToTransformMatrix(Eigen::Quaterniond q, Eigen::Vector3d t) {
    Eigen::Isometry3d T;
    T.setIdentity();
    T.rotate(q.toRotationMatrix());
    T.pretranslate(t);

    return T;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;

    // coordinates: robot, body, lidar, camera, world

    // init parameters
    Eigen::Quaterniond q_w_r{0.55, 0.3, 0.2, 0.2};
    Eigen::Quaterniond q_r_b{0.99, 0, 0, 0.01};
    Eigen::Quaterniond q_b_l{0.3, 0.5, 0, 20.1};
    Eigen::Quaterniond q_b_c{0.8, 0.2, 0.1, 0.1};
    q_w_r.normalize();
    q_r_b.normalize();
    q_b_l.normalize();
    q_b_c.normalize();
    const Eigen::Vector3d t_w_r{0.1, 0.2, 0.3};
    const Eigen::Vector3d t_r_b{0.05, 0, 0.5};
    const Eigen::Vector3d t_b_l{0.4, 0, 0.5};
    const Eigen::Vector3d t_b_c{0.5, 0.1, 0.5};

    // quaternion to transform matrix
    const auto T_w_r = QuaterniondToTransformMatrix(q_w_r, t_w_r);
    const auto T_r_b = QuaterniondToTransformMatrix(q_r_b, t_r_b);
    const auto T_b_l = QuaterniondToTransformMatrix(q_b_l, t_b_l);
    const auto T_b_c = QuaterniondToTransformMatrix(q_b_c, t_b_c);

    // initial point x corresponding to Camera
    Eigen::Vector3d x_c{0.3, 0.2, 1.2};
    
    // calculate x_l, x corresponding to Lidar
    Eigen::Vector3d x_l_q = q_b_l.inverse() * ((q_b_c * x_c + t_b_c) - t_b_l);
    Eigen::Vector3d x_l_t = T_b_l.inverse() * T_b_c * x_c;

    // calculate x_w, x corresponding to World
    Eigen::Vector3d x_w_q = q_w_r * (q_r_b * (q_b_c * x_c + t_b_c) + t_r_b) + t_w_r;
    Eigen::Vector3d x_w_t = T_w_r * T_r_b * T_b_c * x_c;

    std::cout << "==x_l with quaternion: \n" << x_l_q << std::endl;
    std::cout << "==x_l with transform : \n" << x_l_t << std::endl;
    std::cout << "==x_w with quaternion: \n" << x_w_q << std::endl;     
    std::cout << "==x_w with transform : \n" << x_w_t << std::endl; 

    return 0;
}