//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Python.h>
#include <pybind11/pybind11.h>

// todo task2
// todo end

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) :Rwc(R), qwc(R), twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};
double run(double stdvar, int startFrame)
{


    int poseNums = 100;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for (int n = 0; n < poseNums; ++n) {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R, t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    // todo::homework, task 2, add noise in measurement
    std::default_random_engine generator_2;
    std::normal_distribution<double> dist(0., stdvar);

    // todo end

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = startFrame;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();
        // todo: homework, task 2, add noise in measurement
        double u = x / z;
        double v = y / z;
        if (stdvar != 0.) {
            u = u + dist(generator);
            v = v + dist(generator);
        }


        camera_pose[i].uv = Eigen::Vector2d(u, v);
        // todo end
    }

    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */
    int num = end_frame_id - start_frame_id;
    Eigen::MatrixXd D(num * 2, 4);

    for (int i = 0; i < num; ++i) {
        double u = camera_pose[i + start_frame_id].uv[0];
        double v = camera_pose[i + start_frame_id].uv[1];

        Eigen::Matrix3d Rcw = camera_pose[i + start_frame_id].Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw * camera_pose[i + start_frame_id].twc;

        Eigen::Vector4d P0, P1, P2;
        P0 << Rcw(0, 0), Rcw(0, 1), Rcw(0, 2), tcw(0);
        P1 << Rcw(1, 0), Rcw(1, 1), Rcw(1, 2), tcw(1);
        P2 << Rcw(2, 0), Rcw(2, 1), Rcw(2, 2), tcw(2);

        D.block(2 * i, 0, 1, 4) = u * P2.transpose() - P0.transpose();
        D.block(2 * i + 1, 0, 1, 4) = v * P2.transpose() - P1.transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix4d V = svd.matrixV();
    Eigen::Vector4d v_last = V.col(3);
    P_est = v_last.hnormalized();

    Eigen::Vector4d SingularValues = svd.singularValues();
    double ratio = SingularValues[3] / SingularValues[2];

    /* your code end */

    std::cout << "==========" << std::endl;
    std::cout << "stdvar: " << stdvar << std::endl;
    std::cout << "startFrame: " << startFrame << std::endl;
    std::cout << "ground truth: \n" << Pw.transpose() << std::endl;
    std::cout << "your result: \n" << P_est.transpose() << std::endl;
    std::cout << "theta[2]: " << SingularValues[2] << " theta[3]: " << SingularValues[3] << std::endl;
    std::cout << "Ratio: " << ratio << std::endl;
    // TODO:: 请如课程讲解中提到的判断三角化结果好坏的方式，绘制奇异值比值变化曲线
    return ratio;
}

int main(int argc, char** argv) {
    double stdvar = 0.0;
    int startFrame = 3;
    if (argc == 2) {
        stdvar = atof(argv[1]);
    }
    else if (argc == 3) {
        stdvar = atof(argv[1]);
        startFrame = atoi(argv[2]);

    }
    run(stdvar, startFrame);
    return EXIT_SUCCESS;
}

PYBIND11_MODULE(triangulate, m) {
    m.doc() = "pybind 11 triangulate";
    m.def("run", &run, "run(double stdvar, int startFrame)", pybind11::arg("stdvar"), pybind11::arg("startFrame"));
}


