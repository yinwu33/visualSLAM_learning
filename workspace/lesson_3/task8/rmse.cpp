#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>


using SE3dList = std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;

bool ParseData(SE3dList &poses, const std::string filePath) {
    std::ifstream ifs(filePath);

    if(!ifs.is_open()) {
    std::cout << "open file error!" << std::endl;
    return false;
    }

    double t;
    double t_x;
    double t_y;
    double t_z;
    double q_x;
    double q_y;
    double q_z;
    double q_w;
    
    while(ifs >> t) {
        ifs >> t_x;
        ifs >> t_y;
        ifs >> t_z;
        ifs >> q_x;
        ifs >> q_y;
        ifs >> q_z;
        ifs >> q_w;
        Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
        Eigen::Vector3d t(t_x, t_y, t_z);

        Sophus::SE3d p(q, t);
        poses.push_back(p);
    }
    return true;
};

double rmse(const SE3dList &a, const SE3dList &b) {
    int size = a.size();
    double err = 0.0;

    for (int i = 0; i < size; ++i) {
        Sophus::Vector6d residual = (b[i].inverse() * a[i]).log();
        err += residual.squaredNorm();
    }

    return sqrt(err/size);
};

int main(int argc, char** argv) {
    SE3dList estimated;
    SE3dList groundtruth;

    if(!ParseData(estimated, "../estimated.txt")) return -1;
    if(!ParseData(groundtruth, "../groundtruth.txt")) return -1;

    if(estimated.size() != groundtruth.size()) {
        std::cout << "size not same!" << std::endl;
        return -1;
    }

    std::cout << "RMSE: " << rmse(estimated, groundtruth) << std::endl;
    return 0;
}