#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>

using namespace std;

using Pose = Sophus::SE3d;
using VecPose =vector<Pose, Eigen::aligned_allocator<Pose>>;
using Point = Eigen::Vector3d;
using VecPoint = vector<Point, Eigen::aligned_allocator<Point>>;

// path to trajectory file
string compare_file = "../compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(VecPose, VecPose);

void solveICP(const VecPose &T_e_list, const VecPose &T_g_list, Pose &T_e_g);
void readFile(string file_path, VecPose &estimate, VecPose &groundtruth);

int main(int argc, char **argv) {

    // read file
    VecPose T_e_list;
    VecPose T_g_list;
    VecPose T_g_transformed_list;

    readFile(compare_file, T_e_list, T_g_list);
    // end your code here

    assert(T_e_list.size() == T_g_list.size());
    int path_size = T_e_list.size();
    cout << "path size = " << path_size << endl;

    int iterations = 100;
    double cost = 0;

    Pose T_e_g;

    solveICP(T_e_list, T_g_list, T_e_g);

    // transform
    for (int i = 0; i < path_size; ++i) {
        T_g_transformed_list.push_back(T_e_g * T_g_list[i]);
    }

    // draw trajectory in pangolin
    assert(T_e_list.size() == T_g_transformed_list.size());
    DrawTrajectory(T_e_list, T_g_transformed_list);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(VecPose poses, VecPose poses_2) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses_2.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = poses_2[i], p2 = poses_2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

void readFile(string file_path, VecPose &estimate, VecPose &groundtruth) {
    ifstream ifs(file_path);

    if (!ifs.is_open()) {
        std:cerr << "cannot open file: " << file_path << std::endl;
        return;
    }

    double t;
    double t_x;
    double t_y;
    double t_z;
    double q_x;
    double q_y;
    double q_z;
    double q_w;

    while (!ifs.eof()) {
        // estimate
        ifs >> t;
        ifs >> t_x;
        ifs >> t_y;
        ifs >> t_z;
        ifs >> q_x;
        ifs >> q_y;
        ifs >> q_z;
        ifs >> q_w;
        Eigen::Quaterniond q_e(q_w, q_x, q_y, q_z);
        Eigen::Vector3d t_e_list(t_x, t_y, t_z);

        Pose p_e(q_e, t_e_list);
        estimate.push_back(p_e);

        // ground truth
        ifs >> t;
        ifs >> t_x;
        ifs >> t_y;
        ifs >> t_z;
        ifs >> q_x;
        ifs >> q_y;
        ifs >> q_z;
        ifs >> q_w;
        Eigen::Quaterniond q_g(q_w, q_x, q_y, q_z);
        Eigen::Vector3d t_g_list(t_x, t_y, t_z);

        Pose p_g(q_g, t_g_list);
        groundtruth.push_back(p_g);

        }

    ifs.close();
}

void solveICP(const VecPose &T_e_list,const  VecPose &T_g_list, Pose &T_e_g) {
    // calculate center of mass
    int path_size = T_e_list.size();

    Point mean_e;
    Point mean_g;

    double x_e, y_e, z_e, x_g, y_g, z_g;

    for (int i = 0; i < path_size; ++i) {
        mean_e += T_e_list[i].translation();
        mean_g += T_g_list[i].translation();
    }

    cout << "mean e before: \n" << mean_e << endl;
    cout << "mean g before: \n" << mean_g << endl;
    auto m_e = mean_e.array();
    m_e = m_e / path_size;
    mean_e = Point(m_e);

    auto m_g = mean_g.array();
    m_g = m_g / path_size;
    mean_g = Point(m_g);

    cout << "mean e: \n" << mean_e << endl;
    cout << "mean g: \n" << mean_g << endl;

    // subscribe mean value
    VecPoint estimate_list;
    VecPoint groundtruth_list;

    for (int i = 0; i < path_size; ++i) {
        Point e = T_e_list[i].translation() - mean_e;
        Point g = T_g_list[i].translation() - mean_g;

        estimate_list.push_back(e);
        groundtruth_list.push_back(g);
    }

    Eigen::Matrix3d W;
    W.setZero();

    for (int i = 0; i < path_size; ++i) {
        W += estimate_list[i] * groundtruth_list[i].transpose();
    }

    cout << "W: \n" << W << endl;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    cout << "U: \n" << U << endl;
    cout << "V: \n" << V << endl;

    Eigen::Matrix3d R = U * V.transpose();
    Point t = mean_e - R * mean_g;

    T_e_g = Pose(R, t);
    cout << "Transform Matrix: \n" << T_e_g.matrix() << endl;
}
