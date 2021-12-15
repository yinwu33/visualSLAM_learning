#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <Eigen/Core>

#include <pangolin/pangolin.h>

#include "point.hpp"
#include "cameraParameters.hpp"
#include "measurement.hpp"

#include "vertexPosition.hpp"
#include "vertexCamera.hpp"
#include "edgeProjection.hpp"


using namespace std;

void Draw(const std::vector<Point> &points, std::string window_name) {

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind(window_name, 1024, 768);
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
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            Eigen::Vector3d position = points[i].position;
            glColor3f(0.0, position[2]/4, 1.0-position[2]/4);
            glVertex3d(position[0], position[1], position[2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char** argv) {

    if (argc != 2) {
        std::cout << "need file path" << std::endl;
        return 1;
    }
    std::ifstream ifs(argv[1]);

    if (!ifs.is_open()) {
        std::cout << "wrong file path" << std::endl;
        return 1;
    }

    // load measurements
    std::vector<Measurement> measurements;

    int num_cameras, num_points, num_observations;
    ifs >> num_cameras >> num_points >> num_observations;

    std::cout << "num_cameras: " << num_cameras << "\nnum_points: " << num_points << "\nnum_observations: " << num_observations << std::endl;

    for (int i = 0; i < num_observations; ++i) {
        size_t camera_index, point_index;
        double u, v;
        ifs >> camera_index >> point_index >> u >> v;
        Measurement m(camera_index, point_index, u, v);

        measurements.push_back(m);
    }

    // load camera parameters
    std::vector<CameraParameters> camera_parameters;
    for (int i = 0; i < num_cameras; ++i) {
        double r1, r2, r3, t1, t2, t3, f, k1, k2;

        ifs >> r1 >> r2 >> r3 >> t1 >> t2 >> t3 >> f >> k1 >> k2;
        CameraParameters cp(r1, r2, r3, t1, t2, t3, f, k1, k2);
        camera_parameters.push_back(cp);
    }

    // load points
    std::vector<Point> points;
    for (int i = 0; i < num_points; ++i) {
        double x, y, z;
        ifs >> x >> y >> z;
        Point p(x, y, z);
        points.push_back(p);
    }
    ifs.close();


    // create and initialize g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> MyBlockSolver;
    typedef g2o::LinearSolverEigen<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    std::vector<VertexCamera *> vertices_cam;
    std::vector<VertexPosition *> vertices_pos;
    std::vector<EdgeProjection *> edges;

    // add edges and vertices
    // add camera pose into vertices
    for (int i = 0; i < num_cameras; ++i) {
        VertexCamera* v = new VertexCamera();

        v->setId(i);
        v->setEstimate(camera_parameters[i]);

        if (i == 0) {
            v->setFixed(true);
        } // todo

        optimizer.addVertex(v);
        vertices_cam.push_back(v);
    }
    // add point position into vertices
    for (int i = 0; i < num_points; ++i) {
        VertexPosition* v = new VertexPosition();

        v->setId(i + num_cameras);
        v->setEstimate(points[i].position);
        v->setMarginalized(true); // ???

        optimizer.addVertex(v);
        vertices_pos.push_back(v);
    }

    // add edges of pair<VertexPosition, VertexSE3d>
    int index_e = 0;
    for (int i = 0; i < num_observations; ++i) {
        EdgeProjection* e = new EdgeProjection();

        int cam_index = measurements[i].camera_index;
        int p_index = measurements[i].point_index + num_cameras;

        e->setId(index_e++);
        e->setVertex(0, optimizer.vertices()[cam_index]);
        e->setVertex(1, optimizer.vertices()[p_index]);
        e->setInformation(Eigen::Matrix2d::Identity());
        e->setMeasurement(Eigen::Vector2d{ measurements[i].u, measurements[i].v });

        optimizer.addEdge(e);
        edges.push_back(e);
    }

    // start optimization
    optimizer.initializeOptimization();

    optimizer.optimize(20);

    // ofstream fout("../graph.g2o");
    // for (const auto v : vertices_cam) {
    //     fout << "VERTEX_SE3:QUAT ";
    //     v->write(fout);
    // }

    // for (const auto v: vertices_pos) {
    //     fout << "VERTEX_TRACKXYZ ";
    //     v->write(fout);
    // }

    // for (const auto e: edges) {
    //     fout << "EDGE_SE3_TRACKXYZ ";
    //     e->write(fout);
    // }

    Draw(points, "before");

    return 0;
}

