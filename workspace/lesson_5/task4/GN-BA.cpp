//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

// #include "sophus/se3.h"
#include <sophus/se3.hpp>

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream ifs_3d(p3d_file);
    if (not ifs_3d.is_open()) 
        return -1;
    
    double x_;
    double y_; 
    double z_;

    while (ifs_3d >> x_) {
        ifs_3d >> y_;
        ifs_3d >> z_;
        p3d.push_back(Vector3d{x_, y_, z_});
    }
    ifs_3d.close();
    cout << "p3d loaded with size: " << p3d.size() << endl;

    ifstream ifs_2d(p2d_file);
    if (! ifs_2d.is_open())
        return -1;
    
    while (ifs_2d >> x_) {
        ifs_2d >> y_;
        p2d.push_back(Vector2d{x_, y_});
    }
    ifs_2d.close();
    cout << "p3d loaded with size: " << p2d.size() << endl;
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector3d p3d_reproj = (T_esti * p3d[i]).matrix();
            double x = p3d_reproj[0];
            double y = p3d_reproj[1];
            double z = p3d_reproj[2];
            p3d_reproj = K * p3d_reproj;
            Vector2d e_2 = p2d[i] - Vector2d{p3d_reproj[0]/p3d_reproj[2], p3d_reproj[1]/p3d_reproj[2]};
            cost += e_2.transpose()*e_2/2;
	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            J(0, 0) = fx/z;
            J(0, 1) = 0;
            J(0, 2) = -fx*x/(z*z);

            J(0, 3) = -fx*x*y/(z*z);
            J(0, 4) = fx+fx*x*x/(z*z);
            J(0, 5) = -fx*y/z;

            J(1, 0) = 0;
            J(1, 1) = fy/z;
            J(1, 2) = -fy*y/(z*z);

            J(1, 3) = -fy-fy*y*y/(z*z);
            J(1, 4) = fy*x*y/(z*z);
            J(1, 5) = fy*x/z;
            J = -J;

	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e_2;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        T_esti = T_esti * Sophus::SE3d::exp(dx);

        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
