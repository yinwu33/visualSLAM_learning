#ifndef EDGE_PROJECTION_HPP
#define EDGE_PROJECTION_HPP

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <g2o/core/base_binary_edge.h>
#include <sophus/se3.hpp>

#include "cameraParameters.hpp"
#include "vertexPosition.hpp"
#include "vertexCamera.hpp"


class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCamera, VertexPosition> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual bool read(std::istream &is) override { return true; }

    virtual bool write(std::ostream &os) const override {
        VertexCamera* v1 = static_cast<VertexCamera *>(_vertices[0]);
        VertexPosition* v2 = static_cast<VertexPosition *>(_vertices[1]);

        os << v1->id() << " " << v2->id() << " ";
        os << _measurement[0] << " " << _measurement[1] << " ";

        for (int i = 0; i < information().rows(); i++)
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        os << std::endl;

        return true;
    }

    virtual void computeError() override {
        CameraParameters v1 = static_cast<VertexCamera*>(_vertices[0])->estimate(); // camera 6D pose
        Eigen::Vector3d v2 = static_cast<VertexPosition*>(_vertices[1])->estimate(); // predict 3D position of points

        Eigen::Vector2d point_proj = v1.projection(v2);  //todo ? negtive sign?

        // _error = point_proj - _measurement;
        _error = _measurement - point_proj; // ! which one?

    }

    // virtual void linearizeOplus() override {
    //     // vertex 1: camera with 6 parameters J = 2x6
        
    //     // vertex 2: position with 3 parameters J = 2x3
    //     // _jacobianOplusXi = Eigen::Matrix<double, 2, 6>::Zero();
    //     // _jacobianOplusXj = Eigen::Matrix<double, 2, 3>::Zero();
    // }
};

#endif // EDGE_PROJECTION_HPP