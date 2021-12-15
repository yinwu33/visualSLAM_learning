#ifndef VERTEX_POSITION_HPP
#define VERTEX_POSITION_HPP

#include <fstream>

#include <Eigen/Core>

#include <g2o/core/base_vertex.h>


class VertexPosition : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual bool read(std::istream &is) override { return true; }

    virtual bool write(std::ostream &os) const override {
        os << id() << " ";
        os << _estimate[0] << " " << _estimate[1] << " " << _estimate[2] << std::endl;

        return true;
        }

    virtual void setToOriginImpl() override {
        _estimate = Eigen::Vector3d{0, 0, 0};
    }

    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d{update[0], update[1], update[2]};
    }
};

# endif // VERTEX_POSITION_HPP