#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
#include <vector>
#include <fstream>

using std::vector;
using std::exp;
using std::istream;
using std::ostream;
using std::cout;
using std::endl;
// <dimension of optimized paarameter, data type of parameters>
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // reset, _estimate now is vector3d
    virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }
    // update
    virtual void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector3d(update);
    }
    // read and write, keep empty now
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
};

// uniry edge, <dimension of observation, observation type, vertex type>
class CurveFitttingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // constructor, take the input
    CurveFitttingEdge(double x) : BaseUnaryEdge(), _x(x) {}
    // calculate the residual error
    virtual void computeError() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        // v->estimate() return _estimate, the optimized variables
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - exp(abc(0, 0) *_x *_x + abc(1, 0) *_x + abc(2, 0));
    }
    // calculate the jacobian
    virtual void linearizeOplus() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = exp(abc[0] *_x *_x + abc[1] *_x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }     
    // read and write, keep empty now
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
 public:
    double _x;
};

int main(int argc, char **argv) {
    
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;

    vector<double> x_data, y_data;
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // construct the graph optimization, varaible has 3 dimensions, residual error has 1 dimension
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
    // linear solver
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // set the optimization method
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // add vertex into graph
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    // add edge into graph
    for (int i = 0; i < N; i++) {
        CurveFitttingEdge * edge = new CurveFitttingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma*w_sigma));
        optimizer.addEdge(edge);
    }

    // execute optimization
    cout << "start optimization" << endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " second." << endl;

    // cout optimized value
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl; 
    cout << "#####################################################" << endl;
    
    return 0;
}