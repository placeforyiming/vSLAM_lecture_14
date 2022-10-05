#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <chrono>

using std::cout;
using std::endl;
using std::vector;
using Eigen::Matrix3d;
using Eigen::Vector3d;

int main(int argc, char **argv) {
    // fitting function y = exp(ax^2 + bx + c) + w,
    // where a, b, c are parameters and w is gaussion noise
    // the correct function parameter:
    double ar = 1.0, br = 2.0, cr = 1.0;
    // the initial guess
    double ae = 2.0, be = -1.0, ce = 5.0;
    // num of samples
    int N = 200;
    // tje sigma noise of w
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    // random number generator
    cv::RNG rng;

    // sample data
    vector<double> x_data, y_data;
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // start gaussian newton iteration
    int iterations = 100;
    double cost = 0, lastCost = 0;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++) {
        Matrix3d H = Matrix3d::Zero();
        Vector3d b = Vector3d::Zero();
        cost = 0;
        for (int i = 0; i < N; i++) {
            double xi = x_data[i], yi = y_data[i];
            double error = yi - exp(ae*xi*xi + be*xi + ce);
            Vector3d J;
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // Ja
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // Jb
            J[2] = -exp(ae * xi * xi + be * xi + ce);  // Jc
            // weighting the H,b with the gausian probability
            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;
            cost += error * error;
        }

        // solve equation Hx = b
        Vector3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
        cout << "result is nan!" << endl;
        break;
        }
        
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;

        cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
            "\t\testimated params: " << ae << "," << be << "," << ce << endl;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;

    cout << "#####################################################" << endl;
    return 0;
}