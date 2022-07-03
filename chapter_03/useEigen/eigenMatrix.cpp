#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using std::cout;
using std::endl;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Dynamic;

#define MATRIX_SIZE 50

int main(int argc, char **argv) {
    // float matrix 
    Matrix<float, 2, 3> matrix_23;

    // those two are the same representation of 1-d vector
    Vector3d v_3d;
    Matrix<float, 3, 1> vd_3d;
    
    // double zero matrix 
    Matrix3d matrix_33 = Matrix3d::Zero();

    // dynamic matrix; those two are same
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    MatrixXd matrix_x;

    // give element to the matrix
    matrix_23<<1, 2, 3, 4, 5, 6;
    cout<<"matrix_23 from 1 to 6: \n"<<matrix_23<<endl;

    // print members of a matrix
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cout<<matrix_23(i, j)<<"\t"<<endl;
        }
    }

    // matrix multiplication with the same precision
    v_3d<<3, 2, 1;
    vd_3d<<4, 5, 6;
    Matrix<double ,2, 1> result = matrix_23.cast<double>() * v_3d;
    cout<< "[1, 2, 3; 4, 5, 6] * [3, 2, 1] = "<<result.transpose()<<endl;
    Matrix<float , 2, 1> result2 = matrix_23 * vd_3d;
    cout<<"[1, 2, 3; 4, 5, 6] * [4, 5, 6]= "<<result2.transpose()<<endl;

    // some matrix operation
    matrix_33 = Matrix3d::Random();
    cout<< "random matrix : \n" << matrix_33 <<endl;
    cout<< "transpose: \n" << matrix_33.transpose()<<endl;
    cout<< "sum: "<< matrix_33.sum() <<endl;
    cout<< "trace: "<<matrix_33.trace()<<endl;
    cout<< "times 10: \n"<< 10*matrix_33<<endl;
    cout<< "inverse: \n" << matrix_33.inverse()<<endl;
    cout<< "det: " << matrix_33.determinant()<<endl;

    // eigen values and eigen vectors
    Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose()*matrix_33);
    cout<< "Eigen values = \n"<<eigen_solver.eigenvalues()<<endl;
    cout<< "Eigen vectors = \n"<<eigen_solver.eigenvectors()<<endl;

    // solve equations with different methods
    // set up the equation 
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);
    clock_t time_tt = clock();
    // directly calculate inverse
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout<<"time of normal inverse is "<<1000*(clock() - time_tt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
    cout<<"x = "<<x.transpose()<<endl;
    //matrix factorization, QR 
    time_tt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout<<"time of QR decomposition is "<<1000*(clock() - time_tt)/(double) CLOCKS_PER_SEC<<"ms"<<endl;
    cout<<"x = "<<x.transpose()<<endl;
    //positive definite matrix factorization, QR 
    time_tt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout<<"time of ldlt decomposition is "<<1000*(clock() - time_tt)/(double) CLOCKS_PER_SEC<<"ms"<<endl;
    cout<<"x = "<<x.transpose()<<endl;

    return 0;
}