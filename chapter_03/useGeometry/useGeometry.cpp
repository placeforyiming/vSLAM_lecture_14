#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using std::cout;
using std::endl;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Isometry3d;
using Eigen::Quaterniond;


int main(int argc, char ** argv) {
    // rotation 
    Matrix3d rotation_matrix = Matrix3d::Identity();
    //define rotation vector, which rotates 45 degree around z-axix
    AngleAxisd rotation_vector(M_PI/4, Vector3d(0, 0, 1));  
    cout.precision(3);
    // rotation vector to rotation matrix
    cout<<"rotation matrix = \n"<<rotation_vector.matrix()<<endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    // tranformation with AngleAxis
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    // transformation with rotation matrix
    v_rotated = rotation_matrix * v;
    cout<<"(1, 0, 0) after rotation (by angle axis) = "<<v_rotated.transpose()<<endl;
    // rottion matrix to euler angle, ZYX, roll pitch yaw
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;

    // euclidean transformation
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1,3,4)); // assign a translation vector
    cout<<"Transformation matrix = \n"<<T.matrix()<<endl;

    Vector3d v_transformed = T * v;
    cout<<"v transformed = "<<v_transformed.transpose()<<endl;
    // Eigen::Affine3d Eigen::Projective3d for other two transformations

    //quaternion with rotation vector
    Quaterniond q = Quaterniond(rotation_vector);
    cout<<"quaternion from rotation vector = "<<q.coeffs().transpose()<<endl;
    // quaternion with rotation matrix
    q = Quaterniond(rotation_matrix);
    cout<<"quaternion from rotation matrix = "<<q.coeffs().transpose()<<endl;

    // rotate a vector with quaterniond
    v_rotated = q * v;
    cout<<"(1, 0, 0) after rotation = "<< v_rotated.transpose()<<endl;
    cout<<"should be equal to "<<(q*Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose()<<endl;
    return 0;

}