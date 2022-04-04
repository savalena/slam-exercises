#include <iostream>
#include <cmath>

using namespace std;

#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace Eigen;

int QuaternionsRotationVectorMatrix() {

    // create rotation matrix
    Matrix3d R = Matrix3d::Identity();

    // create the rotation vector, rotated 45 degrees along Z axis

    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));

    cout.precision(3);
    cout << "rotation vector-> matrix: " << endl << rotation_vector.matrix() << endl;
    R = rotation_vector.toRotationMatrix();
//    cout << "rotation matrix: " << endl << R << endl;

    // coordinate transformation with AngleAxis
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;

    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;

    // or use the rotation matrix
    v_rotated = R * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;

    // cvt rotation matrix to Euler Angles
    Vector3d euler_angle = R.eulerAngles(2, 1, 0); // ZYX - rpy order
    cout << "rpy: " << euler_angle.transpose() << endl;

    // Euclidean transformation matrix using Eigen::Isometry. ~ Matrix T
    Isometry3d T = Isometry3d::Identity(); //it is 4x4 by call for 3d

    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1, 3, 4));
    cout << "Euclidean Transfromation T: " << endl << T.matrix() << endl;

    Vector3d v_transformed = T * v;
    cout << "v transformed: " << v_transformed.transpose() << endl;

    // Quaternions
    Quaterniond q(rotation_vector);
    cout << "quaternion from rotation vector = " << q.coeffs().transpose() << endl;

    // can also assign a rotation matrix to it
    q = Quaterniond(R);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;

    // Rotate with quanternion
    Vector3d new_rotated = q * v;
    cout << "rotated w quaternions: " << new_rotated.transpose() << endl;
    // obtaining rotated v using math formula q * x * q(-1)
    cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs
            ().transpose() << endl;



    return 0;
}
