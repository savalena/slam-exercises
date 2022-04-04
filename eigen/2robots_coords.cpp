#include <iostream>

using namespace std;

#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace Eigen;

template<typename Q>
void MakeTransfrom(Isometry3d &T, Q &q, Vector3d& t){
    T.rotate(q);
    T.pretranslate(t);
};


int TwoRobotsCords() {
    Quaterniond q1(0.35, 0.2, 0.3, 0.1),
            q2(-0.5, 0.4, -0.1, 0.2);
    q1.normalize();
    q2.normalize();
    Vector3d t1(0.3, 0.1, 0.1),
            t2(-0.1, 0.5, 0.3),
            p1(0.5, 0, 0.2);

    Isometry3d T1 = Isometry3d::Identity(),
            T2 = Isometry3d::Identity(), T1_inv;

    MakeTransfrom(T1, q1, t1);
    MakeTransfrom(T2, q2, t2);

    Vector3d p3 = T2 * T1.inverse() * p1;
    cout << p3.transpose() << endl;
    // My method with the usage of definition of T inverse doesn't work.
    // Something wrong with Quaternion  ???
////    Matrix3d T1_inv = T1.matrix().inverse();
//    Matrix3d R = q1.toRotationMatrix();
//    Vector3d t = t1;
//    cout << "T: "<< T1.matrix() << endl;
//    cout << "R: " << R << endl;
//    cout << "t: " << t.transpose() << endl;
//
//    Matrix3d Rinv = R.transpose();
//    cout << "Rinv : "<< Rinv << endl;
//    Vector3d t_inv = -Rinv * t;
//    cout << "tinv: " << t_inv;
//
//    Quaterniond q = Quaterniond(R);
//    cout << "q: " << q.coeffs().transpose() << endl;
//    q.normalize();
//    MakeTransfrom(T1_inv, q, t_inv);

//    cout << t << endl;
//    cout << "T1inv: " << T1_inv.matrix() << endl;
//
//    Vector3d p2 = T2 * T1_inv * p1;
//    cout << p2.transpose() << endl;
    return 0;
}
