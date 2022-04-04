#include <iostream>
using namespace std;

#include<ctime>
// Eigen core
#include<Eigen/Core>
//Algebraic operation of dense matrices (inverse, eigen vals)
#include<Eigen/Dense>

#define MATRIX_SIZE 50

template<typename T>
void PrintMatrix(const Eigen::MatrixBase<T>& m, int rows, int cols) {
    for (int i=0; i < rows; ++i) {
        for (int j=0; j< cols; ++j) {
            cout << m(i, j) << "|";
        }
        cout << "_____________" << endl;
    }
}

using namespace Eigen;

int OperationWithMatrix() {
    // type, row, column
    Matrix<float, 2, 3> matrix23;

    // Essentially vector3 is 3x1 matrix
    Vector3d v3d;
    Matrix<float, 3, 1> vd_3d;

    //Matrix_3d is 3x3 Matrix ~ Matrix<float, 3, 3>
    Matrix3d matrix_33 = Matrix3d::Zero();

    // Dynamic size matrix
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    // same
    MatrixXd matrix_x;

    // Operations;
    matrix23 << 1, 2, 3, 4, 5, 6; //input (init)
    cout << "matrix 2x3 from 1 to 6" << endl;
    for (int i=0; i < 2; ++i) {
        for (int j=0; j< 3; ++j) {
            cout << matrix23(i, j) << "\t";
        }
        cout << endl;
    }

    // init vector and matrix
    v3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    Matrix<double, 2, 1> result = matrix23.cast<double>() * v3d;
    cout << "[1,2,3;4,5,6]∗[3,2,1]: " << result.transpose() << endl;

    Matrix<float, 2, 1> result2 = matrix23 * vd_3d;
    cout <<"[1,2,3;4,5,6]∗[4,5,6]: " << result2.transpose() << endl;

    matrix_33 = Matrix3d::Random();
    cout << "random matrix: \n" << matrix_33 << endl;
    cout << "transpose: \n" << matrix_33.transpose() << endl;
    cout << "sum: " << matrix_33.sum() << endl;
    cout << "trace: " << matrix_33.trace() << endl;
    cout << "times 10: \n" << 10 * matrix_33 << endl;
    cout << "inverse: \n" << matrix_33.inverse() << endl;
    cout << "det: " << matrix_33.determinant() << endl;

    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrixNN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrixNN *= matrixNN.transpose();

    Matrix<double, MATRIX_SIZE, 1> vectorN =  MatrixXd::Random(MATRIX_SIZE, 1);


    // Solving Matrix equation w\w-t Decomposition and compare the time

    clock_t time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x = matrixNN.inverse() * vectorN;
    cout << "time of normal inverse is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;

    cout << "x = " << x.transpose() << endl;

    time_stt = clock();
    x = matrixNN.colPivHouseholderQr().solve(vectorN);
    cout << "time of Qr decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;


    time_stt = clock();
    x = matrixNN.ldlt().solve(vectorN);
    cout << "time of ldlt decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;


    cout << "TEST PRINTING" << endl;

    PrintMatrix(matrix_33, 3, 3);
    return 0;
}
