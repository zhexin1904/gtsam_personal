//
// Created by Scott on 4/18/2025.
//
#include <gtsam/nonlinear/LIEKF.h>
#include <gtsam/geometry/Pose2.h>
#include <iostream>

using namespace std;
using namespace gtsam;

  // Measurement Processor
  Vector2 h_gps(const Pose2& X,
                        OptionalJacobian<2,3> H = {}) {
    return X.translation(H);
    }
  int main() {
    static const int dim = traits<Pose2>::dimension;

    // Initialization
    Pose2 X0(0.0, 0.0, 0.0);
    Matrix3 P0 = Matrix3::Identity() * 0.1;
    double dt = 1.0;

    // Define GPS measurements
    Matrix23 H;
    h_gps(X0, H);
    Vector2 z1, z2;
    z1 << 1.0, 0.0;
    z2 << 1.0, 1.0;

	std::function<Vector2(const Pose2&, gtsam::OptionalJacobian<2, 3>)> measurement_function = h_gps;
    LIEKF<Pose2, Vector2> ekf(X0, P0, measurement_function);

    // Define Covariances
    Matrix3 Q = (Vector3(0.05, 0.05, 0.001)).asDiagonal();
    Matrix2 R = (Vector2(0.01, 0.01)).asDiagonal();

    // Define odometry movements
    Pose2 U1(1.0,1.0,0.5), U2(1.0,1.0,0.0);

    // Define a transformation matrix to convert the covariance into (theta, x, y) form.
    Matrix3 TransformP;
    TransformP << 0, 0, 1,
        1,0,0,
        0,1,0;

    // Predict / update stages
    cout << "\nInitialization:\n";
    cout << "X0: " << ekf.getState() << endl;
    cout << "P0: " << TransformP * ekf.getCovariance() * TransformP.transpose() << endl;


    ekf.predict(U1, Q);
    cout << "\nFirst Prediction:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << TransformP * ekf.getCovariance() * TransformP.transpose() << endl;

    ekf.update(z1, R);
    cout << "\nFirst Update:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << TransformP * ekf.getCovariance() * TransformP.transpose() << endl;

    ekf.predict(U2, Q);
    cout << "\nSecond Prediction:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << TransformP * ekf.getCovariance() * TransformP.transpose() << endl;

    ekf.update(z2, R);
    cout << "\nSecond Update:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << TransformP * ekf.getCovariance() * TransformP.transpose() << endl;

    return 0;
   }