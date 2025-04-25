//
// Created by Scott on 4/18/2025.
//
#include <gtsam/nonlinear/LIEKF.h>
#include <gtsam/navigation/NavState.h>
#include <iostream>

using namespace std;
using namespace gtsam;

// define dynamics
Vector9 dynamics(const NavState& X, const Vector6& imu,
                 OptionalJacobian<9, 9> H = {}) {
  const auto a = imu.head<3>();
  const auto w = imu.tail<3>();
  Vector9 result;
  result << w, Z_3x1, a;
  if (H) {
    *H = Matrix::Zero(9, 9);
  }
  return result;
}

// define measurement processor
  Vector3 h_gps(const NavState& X,
                        OptionalJacobian<3,9> H = {}) {
    if (H) *H << Z_3x3, Z_3x3, X.R();
    return X.t();
    }

  int main() {
    // Initialization
    NavState X0;
    Matrix9 P0 = Matrix9::Identity() * 0.1;
    double dt = 1.0;

    // Create measurement function h_func
    GeneralLIEKF<NavState, Vector3, 6>::MeasurementFunction h_func =
        [](const NavState& X, OptionalJacobian<3, 9> H) { return h_gps(X, H); };

    // Create dynamics
    GeneralLIEKF<NavState, Vector3, 6>::Dynamics dynamics_func = dynamics;

    // Initialize filter
    GeneralLIEKF<NavState, Vector3, 6> ekf(X0, P0, dynamics_func, h_func);

    // Covariances
    Matrix9 Q = Matrix9::Identity() * 0.1;
    Matrix3 R = Matrix3::Identity()*0.01;

    // IMU measurements
    Vector6 imu1, imu2;
    imu1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    imu2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // GPS measurements
    Vector3 z1, z2;
    z1 << 0.0, 0.0, 0.0;
    z2 << 0.0, 0.0, 0.0;

    // Predict / update stages
    cout << "\nInitialization:\n";
    cout << "X0: " << ekf.getState() << endl;
    cout << "P0: " <<  ekf.getCovariance()  << endl;


    ekf.predict(imu1, dt, Q);
    cout << "\nFirst Prediction:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << ekf.getCovariance() << endl;

    ekf.update(z1, R);
    cout << "\nFirst Update:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << ekf.getCovariance() << endl;

    ekf.predict(imu2, dt, Q);
    cout << "\nSecond Prediction:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << ekf.getCovariance() << endl;

    ekf.update(z2, R);
    cout << "\nSecond Update:\n";
    cout << "X: " << ekf.getState() << endl;
    cout << "P: " << ekf.getCovariance() << endl;

    return 0;
   }