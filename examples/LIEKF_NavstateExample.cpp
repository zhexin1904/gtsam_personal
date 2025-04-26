/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LIEKF_NavstateExample.cpp
 * @brief A left invariant Extended Kalman Filter example using the LIEKF
 * on NavState using IMU/GPS measurements.
 *
 * This example uses the templated LIEKF class to estimate the state of
 * an object using IMU/GPS measurements. The prediction stage of the LIEKF uses
 * a generic dynamics function to predict the state. This simulates a navigation
 * state of (pose, velocity, position)
 *
 * @date Apr 25, 2025
 * @author Scott Baker
 * @author Matt Kielo
 * @author Frank Dellaert
 */
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LIEKF.h>

#include <iostream>

using namespace std;
using namespace gtsam;

// Define a dynamics function.
// The dynamics function for NavState returns a result vector of
// size 9 of [angular_velocity, 0, 0, 0, linear_acceleration] as well as
// a Jacobian of the dynamics function with respect to the state X.
// Since this is a left invariant EKF, the error dynamics do not rely on the
// state
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

// define a GPS measurement processor. The GPS measurement processor returns
// the expected measurement h(x) = translation of X with a Jacobian H used in
// the update stage of the LIEKF.
Vector3 h_gps(const NavState& X, OptionalJacobian<3, 9> H = {}) {
  if (H) *H << Z_3x3, Z_3x3, X.R();
  return X.t();
}

int main() {
  // Initialize the filter's state, covariance, and time interval values.
  NavState X0;
  Matrix9 P0 = Matrix9::Identity() * 0.1;
  double dt = 1.0;

  // Create the filter with the initial state, covariance, and dynamics and
  // measurement functions.
  LIEKF<NavState> ekf(X0, P0);

  // Create the process covariance and measurement covariance matrices Q and R.
  Matrix9 Q = Matrix9::Identity() * 0.01;
  Matrix3 R = Matrix3::Identity() * 0.5;

  // Create the IMU measurements of the form (linear_acceleration,
  // angular_velocity)
  Vector6 imu1, imu2;
  imu1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  imu2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Create the GPS measurements of the form (px, py, pz)
  Vector3 z1, z2;
  z1 << 0.0, 0.0, 0.0;
  z2 << 0.0, 0.0, 0.0;

  // Run the predict and update stages, and print their results.
  cout << "\nInitialization:\n";
  cout << "X0: " << ekf.state() << endl;
  cout << "P0: " << ekf.covariance() << endl;

  // First prediction stage
  ekf.predict(dynamics, imu1, dt, Q);
  cout << "\nFirst Prediction:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << ekf.covariance() << endl;

  // First update stage
  ekf.update(h_gps, z1, R);
  cout << "\nFirst Update:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << ekf.covariance() << endl;

  // Second prediction stage
  ekf.predict(dynamics, imu2, dt, Q);
  cout << "\nSecond Prediction:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << ekf.covariance() << endl;

  // Second update stage
  ekf.update(h_gps, z2, R);
  cout << "\nSecond Update:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << ekf.covariance() << endl;

  return 0;
}