/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LIEKF_NavstateExample.cpp
 * @brief Example of a Left-Invariant Extended Kalman Filter on NavState
 *        using IMU (predict) and GPS (update) measurements.
 * @date April 25, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LIEKF.h>

#include <iostream>

using namespace std;
using namespace gtsam;

/**
 * @brief Left-invariant dynamics for NavState.
 * @param X    Current state (unused for left-invariant error dynamics).
 * @param imu  6×1 vector [a; ω]: linear accel (first 3) and angular vel (last
 * 3).
 * @param H    Optional 9×9 Jacobian w.r.t. X (always zero here).
 * @return     9×1 tangent: [ω; 0₃; a].
 */
Vector9 dynamics(const NavState& X, const Vector6& imu,
                 OptionalJacobian<9, 9> H = {}) {
  auto a = imu.head<3>();
  auto w = imu.tail<3>();
  Vector9 xi;
  xi << w, Vector3::Zero(), a;
  if (H) *H = Matrix9::Zero();
  return xi;
}

/**
 * @brief GPS measurement model: returns position and its Jacobian.
 * @param X    Current state.
 * @param H    Optional 3×9 Jacobian w.r.t. X.
 * @return     3×1 position vector.
 */
Vector3 h_gps(const NavState& X, OptionalJacobian<3, 9> H = {}) {
  if (H) {
    // H = [ 0₃×3, 0₃×3, R ]
    *H << Z_3x3, Z_3x3, X.R();
  }
  return X.t();
}

int main() {
  // Initial state, covariance, and time step
  NavState X0;
  Matrix9 P0 = Matrix9::Identity() * 0.1;
  double dt = 1.0;

  // Create the filter with the initial state and covariance.
  LIEKF<NavState> ekf(X0, P0);

  // Process & measurement noise
  Matrix9 Q = Matrix9::Identity() * 0.01;
  Matrix3 R = Matrix3::Identity() * 0.5;

  // Create the IMU measurements of the form (linear_acceleration,
  // angular_velocity)
  Vector6 imu1, imu2;
  imu1 << 0.1, 0.0, 0.0, 0.0, 0.2, 0.0;
  imu2 << 0.0, 0.3, 0.0, 0.4, 0.0, 0.0;

  // Create the GPS measurements of the form (px, py, pz)
  Vector3 z1, z2;
  z1 << 0.3, 0.0, 0.0;
  z2 << 0.6, 0.0, 0.0;

  cout << "=== Initialization ===\n"
       << "X0: " << ekf.state() << "\n"
       << "P0: " << ekf.covariance() << "\n\n";

  ekf.predict(dynamics, imu1, dt, Q);
  cout << "--- After first predict ---\n"
       << "X: " << ekf.state() << "\n"
       << "P: " << ekf.covariance() << "\n\n";

  ekf.update(h_gps, z1, R);
  cout << "--- After first update ---\n"
       << "X: " << ekf.state() << "\n"
       << "P: " << ekf.covariance() << "\n\n";

  ekf.predict(dynamics, imu2, dt, Q);
  cout << "--- After second predict ---\n"
       << "X: " << ekf.state() << "\n"
       << "P: " << ekf.covariance() << "\n\n";

  ekf.update(h_gps, z2, R);
  cout << "--- After second update ---\n"
       << "X: " << ekf.state() << "\n"
       << "P: " << ekf.covariance() << "\n";

  return 0;
}
