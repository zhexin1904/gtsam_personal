/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LIEKF_SE2_SimpleGPSExample.cpp
 * @brief A left invariant Extended Kalman Filter example using a Lie group
 * odometry as the prediction stage on SE(2) and
 *
 * This example uses the templated LIEKF class to estimate the state of
 * an object using odometry / GPS measurements The prediction stage of the LIEKF
 * uses a Lie Group element to propagate the stage in a discrete LIEKF. For most
 * cases, U = exp(u^ * dt) if u is a control vector that is constant over the
 * interval dt. However, if u is not constant over dt, other approaches are
 * needed to find the value of U. This approach simply takes a Lie group element
 * U, which can be found in various different ways.
 *
 * This data was compared to a left invariant EKF on SE(2) using identical
 * measurements and noise from the source of the InEKF plugin
 * https://inekf.readthedocs.io/en/latest/ Based on the paper "An Introduction
 * to the Invariant Extended Kalman Filter" by Easton R. Potokar, Randal W.
 * Beard, and Joshua G. Mangelson
 *
 * @date Apr 25, 2025
 * @author Scott Baker
 * @author Matt Kielo
 * @author Frank Dellaert
 */
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LIEKF.h>

#include <iostream>

using namespace std;
using namespace gtsam;

// Create a 2D GPS measurement function that returns the predicted measurement h
// and Jacobian H. The predicted measurement h is the translation of the state
// X.
Vector2 h_gps(const Pose2& X, OptionalJacobian<2, 3> H = {}) {
  return X.translation(H);
}
// Define a LIEKF class that uses the Pose2 Lie group as the state and the
// Vector2 measurement type.
int main() {
  // // Initialize the filter's state, covariance, and time interval values.
  Pose2 X0(0.0, 0.0, 0.0);
  Matrix3 P0 = Matrix3::Identity() * 0.1;
  double dt = 1.0;

  // Create the function measurement_function that wraps h_gps.
  std::function<Vector2(const Pose2&, gtsam::OptionalJacobian<2, 3>)>
      measurement_function = h_gps;
  // Create the filter with the initial state, covariance, and measurement
  // function.
  LIEKF<Pose2, Vector2> ekf(X0, P0, measurement_function);

  // Define the process covariance and measurement covariance matrices Q and R.
  Matrix3 Q = (Vector3(0.05, 0.05, 0.001)).asDiagonal();
  Matrix2 R = (Vector2(0.01, 0.01)).asDiagonal();

  // Define odometry movements.
  // U1: Move 1 unit in X, 1 unit in Y, 0.5 radians in theta.
  // U2: Move 1 unit in X, 1 unit in Y, 0 radians in theta.
  Pose2 U1(1.0, 1.0, 0.5), U2(1.0, 1.0, 0.0);

  // Create GPS measurements.
  // z1: Measure robot at (1, 0)
  // z2: Measure robot at (1, 1)
  Vector2 z1, z2;
  z1 << 1.0, 0.0;
  z2 << 1.0, 1.0;

  // Define a transformation matrix to convert the covariance into (theta, x, y)
  // form. The paper and data mentioned above uses a (theta, x, y) notation,
  // whereas GTSAM uses (x, y, theta). The transformation matrix is used to
  // directly compare results of the covariance matrix.
  Matrix3 TransformP;
  TransformP << 0, 0, 1, 1, 0, 0, 0, 1, 0;

  // Propagating/updating the filter
  // Initial state and covariance
  cout << "\nInitialization:\n";
  cout << "X0: " << ekf.state() << endl;
  cout << "P0: " << TransformP * ekf.covariance() * TransformP.transpose()
       << endl;

  // First prediction stage
  ekf.predict(U1, Q);
  cout << "\nFirst Prediction:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << TransformP * ekf.covariance() * TransformP.transpose()
       << endl;

  // First update stage
  ekf.update(z1, R);
  cout << "\nFirst Update:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << TransformP * ekf.covariance() * TransformP.transpose()
       << endl;

  // Second prediction stage
  ekf.predict(U2, Q);
  cout << "\nSecond Prediction:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << TransformP * ekf.covariance() * TransformP.transpose()
       << endl;

  // Second update stage
  ekf.update(z2, R);
  cout << "\nSecond Update:\n";
  cout << "X: " << ekf.state() << endl;
  cout << "P: " << TransformP * ekf.covariance() * TransformP.transpose()
       << endl;

  return 0;
}