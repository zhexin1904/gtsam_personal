
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SE2LIEKF_NoFactors.cpp
 *
 * A simple left invariant EKF operating in SE(2) using an odometry and GPS measurements.
 * No factors are used here.
 * This data was compared to a left invariant EKF on SE(2) using identical measurements and noise from the source of the
 * InEKF plugin https://inekf.readthedocs.io/en/latest/
 * Based on the paper "An Introduction to the Invariant Extended Kalman Filter" by
 * Easton R. Potokar, Randal W. Beard, and Joshua G. Mangelson
 * @date April 1, 2025
 * @author Scott Baker
 */



#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <Eigen/Dense>
#include <gtsam/base/Matrix.h>

using namespace std;
using namespace gtsam;

#define PI    3.14159265358

// Create an adjoint class
// Function for the prediction stage.
// Note that U is the exponential map of some control vector (u*dt)
  void predict(Pose2& X, Matrix& P,  Vector3& u, Matrix& Q)
      {
    //Pose2 U = Pose2::Expmap(u*dt);
    Pose2 U(u(0), u(1), u(2));
    Matrix Adj = U.inverse().AdjointMap();
    X = X.compose(U);
    P = Adj * P * Adj.transpose() + Q;
    }

  // Update stage that uses GPS measurements in a left invariant update way.
  void update(Pose2& X, Matrix& P, Point2& z, Matrix& H, Matrix& R)
        {
    // find residual R^T * (z - zhat)
    Point2 zhat = X.translation();
    Point2 e_world = z-zhat;
    Point2 residual = X.rotation().unrotate(e_world);
    Matrix S = H*P*H.transpose() + R;
    Matrix K = P*H.transpose()*S.inverse();
    Vector3 delta = K*residual;
    X = X * Pose2::Expmap(delta);
    P = (Matrix3::Identity() - K*H) * P;
    }



int main() {

  // Define movements and measurements
  const double dt = 1.0;

  Vector3 u1(1.0, 1.0, 0.5), u2(1.0,1.0,0.0); // odometry
  Point2 z1(1.0, 0.0), z2(1.0, 1.0); // gps



  // Set up noise models (uses simple GPS)

  // Odometry Process
  Vector3 pModel(0.05, 0.05, 0.001);    // covariance of the odometry process (rad)
  Matrix Q = pModel.asDiagonal();       // Q covariance matrix

  // GPS process
  Vector2 rModel(0.01, 0.01);
  Matrix R = rModel.asDiagonal();
  Matrix H(2,3);
  H << 1,0,0,
      0,1,0;

  // A matrix that transforms the P matrix into a form of [theta, x, y] for comparison to InEKF's structure.
  Matrix3 TransformP;
  TransformP << 0, 0, 1,
      1,0,0,
      0,1,0;

// Initialize
  Pose2 X(0, 0, 0);
  Matrix P = Matrix3::Identity()*0.1;

  cout << "Initial X\n" << X << endl;
  cout << "Initial P\n" << TransformP * P * TransformP.transpose() << endl;

 // First Prediction
  predict(X, P, u1, dt, Q);
  cout << "Predicted X1\n" << X << endl;
  cout << "Predicted P1\n" << TransformP * P * TransformP.transpose() << endl;

  // First Update
  update(X, P, z1, H, R);
  cout << "Updated X1\n" << X << endl;
  cout << "Updated P1\n" << TransformP * P * TransformP.transpose() << endl;

  // Second Prediction
  predict(X, P, u2, dt, Q);
  cout << "Predicted X2\n" << X << endl;
  cout << "Predicted P2\n" << TransformP * P * TransformP.transpose() << endl;

  // Second Update
  update(X, P, z2, H, R);
  cout << "Updated X2\n" << X << endl;
  cout << "Updated P2\n" << TransformP * P * TransformP.transpose() << endl;

  return 0;
  }

