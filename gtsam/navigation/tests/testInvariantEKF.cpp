/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

 /**
  * @file testInvariantEKF_Pose2.cpp
  * @brief Unit test for the InvariantEKF using Pose2 state.
  *        Based on the logic from IEKF_SE2Example.cpp
  * @date April 26, 2025
  * @authors Frank Dellaert (adapted from example by Scott Baker, Matt Kielo)
  */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/navigation/InvariantEKF.h>

#include <iostream>

using namespace std;
using namespace gtsam;

// Create a 2D GPS measurement function that returns the predicted measurement h
// and Jacobian H. The predicted measurement h is the translation of the state X.
// (Copied from IEKF_SE2Example.cpp)
Vector2 h_gps(const Pose2& X, OptionalJacobian<2, 3> H = {}) {
  return X.translation(H);
}


TEST(IEKF_Pose2, PredictUpdateSequence) {
  // GIVEN: Initial state, covariance, noises, motions, measurements
  // (from IEKF_SE2Example.cpp)
  Pose2 X0(0.0, 0.0, 0.0);
  Matrix3 P0 = Matrix3::Identity() * 0.1;

  Matrix3 Q = (Vector3(0.05, 0.05, 0.001)).asDiagonal();
  Matrix2 R = I_2x2 * 0.01;

  Pose2 U1(1.0, 1.0, 0.5), U2(1.0, 1.0, 0.0);

  Vector2 z1, z2;
  z1 << 1.0, 0.0;
  z2 << 1.0, 1.0;

  // Create the filter
  InvariantEKF<Pose2> ekf(X0, P0);
  EXPECT(assert_equal(X0, ekf.state()));
  EXPECT(assert_equal(P0, ekf.covariance()));

  // --- First Prediction ---
  ekf.predict(U1, Q);

  // Calculate expected state and covariance
  Pose2 X1_expected = X0.compose(U1);
  Matrix3 Ad_U1_inv = U1.inverse().AdjointMap();
  Matrix3 P1_expected = Ad_U1_inv * P0 * Ad_U1_inv.transpose() + Q;

  // Verify
  EXPECT(assert_equal(X1_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P1_expected, ekf.covariance(), 1e-9));


  // --- First Update ---
  ekf.update(h_gps, z1, R);

  // Calculate expected state and covariance (manual Kalman steps)
  Matrix H1; // H = dh/dlocal(X) -> 2x3
  Vector2 z_pred1 = h_gps(X1_expected, H1);
  Vector2 y1 = z_pred1 - z1; // Innovation
  Matrix S1 = H1 * P1_expected * H1.transpose() + R;
  Matrix K1 = P1_expected * H1.transpose() * S1.inverse(); // Gain (3x2)
  Vector3 delta_xi1 = -K1 * y1; // Correction (tangent space)
  Pose2 X1_updated_expected = X1_expected.retract(delta_xi1);
  Matrix3 I_KH1 = Matrix3::Identity() - K1 * H1;
  Matrix3 P1_updated_expected = I_KH1 * P1_expected; // Standard form P = (I-KH)P

  // Verify
  EXPECT(assert_equal(X1_updated_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P1_updated_expected, ekf.covariance(), 1e-9));


  // --- Second Prediction ---
  ekf.predict(U2, Q);

  // Calculate expected state and covariance
  Pose2 X2_expected = X1_updated_expected.compose(U2);
  Matrix3 Ad_U2_inv = U2.inverse().AdjointMap();
  Matrix3 P2_expected = Ad_U2_inv * P1_updated_expected * Ad_U2_inv.transpose() + Q;

  // Verify
  EXPECT(assert_equal(X2_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P2_expected, ekf.covariance(), 1e-9));


  // --- Second Update ---
  ekf.update(h_gps, z2, R);

  // Calculate expected state and covariance (manual Kalman steps)
  Matrix H2; // 2x3
  Vector2 z_pred2 = h_gps(X2_expected, H2);
  Vector2 y2 = z_pred2 - z2; // Innovation
  Matrix S2 = H2 * P2_expected * H2.transpose() + R;
  Matrix K2 = P2_expected * H2.transpose() * S2.inverse(); // Gain (3x2)
  Vector3 delta_xi2 = -K2 * y2; // Correction (tangent space)
  Pose2 X2_updated_expected = X2_expected.retract(delta_xi2);
  Matrix3 I_KH2 = Matrix3::Identity() - K2 * H2;
  Matrix3 P2_updated_expected = I_KH2 * P2_expected; // Standard form

  // Verify
  EXPECT(assert_equal(X2_updated_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P2_updated_expected, ekf.covariance(), 1e-9));

}

// Define simple dynamics and measurement for a 2x2 Matrix state
namespace exampleDynamicMatrix {
  // Predicts the next state given current state (Matrix), tangent "velocity" (Vector), and dt.
  // This is mainly for verification; IEKF predict will use tangent vector directly.
  Matrix predictNextStateManually(const Matrix& p, const Vector& vTangent, double dt) {
    return traits<Matrix>::Retract(p, vTangent * dt);
  }
  // Define a measurement model: measure the trace of the Matrix (assumed 2x2 here)
  double measureTrace(const Matrix& p, OptionalJacobian<-1, -1> H = {}) {
    if (H) {
      // p_flat (col-major for Eigen) for a 2x2 matrix p = [[p00,p01],[p10,p11]] is [p00, p10, p01, p11]
      // trace = p(0,0) + p(1,1)
      // H = d(trace)/d(p_flat) = [1, 0, 0, 1]
      // The Jacobian H will be 1x4 for a 2x2 matrix.
      H->resize(1, p.size()); // p.size() is rows*cols
      (*H) << 1.0, 0.0, 0.0, 1.0; // Assuming 2x2, so 1x4
    }
    return p(0, 0) + p(1, 1);
  }
} // namespace exampleDynamicMatrix

// Test fixture for InvariantEKF with a 2x2 Matrix state
struct DynamicMatrixEKFTest {
  Matrix p0Matrix; // Initial state (as 2x2 Matrix)
  Matrix p0Covariance; // Initial covariance (dynamic Matrix, 4x4)
  Vector velocityTangent; // Control input in tangent space (Vector4 for 2x2 matrix)
  double dt;
  Matrix processNoiseCovariance; // Process noise covariance (dynamic Matrix, 4x4)
  Matrix measurementNoiseCovariance; // Measurement noise covariance (dynamic Matrix, 1x1)
  DynamicMatrixEKFTest() :
    p0Matrix((Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished()),
    p0Covariance(I_4x4 * 0.01),
    velocityTangent((Vector(4) << 0.5, 0.1, -0.1, -0.5).finished()), // [dp00, dp10, dp01, dp11]/sec
    dt(0.1),
    processNoiseCovariance(I_4x4 * 0.001),
    measurementNoiseCovariance(Matrix::Identity(1, 1) * 0.005) {
  }
};

TEST(InvariantEKF_DynamicMatrix, Predict) {
  DynamicMatrixEKFTest data;
  InvariantEKF<Matrix> ekf(data.p0Matrix, data.p0Covariance);
  // For a 2x2 Matrix, tangent space dimension is 2*2=4.
  EXPECT_LONGS_EQUAL(4, ekf.state().size());
  EXPECT_LONGS_EQUAL(data.p0Matrix.rows() * data.p0Matrix.cols(), ekf.state().size());
  EXPECT_LONGS_EQUAL(4, ekf.dimension());
  // --- Perform EKF prediction using InvariantEKF::predict(tangentVector, dt, Q) ---
  ekf.predict(data.velocityTangent, data.dt, data.processNoiseCovariance);
  // --- Verification ---
  // 1. Calculate expected next state
  Matrix pNextExpected = exampleDynamicMatrix::predictNextStateManually(data.p0Matrix, data.velocityTangent, data.dt);
  EXPECT(assert_equal(pNextExpected, ekf.state(), 1e-9));
  // 2. Calculate expected covariance
  // For VectorSpace, AdjointMap is Identity. So P_next = P_prev + Q.
  Matrix pCovarianceExpected = data.p0Covariance + data.processNoiseCovariance;
  EXPECT(assert_equal(pCovarianceExpected, ekf.covariance(), 1e-9));
}

TEST(InvariantEKF_DynamicMatrix, Update) {
  DynamicMatrixEKFTest data;
  Matrix pStartMatrix = (Matrix(2, 2) << 1.5, -0.5, 0.8, 2.5).finished();
  Matrix pStartCovariance = I_4x4 * 0.02;
  InvariantEKF<Matrix> ekf(pStartMatrix, pStartCovariance);
  EXPECT_LONGS_EQUAL(4, ekf.state().size());
  EXPECT_LONGS_EQUAL(4, ekf.dimension());
  // Simulate a measurement (true trace of pStartMatrix is 1.5 + 2.5 = 4.0)
  double zTrue = exampleDynamicMatrix::measureTrace(pStartMatrix); // No Jacobian needed here
  double zObserved = zTrue - 0.03; // Add some "error"
  // --- Perform EKF update ---
  // The update method is inherited from ManifoldEKF.
  ekf.update(exampleDynamicMatrix::measureTrace, zObserved, data.measurementNoiseCovariance);
  // --- Verification (Manual Kalman Update Steps) ---
  // 1. Predict measurement and get Jacobian H
  Matrix H_manual(1, 4); // This will be 1x4 for a 2x2 matrix measurement
  double zPredictionManual = exampleDynamicMatrix::measureTrace(pStartMatrix, H_manual);
  // 2. Innovation and Innovation Covariance
  // EKF calculates innovation_tangent = traits<Measurement>::Local(prediction, zObserved)
  // For double (a VectorSpace), Local(A,B) is B-A. So, zObserved - zPredictionManual.
  double innovationY_tangent = zObserved - zPredictionManual;
  Matrix innovationCovarianceS = H_manual * pStartCovariance * H_manual.transpose() + data.measurementNoiseCovariance;
  // 3. Kalman Gain K
  Matrix kalmanGainK = pStartCovariance * H_manual.transpose() * innovationCovarianceS.inverse(); // K is 4x1
  // 4. State Correction (in tangent space of Matrix)
  Vector deltaXiTangent = kalmanGainK * innovationY_tangent; // deltaXi is 4x1 Vector
  // 5. Expected Updated State and Covariance (using Joseph form)
  Matrix pUpdatedExpected = traits<Matrix>::Retract(pStartMatrix, deltaXiTangent);
  Matrix I_KH = I_4x4 - kalmanGainK * H_manual;
  Matrix pUpdatedCovarianceExpected = I_KH * pStartCovariance * I_KH.transpose() + kalmanGainK * data.measurementNoiseCovariance * kalmanGainK.transpose();
  // --- Compare EKF result with manual calculation ---
  EXPECT(assert_equal(pUpdatedExpected, ekf.state(), 1e-9));
  EXPECT(assert_equal(pUpdatedCovarianceExpected, ekf.covariance(), 1e-9));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}