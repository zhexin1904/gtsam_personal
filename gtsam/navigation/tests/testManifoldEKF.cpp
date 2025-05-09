/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

 /**
  * @file testManifoldEKF.cpp
  * @brief Unit test for the ManifoldEKF base class using Unit3.
  * @date April 26, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ManifoldEKF.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace gtsam;

// Define simple dynamics for Unit3: constant velocity in the tangent space
namespace exampleUnit3 {

  // Predicts the next state given current state, tangent velocity, and dt
  Unit3 predictNextState(const Unit3& p, const Vector2& v, double dt) {
    return p.retract(v * dt);
  }

  // Define a measurement model: measure the z-component of the Unit3 direction
  // H is the Jacobian dh/d(local(p))
  Vector1 measureZ(const Unit3& p, OptionalJacobian<1, 2> H) {
    if (H) {
      // H = d(p.point3().z()) / d(local(p))
      // Calculate numerically for simplicity in test
      auto h = [](const Unit3& p_) { return Vector1(p_.point3().z()); };
      *H = numericalDerivative11<Vector1, Unit3, 2>(h, p);
    }
    return Vector1(p.point3().z());
  }

} // namespace exampleUnit3

// Test fixture for ManifoldEKF with Unit3
struct Unit3EKFTest {
  Unit3 p0;
  Matrix2 P0;
  Vector2 velocity;
  double dt;
  Matrix2 Q; // Process noise
  Matrix1 R; // Measurement noise

  Unit3EKFTest() :
    p0(Unit3(Point3(1, 0, 0))), // Start pointing along X-axis
    P0(I_2x2 * 0.01),
    velocity((Vector2() << 0.0, M_PI / 4.0).finished()), // Rotate towards +Z axis
    dt(0.1),
    Q(I_2x2 * 0.001),
    R(Matrix1::Identity() * 0.01) {
  }
};


TEST(ManifoldEKF_Unit3, Predict) {
  Unit3EKFTest data;

  // Initialize the EKF
  ManifoldEKF<Unit3> ekf(data.p0, data.P0);

  // --- Prepare inputs for ManifoldEKF::predict ---
  // 1. Compute expected next state
  Unit3 p_next_expected = exampleUnit3::predictNextState(data.p0, data.velocity, data.dt);

  // 2. Compute state transition Jacobian F = d(local(p_next)) / d(local(p))
  //    We can compute this numerically using the predictNextState function.
  //    GTSAM's numericalDerivative handles derivatives *between* manifolds.
  auto predict_wrapper = [&](const Unit3& p) -> Unit3 {
    return exampleUnit3::predictNextState(p, data.velocity, data.dt);
    };
  Matrix2 F = numericalDerivative11<Unit3, Unit3>(predict_wrapper, data.p0);

  // --- Perform EKF prediction ---
  ekf.predict(p_next_expected, F, data.Q);

  // --- Verification ---
  // Check state
  EXPECT(assert_equal(p_next_expected, ekf.state(), 1e-8));

  // Check covariance
  Matrix2 P_expected = F * data.P0 * F.transpose() + data.Q;
  EXPECT(assert_equal(P_expected, ekf.covariance(), 1e-8));

  // Check F manually for a simple case (e.g., zero velocity should give Identity)
  Vector2 zero_velocity = Vector2::Zero();
  auto predict_wrapper_zero = [&](const Unit3& p) -> Unit3 {
    return exampleUnit3::predictNextState(p, zero_velocity, data.dt);
    };
  Matrix2 F_zero = numericalDerivative11<Unit3, Unit3>(predict_wrapper_zero, data.p0);
  EXPECT(assert_equal<Matrix2>(I_2x2, F_zero, 1e-8));

}

TEST(ManifoldEKF_Unit3, Update) {
  Unit3EKFTest data;

  // Use a slightly different starting point and covariance for variety
  Unit3 p_start = Unit3(Point3(0, 1, 0)).retract((Vector2() << 0.1, 0).finished()); // Perturb pointing along Y
  Matrix2 P_start = I_2x2 * 0.05;
  ManifoldEKF<Unit3> ekf(p_start, P_start);

  // Simulate a measurement (e.g., true value + noise)
  Vector1 z_true = exampleUnit3::measureZ(p_start, {});
  Vector1 z_observed = z_true + Vector1(0.02); // Add some noise

  // --- Perform EKF update ---
  ekf.update(exampleUnit3::measureZ, z_observed, data.R);

  // --- Verification (Manual Kalman Update Steps) ---
  // 1. Predict measurement and get Jacobian H
  Matrix12 H; // Note: Jacobian is 1x2 for Unit3
  Vector1 z_pred = exampleUnit3::measureZ(p_start, H);

  // 2. Innovation and Covariance
  Vector1 y = z_pred - z_observed; // Innovation (using vector subtraction for z)
  Matrix1 S = H * P_start * H.transpose() + data.R; // 1x1 matrix

  // 3. Kalman Gain K
  Matrix K = P_start * H.transpose() * S.inverse(); // 2x1 matrix

  // 4. State Correction (in tangent space)
  Vector2 delta_xi = -K * y; // 2x1 vector

  // 5. Expected Updated State and Covariance
  Unit3 p_updated_expected = p_start.retract(delta_xi);
  Matrix2 I_KH = I_2x2 - K * H;
  Matrix2 P_updated_expected = I_KH * P_start;

  // --- Compare EKF result with manual calculation ---
  EXPECT(assert_equal(p_updated_expected, ekf.state(), 1e-8));
  EXPECT(assert_equal(P_updated_expected, ekf.covariance(), 1e-8));
}

// Define simple dynamics and measurement for a 2x2 Matrix state
namespace exampleDynamicMatrix {

  // Predicts the next state given current state (Matrix), tangent "velocity" (Vector), and dt.
  Matrix predictNextState(const Matrix& p, const Vector& vTangent, double dt) {
    return traits<Matrix>::Retract(p, vTangent * dt);
  }

  // Define a measurement model: measure the trace of the Matrix (assumed 2x2 here)
  Vector1 measureTrace(const Matrix& p, OptionalJacobian<-1, -1> H = {}) {
    if (H) {
      // p_flat (col-major for Eigen) for a 2x2 matrix p = [[p00,p01],[p10,p11]] is [p00, p10, p01, p11]
      // trace = p(0,0) + p(1,1)
      // H = d(trace)/d(p_flat) = [1, 0, 0, 1]
      // The Jacobian H will be 1x4 for a 2x2 matrix.
      H->resize(1, 4);
      *H << 1.0, 0.0, 0.0, 1.0;
    }
    return Vector1(p(0, 0) + p(1, 1));
  }

} // namespace exampleDynamicMatrix

// Test fixture for ManifoldEKF with a 2x2 Matrix state
struct DynamicMatrixEKFTest {
  Matrix p0Matrix;                 // Initial state (as 2x2 Matrix)
  Matrix p0Covariance;             // Initial covariance (dynamic Matrix, 4x4)
  Vector velocityTangent;          // Control input in tangent space (Vector4 for 2x2 matrix)
  double dt;
  Matrix processNoiseCovariance;   // Process noise covariance (dynamic Matrix, 4x4)
  Matrix measurementNoiseCovariance; // Measurement noise covariance (dynamic Matrix, 1x1)

  DynamicMatrixEKFTest() :
    p0Matrix((Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished()),
    p0Covariance(I_4x4 * 0.01),
    velocityTangent((Vector(4) << 0.5, 0.1, -0.1, -0.5).finished()), // [dp00, dp10, dp01, dp11]/sec
    dt(0.1),
    processNoiseCovariance(I_4x4 * 0.001),
    measurementNoiseCovariance(Matrix::Identity(1, 1) * 0.005)
  {
  }
};


TEST(ManifoldEKF_DynamicMatrix, Predict) {
  DynamicMatrixEKFTest data;

  ManifoldEKF<Matrix> ekf(data.p0Matrix, data.p0Covariance);
  // For a 2x2 Matrix, tangent space dimension is 2*2=4.
  EXPECT_LONGS_EQUAL(4, ekf.state().size());
  EXPECT_LONGS_EQUAL(data.p0Matrix.rows() * data.p0Matrix.cols(), ekf.state().size());

  // --- Prepare inputs for ManifoldEKF::predict ---
  Matrix pNextExpected = exampleDynamicMatrix::predictNextState(data.p0Matrix, data.velocityTangent, data.dt);

  // For this linear prediction model (p_next = p_current + V*dt in tangent space), F is Identity.
  Matrix jacobianF = I_4x4; // Jacobian of the state transition function

  // --- Perform EKF prediction ---
  ekf.predict(pNextExpected, jacobianF, data.processNoiseCovariance);

  // --- Verification ---
  EXPECT(assert_equal(pNextExpected, ekf.state(), 1e-9));
  Matrix pCovarianceExpected = jacobianF * data.p0Covariance * jacobianF.transpose() + data.processNoiseCovariance;
  EXPECT(assert_equal(pCovarianceExpected, ekf.covariance(), 1e-9));
}

TEST(ManifoldEKF_DynamicMatrix, Update) {
  DynamicMatrixEKFTest data;

  Matrix pStartMatrix = (Matrix(2, 2) << 1.5, -0.5, 0.8, 2.5).finished();
  Matrix pStartCovariance = I_4x4 * 0.02;
  ManifoldEKF<Matrix> ekf(pStartMatrix, pStartCovariance);
  EXPECT_LONGS_EQUAL(4, ekf.state().size());

  // Simulate a measurement (true trace of pStartMatrix is 1.5 + 2.5 = 4.0)
  Vector1 zTrue = exampleDynamicMatrix::measureTrace(pStartMatrix); // No Jacobian needed here
  Vector1 zObserved = zTrue - Vector1(0.03); // Add some "error"

  // --- Perform EKF update ---
  ekf.update(exampleDynamicMatrix::measureTrace, zObserved, data.measurementNoiseCovariance);

  // --- Verification (Manual Kalman Update Steps) ---
  // 1. Predict measurement and get Jacobian H
  Matrix H(1, 4); // This will be 1x4 for a 2x2 matrix measurement
  Vector1 zPredictionManual = exampleDynamicMatrix::measureTrace(pStartMatrix, H);

  // 2. Innovation and Innovation Covariance
  // EKF calculates innovation_tangent = traits<Measurement>::Local(prediction, zObserved)
  // For Vector1 (a VectorSpace), Local(A,B) is B-A. So, zObserved - zPredictionManual.
  Vector1 innovationY = zObserved - zPredictionManual;
  Matrix innovationCovarianceS = H * pStartCovariance * H.transpose() + data.measurementNoiseCovariance;

  // 3. Kalman Gain K
  Matrix kalmanGainK = pStartCovariance * H.transpose() * innovationCovarianceS.inverse(); // K is 4x1

  // 4. State Correction (in tangent space of Matrix)
  Vector deltaXiTangent = kalmanGainK * innovationY; // deltaXi is 4x1 Vector

  // 5. Expected Updated State and Covariance
  Matrix pUpdatedExpected = traits<Matrix>::Retract(pStartMatrix, deltaXiTangent);

  // Covariance update: P_new = (I - K H) P_old
  Matrix pUpdatedCovarianceExpected = (I_4x4 - kalmanGainK * H) * pStartCovariance;

  // --- Compare EKF result with manual calculation ---
  EXPECT(assert_equal(pUpdatedExpected, ekf.state(), 1e-9));
  EXPECT(assert_equal(pUpdatedCovarianceExpected, ekf.covariance(), 1e-9));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}