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
  double measureZ(const Unit3& p, OptionalJacobian<1, 2> H) {
    if (H) {
      // H = d(p.point3().z()) / d(local(p))
      // Calculate numerically for simplicity in test
      auto h = [](const Unit3& p_) { return p_.point3().z(); };
      *H = numericalDerivative11<double, Unit3, 2>(h, p);
    }
    return p.point3().z();
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
  double z_true = exampleUnit3::measureZ(p_start, {});
  double z_observed = z_true + 0.02; // Add some noise

  // --- Perform EKF update ---
  ekf.update(exampleUnit3::measureZ, z_observed, data.R);

  // --- Verification (Manual Kalman Update Steps) ---
  // 1. Predict measurement and get Jacobian H
  Matrix12 H; // Note: Jacobian is 1x2 for Unit3
  double z_pred = exampleUnit3::measureZ(p_start, H);

  // 2. Innovation and Covariance
  double y = z_pred - z_observed; // Innovation (using vector subtraction for z)
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
  // For Matrix (a VectorSpace type), Retract(M, v) is typically M + v.
  Matrix predictNextState(const Matrix& p, const Vector& vTangent, double dt) {
    return traits<Matrix>::Retract(p, vTangent * dt);
  }

  // Define a measurement model: measure the trace of the Matrix (assumed 2x2 here)
  // H is the Jacobian dh/d(flatten(p))
  double measureTrace(const Matrix& p, OptionalJacobian<-1, -1> H_opt = {}) {
    if (H_opt) {
      // The Jacobian H = d(trace)/d(flatten(p)) will be 1xN, where N is p.size().
      // For a 2x2 matrix, N=4, so H is 1x4: [1, 0, 0, 1].
      Matrix H = Matrix::Zero(1, p.size());
      if (p.rows() >= 2 && p.cols() >= 2) { // Ensure it's at least 2x2 for trace definition
        H(0, 0) = 1.0; // d(trace)/dp00
        H(0, p.rows() * (p.cols() - 1) + (p.rows() - 1)) = 1.0; // d(trace)/dp11 (for col-major)
        // For 2x2: H(0, 2*1 + 1) = H(0,3)
      }
      *H_opt = H;
    }
    if (p.rows() < 1 || p.cols() < 1) return 0.0; // Or throw error
    double trace = 0.0;
    for (DenseIndex i = 0; i < std::min(p.rows(), p.cols()); ++i) {
      trace += p(i, i);
    }
    return trace;
  }

} // namespace exampleDynamicMatrix

TEST(ManifoldEKF_DynamicMatrix, CombinedPredictAndUpdate) {
  Matrix p_initial = (Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished();
  Matrix P_initial = I_4x4 * 0.01; // Covariance for 2x2 matrix (4x4)
  Vector v_tangent = (Vector(4) << 0.5, 0.1, -0.1, -0.5).finished(); // [dp00, dp10, dp01, dp11]/sec
  double dt = 0.1;
  Matrix Q = I_4x4 * 0.001; // Process noise covariance (4x4)
  Matrix R = Matrix::Identity(1, 1) * 0.005; // Measurement noise covariance (1x1)

  ManifoldEKF<Matrix> ekf(p_initial, P_initial);
  // For a 2x2 Matrix, tangent space dimension is 2*2=4.
  EXPECT_LONGS_EQUAL(4, ekf.state().size());
  EXPECT_LONGS_EQUAL(p_initial.rows() * p_initial.cols(), ekf.state().size());

  // Predict Step
  Matrix p_predicted_mean = exampleDynamicMatrix::predictNextState(p_initial, v_tangent, dt);

  // For this linear prediction model (p_next = p_current + V*dt in tangent space),
  // Derivative w.r.t delta_xi is Identity.
  Matrix F = I_4x4;

  ekf.predict(p_predicted_mean, F, Q);

  EXPECT(assert_equal(p_predicted_mean, ekf.state(), 1e-9));
  Matrix P_predicted_expected = F * P_initial * F.transpose() + Q;
  EXPECT(assert_equal(P_predicted_expected, ekf.covariance(), 1e-9));

  // Update Step
  Matrix p_current_for_update = ekf.state();
  Matrix P_current_for_update = ekf.covariance();

  // True trace of p_current_for_update (which is p_predicted_mean)
  // p_predicted_mean = p_initial + v_tangent*dt
  // = [1.0, 2.0; 3.0, 4.0] + [0.05, -0.01; 0.01, -0.05] (v_tangent reshaped to 2x2 col-major)
  // Trace = 1.05 + 3.95 = 5.0
  double z_true = exampleDynamicMatrix::measureTrace(p_current_for_update);
  EXPECT_DOUBLES_EQUAL(5.0, z_true, 1e-9);
  double z_observed = z_true - 0.03;

  ekf.update(exampleDynamicMatrix::measureTrace, z_observed, R);

  // Manual Kalman Update Steps for Verification
  Matrix H(1, 4); // Measurement Jacobian H (1x4 for 2x2 matrix, trace measurement)
  double z_prediction_manual = exampleDynamicMatrix::measureTrace(p_current_for_update, H);
  Matrix H_expected = (Matrix(1, 4) << 1.0, 0.0, 0.0, 1.0).finished();
  EXPECT(assert_equal(H_expected, H, 1e-9));


  // Innovation: y = z_observed - z_prediction_manual (since measurement is double)
  double y_innovation = z_observed - z_prediction_manual;
  Matrix S = H * P_current_for_update * H.transpose() + R; // S is 1x1

  Matrix K_gain = P_current_for_update * H.transpose() * S.inverse(); // K is 4x1

  // State Correction (in tangent space of Matrix)
  Vector delta_xi_tangent = K_gain * y_innovation; // delta_xi is 4x1 Vector

  Matrix p_updated_manual_expected = traits<Matrix>::Retract(p_current_for_update, delta_xi_tangent);
  Matrix P_updated_manual_expected = (I_4x4 - K_gain * H) * P_current_for_update;

  EXPECT(assert_equal(p_updated_manual_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P_updated_manual_expected, ekf.covariance(), 1e-9));
}



int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}