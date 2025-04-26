/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testLIEKFNavState.cpp
 * @brief Unit test for the NavState dynamics Jacobian in LIEKF example.
 * @date April 26, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/NavState.h>

using namespace gtsam;

// Duplicate the dynamics function under test:
namespace example {
Vector9 dynamics(const NavState& X, const Vector6& imu,
                 OptionalJacobian<9, 9> H = {}) {
  auto a = imu.head<3>();
  auto w = imu.tail<3>();
  Vector9 xi;
  xi << w, Vector3::Zero(), a;
  if (H) *H = Matrix9::Zero();
  return xi;
}
}  // namespace example

TEST(LIEKFNavState, dynamicsJacobian) {
  // Construct a nontrivial state and IMU input
  NavState X(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1.0, 2.0, 3.0),
             Vector3(0.5, -0.5, 0.5));
  Vector6 imu;
  imu << 0.1, -0.1, 0.2,  // acceleration
      0.01, -0.02, 0.03;  // angular velocity

  // Analytic Jacobian (always zero for left-invariant dynamics)
  OptionalJacobian<9, 9> H_analytic;
  example::dynamics(X, imu, H_analytic);
  Matrix actualH = *H_analytic;

  // Numeric Jacobian w.r.t. the state X
  auto f = [&](const NavState& X_) { return example::dynamics(X_, imu); };
  Matrix expectedH = numericalDerivative11<Vector9, NavState>(f, X, 1e-6);

  // Compare
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
