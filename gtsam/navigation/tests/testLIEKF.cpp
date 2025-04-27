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

// Duplicate the dynamics function in LIEKF_Rot3Example
namespace example {
static constexpr double k = 0.5;
Vector3 dynamics(const Rot3& X, OptionalJacobian<3, 3> H = {}) {
  // φ = Logmap(R), Dφ = ∂φ/∂δR
  Matrix3 Dφ;
  Vector3 φ = Rot3::Logmap(X, Dφ);
  // zero out yaw
  φ[2] = 0.0;
  Dφ.row(2).setZero();

  if (H) *H = -k * Dφ;  // ∂(–kφ)/∂δR
  return -k * φ;        // xi ∈ 𝔰𝔬(3)
}
}  // namespace example

TEST(LIEKFNavState, dynamicsJacobian) {
  // Construct a nontrivial state and IMU input
  Rot3 R = Rot3::RzRyRx(0.1, -0.2, 0.3);

  // Analytic Jacobian (always zero for left-invariant dynamics)
  Matrix3 actualH;
  example::dynamics(R, actualH);

  // Numeric Jacobian w.r.t. the state X
  auto f = [&](const Rot3& X_) { return example::dynamics(X_); };
  Matrix3 expectedH = numericalDerivative11<Vector3, Rot3>(f, R, 1e-6);

  // Compare
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
