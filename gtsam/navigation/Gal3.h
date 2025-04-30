/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    Gal3.h
 * @brief   3D Galilean Group SGal(3) state (attitude, position, velocity, time)
 * Based on manif convention: [R, r, v, t], tangent [rho, nu, theta, t]
 * @author  Based on Python implementation by User
 * @date    April 29, 2025
 */

#pragma once

#include <gtsam/geometry/Pose3.h> // Includes Rot3, Point3
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Lie.h> // For LieGroup base class

#include <cmath> // For std::sqrt, std::cos, std::sin, std::tan
#include <limits> // For std::numeric_limits
#include <functional> // For std::function

namespace gtsam {

// Forward declaration
class Gal3;

// Use Vector3 for velocity for consistency with NavState
using Velocity3 = Vector3;
// Define Vector10 for tangent space
using Vector10 = Eigen::Matrix<double, 10, 1>;
// Define Matrix5 for Lie Algebra matrix representation
using Matrix5 = Eigen::Matrix<double, 5, 5>;
// Define Matrix10 for Jacobians
using Matrix10 = Eigen::Matrix<double, 10, 10>;


/**
 * Represents an element of the 3D Galilean group SGal(3).
 * Aligned with manif conventions: state (R, r, v, t), tangent (rho, nu, theta, t_tan).
 * Internal state:
 * R_: Rotation (Rot3) - attitude (world R body)
 * r_: Translation (Point3) - position in world frame
 * v_: Velocity Boost (Velocity3) - velocity in world frame
 * t_: Time (double)
 *
 * Homogeneous Matrix Representation (manif convention):
 * [ R  v  r ]
 * [ 0  1  t ]
 * [ 0  0  1 ]  (Where R is 3x3, v,r are 3x1, t is scalar, 0 are row vectors)
 *
 * Lie Algebra Matrix Representation (manif convention):
 * [ skew(theta)  nu  rho ]
 * [      0        0   t  ]
 * [      0        0   0  ] (Where skew(theta) is 3x3, nu,rho are 3x1, t is scalar)
 *
 * Tangent Vector xi (Vector10): [rho; nu; theta; t_tan]
 * rho (3x1): Position tangent component
 * nu  (3x1): Velocity tangent component
 * theta(3x1): Rotation tangent component (so(3))
 * t_tan (1x1): Time tangent component
 */
class GTSAM_EXPORT Gal3 : public LieGroup<Gal3, 10> {
 private:
  Rot3 R_;      ///< Rotation world R body
  Point3 r_;    ///< Position in world frame, n_r_b
  Velocity3 v_; ///< Velocity in world frame, n_v_b
  double t_;    ///< Time component

  // Small number epsilon for checking small angles, etc.
  static constexpr double kSmallAngleThreshold = 1e-10;

 public:

  inline constexpr static size_t dimension = 10;

  /// @name Constructors
  /// @{

  /// Default constructor: Identity element
  Gal3() : R_(Rot3::Identity()), r_(Point3::Zero()), v_(Velocity3::Zero()), t_(0.0) {}

  /// Construct from attitude, position, velocity, time (manif order)
  Gal3(const Rot3& R, const Point3& r, const Velocity3& v, double t) :
    R_(R), r_(r), v_(v), t_(t) {}

  /// Construct from a 5x5 matrix representation (manif convention)
  explicit Gal3(const Matrix5& M);

  /// @}
  /// @name Component Access
  /// @{

  const Rot3& rotation(OptionalJacobian<3, 10> H = {}) const;
  const Point3& translation(OptionalJacobian<3, 10> H = {}) const;
  const Velocity3& velocity(OptionalJacobian<3, 10> H = {}) const;
  const double& time(OptionalJacobian<1, 10> H = {}) const;

  // Convenience accessors matching NavState where names overlap
  const Rot3& attitude(OptionalJacobian<3, 10> H = {}) const { return rotation(H); }
  const Point3& position(OptionalJacobian<3, 10> H = {}) const { return translation(H); }

  /// @}
  /// @name Derived quantities
  /// @{

  /// Return rotation matrix.
  Matrix3 R() const { return R_.matrix(); }

  /// Return position as Vector3
  Vector3 r() const { return Vector3(r_); }

  /// Return velocity as Vector3. Computation-free.
  const Vector3& v() const { return v_; }

  /// Return time scalar. Computation-free.
  const double& t() const { return t_; }

  /// Return 5x5 homogeneous matrix representation (manif convention).
  Matrix5 matrix() const;

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const Gal3& state);

  /// print
  void print(const std::string& s = "") const;

  /// equals
  bool equals(const Gal3& other, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static Gal3 Identity() { return Gal3(); }

  /// inverse transformation (Manif formula from SGal3_base.h)
  Gal3 inverse() const;

  // Bring LieGroup::inverse() into scope (version with derivative)
  using LieGroup<Gal3, 10>::inverse;

  /// compose (Manif formula from SGal3_base.h)
  /// Returns this * other
  Gal3 compose(const Gal3& other, OptionalJacobian<10, 10> H1 = {},
               OptionalJacobian<10, 10> H2 = {}) const;

  /// compose syntactic sugar
  Gal3 operator*(const Gal3& other) const {
      return compose(other);
  }

  /// Calculate the relative transformation: this->inverse() * other
  Gal3 between(const Gal3& other, OptionalJacobian<10, 10> H1 = {},
               OptionalJacobian<10, 10> H2 = {}) const;


  /// @}
  /// @name Manifold Operations (Retract/Local)
  /// @{

  /// Default retract uses Expmap
  Gal3 retract(const Vector10& xi, OptionalJacobian<10, 10> H1 = {},
               OptionalJacobian<10, 10> H2 = {}) const;

  /// Default localCoordinates uses Logmap
  Vector10 localCoordinates(const Gal3& other, OptionalJacobian<10, 10> H1 = {},
                            OptionalJacobian<10, 10> H2 = {}) const;

  /// @}
  /// @name Lie Group
  using LieAlgebra = Matrix5;
  /// @{

  // Helper functions for accessing tangent vector components
  // Input xi is [rho(3), nu(3), theta(3), t_tan(1)]
  static Eigen::Block<Vector10, 3, 1> rho(Vector10& v) { return v.segment<3>(0); }
  static Eigen::Block<Vector10, 3, 1> nu(Vector10& v) { return v.segment<3>(3); }
  static Eigen::Block<Vector10, 3, 1> theta(Vector10& v) { return v.segment<3>(6); }
  static Eigen::Block<Vector10, 1, 1> t_tan(Vector10& v) { return v.segment<1>(9); }
  // Const versions too
  static const Eigen::Block<const Vector10, 3, 1> rho(const Vector10& v) { return v.segment<3>(0); }
  static const Eigen::Block<const Vector10, 3, 1> nu(const Vector10& v) { return v.segment<3>(3); }
  static const Eigen::Block<const Vector10, 3, 1> theta(const Vector10& v) { return v.segment<3>(6); }
  static const Eigen::Block<const Vector10, 1, 1> t_tan(const Vector10& v) { return v.segment<1>(9); }

  /// Exponential map at identity - create a Gal3 element from Lie algebra tangent vector xi.
  /// xi = [rho, nu, theta, t_tan] (10x1)
  static Gal3 Expmap(const Vector10& xi, OptionalJacobian<10, 10> Hxi = {});
  // REMOVED using LieGroup<Gal3, 10>::Expmap;

  /// Logarithmic map at identity - return the Lie algebra tangent vector xi for this element.
  /// Returns xi = [rho, nu, theta, t_tan] (10x1)
  static Vector10 Logmap(const Gal3& g, OptionalJacobian<10, 10> Hg = {});
  // REMOVED using LieGroup<Gal3, 10>::Logmap;

  /// Calculate Adjoint map, transforming a twist in the identity frame to the frame of this element.
  Matrix10 AdjointMap() const;

  /// Apply this element's AdjointMap Ad_g to a tangent vector xi, H_g = d(Ad_g * xi)/dg, H_xi = d(Ad_g * xi)/dxi = Ad_g
  Vector10 Adjoint(const Vector10& xi, OptionalJacobian<10, 10> H_g = {}, OptionalJacobian<10, 10> H_xi = {}) const;

  /// The adjoint action of xi on y `ad(xi, y)`
  static Vector10 adjoint(const Vector10& xi, const Vector10& y, OptionalJacobian<10, 10> Hxi = {}, OptionalJacobian<10, 10> Hy = {});

  /// Compute the 10x10 adjoint map `ad(xi)` that acts on a tangent vector y `ad(xi, y) = ad(xi) * y`
  static Matrix10 adjointMap(const Vector10& xi);

  /// Derivative of Expmap(xi) w.r.t. xi
  static Matrix10 ExpmapDerivative(const Vector10& xi);

  /// Derivative of Logmap(g) w.r.t. g (represented as tangent vector at identity)
  static Matrix10 LogmapDerivative(const Gal3& g);


  // Chart at origin, depends on compile-time flag GTSAM_USE_MANIFOLD_MARKERS
  struct ChartAtOrigin {
    static Gal3 Retract(const Vector10& xi, ChartJacobian Hxi = {});
    static Vector10 Local(const Gal3& g, ChartJacobian Hg = {});
  };


  /// Hat operator: maps tangent vector xi to Lie algebra matrix (Manif convention).
  /// xi = [rho, nu, theta, t] -> [ skew(theta) nu rho ; 0 0 t ; 0 0 0 ]
  static Matrix5 Hat(const Vector10& xi);

  /// Vee operator: maps Lie algebra matrix to tangent vector xi (Manif convention).
  /// [ S N R ; 0 0 T ; 0 0 0 ] -> [ R ; N ; vee(S) ; T ] = [rho; nu; theta; t]
  static Vector10 Vee(const Matrix5& X);


  /// @}
  /// @name Helper Functions (moved inside class for context, could be private or in detail namespace)
  /// @{

  /// Calculate the SO(3) Left Jacobian J_l(theta)
  static Matrix3 SO3_LeftJacobian(const Vector3& theta);

  /// Calculate the inverse SO(3) Left Jacobian J_l(theta)^-1
  static Matrix3 SO3_LeftJacobianInverse(const Vector3& theta);

  /// Calculate the 3x3 E matrix used in SGal3 Expmap/Logmap
  static Matrix3 Calculate_E(const Vector3& theta);

  /// @}


private:
  /// @{
  /// serialization
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(R_);
    ar& BOOST_SERIALIZATION_NVP(r_);
    ar& BOOST_SERIALIZATION_NVP(v_);
    ar& BOOST_SERIALIZATION_NVP(t_);
  }
#endif
  /// @}
}; // class Gal3

// // Specialize Gal3 traits
template <>
struct traits<Gal3> : public internal::MatrixLieGroup<Gal3> {};

template <>
struct traits<const Gal3> : public internal::MatrixLieGroup<Gal3> {};

} // namespace gtsam
