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
 * Based on manif convention: [R, r, v, t], tangent [rho, nu, theta, t_tan]
 * @authors Matt Kielo, Scott Baker, Frank Dellaert
 * @date    April 30, 2025
 */

#pragma once

#include <gtsam/geometry/Pose3.h> // Includes Rot3, Point3
#include <gtsam/base/Lie.h>       // For LieGroup base class and traits
#include <gtsam/base/Manifold.h>  // For Manifold traits

#include <cmath> // For std::sqrt, std::cos, std::sin
#include <functional> // For std::function used in numerical derivatives

namespace gtsam {

// Forward declaration
class Gal3;

// Use Vector3 for velocity for consistency with NavState
using Velocity3 = Vector3;
// Define Vector10 for tangent space
using Vector10 = Eigen::Matrix<double, 10, 1>;
// Define Matrix5 for Lie Algebra matrix representation (Hat map output)
using Matrix5 = Eigen::Matrix<double, 5, 5>;
// Define Matrix10 for Jacobians (AdjointMap, Expmap/Logmap derivatives)
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

 public:

  /// The dimension of the tangent space
  inline static constexpr size_t dimension = 10;

  /// @name Constructors
  /// @{

  /// Default constructor: Identity element
  Gal3() : R_(Rot3::Identity()), r_(Point3::Zero()), v_(Velocity3::Zero()), t_(0.0) {}

  /// Construct from attitude, position, velocity, time (manif order)
  Gal3(const Rot3& R, const Point3& r, const Velocity3& v, double t) :
    R_(R), r_(r), v_(v), t_(t) {}

  /// Construct from a 5x5 matrix representation (manif convention)
  explicit Gal3(const Matrix5& M);

  /// Named constructor from components with derivatives
  static Gal3 Create(const Rot3& R, const Point3& r, const Velocity3& v, double t,
                    OptionalJacobian<10, 3> H1 = {},
                    OptionalJacobian<10, 3> H2 = {},
                    OptionalJacobian<10, 3> H3 = {},
                    OptionalJacobian<10, 1> H4 = {});

  /// Named constructor from Pose3, velocity, and time with derivatives
  static Gal3 FromPoseVelocityTime(const Pose3& pose, const Velocity3& v, double t,
                                OptionalJacobian<10, 6> H1 = {},
                                OptionalJacobian<10, 3> H2 = {},
                                OptionalJacobian<10, 1> H3 = {});

  /// @}
  /// @name Component Access
  /// @{

  /// Access rotation component (Rot3)
  const Rot3& rotation(OptionalJacobian<3, 10> H = {}) const;

  /// Access translation component (Point3)
  const Point3& translation(OptionalJacobian<3, 10> H = {}) const;

  /// Access velocity component (Vector3)
  const Velocity3& velocity(OptionalJacobian<3, 10> H = {}) const;

  /// Access time component (double)
  const double& time(OptionalJacobian<1, 10> H = {}) const;

  // Convenience accessors matching NavState names where they overlap
  const Rot3& attitude(OptionalJacobian<3, 10> H = {}) const { return rotation(H); }
  const Point3& position(OptionalJacobian<3, 10> H = {}) const { return translation(H); }


  /// @}
  /// @name Derived quantities
  /// @{

  /// Return rotation matrix (Matrix3).
  Matrix3 R() const { return R_.matrix(); }

  /// Return position as Vector3.
  Vector3 r() const { return Vector3(r_); } // Conversion from Point3

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

  /// Print with optional string prefix
  void print(const std::string& s = "") const;

  /// Check equality within tolerance
  bool equals(const Gal3& other, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// Return the identity element
  static Gal3 Identity() { return Gal3(); }

  /// Return the inverse of this element
  Gal3 inverse() const;

  // Bring LieGroup::inverse() into scope (version with derivative)
  using LieGroup<Gal3, 10>::inverse;

  /// Compose with another element: `this * other`
  Gal3 compose(const Gal3& other, OptionalJacobian<10, 10> H1 = {},
               OptionalJacobian<10, 10> H2 = {}) const;

  /// Group composition operator: `this * other`
  Gal3 operator*(const Gal3& other) const { return compose(other); }

  /// Calculate the relative transformation: `this->inverse() * other`
  Gal3 between(const Gal3& other, OptionalJacobian<10, 10> H1 = {},
               OptionalJacobian<10, 10> H2 = {}) const;

  /// @}
  /// @name Manifold Operations
  /// @{

  /// Retract from tangent space xi to manifold element (this + xi)
  Gal3 retract(const Vector10& xi, OptionalJacobian<10, 10> H1 = {},
               OptionalJacobian<10, 10> H2 = {}) const;

  /// Compute local coordinates (tangent vector) from this to other (logmap(this.inverse * other))
  Vector10 localCoordinates(const Gal3& other, OptionalJacobian<10, 10> H1 = {},
                            OptionalJacobian<10, 10> H2 = {}) const;


  /**
   * Interpolate between two Gal3 elements.
   * @param other The other Gal3 element.
   * @param alpha Interpolation fraction (0=this, 1=other).
   * @return Interpolated Gal3 element: this->compose(Expmap(alpha * Logmap(this->between(other))))
   */
   Gal3 interpolate(const Gal3& other, double alpha,
                    OptionalJacobian<10, 10> H1 = {},
                    OptionalJacobian<10, 10> H2 = {},
                    OptionalJacobian<10, 1> Ha = {}) const; // Added Jacobian args to match NavState/typical patterns

  /// @}
  /// @name Group Action
  /// @{

  /**
   * Action of the group element on a 3D point.
   * Following GTSAM convention for Pose3::transformFrom, this applies
   * the instantaneous rotation and translation: p' = R*p + r.
   * If time evolution is needed, use compose or specific dynamics model.
   */
  Point3 act(const Point3& p,
             OptionalJacobian<3, 10> Hself = {},
             OptionalJacobian<3, 3> Hp = {}) const;

  // Alias for consistency if used elsewhere
  Point3 transformFrom(const Point3& p,
                       OptionalJacobian<3, 10> Hself = {},
                       OptionalJacobian<3, 3> Hp = {}) const {
      return act(p, Hself, Hp);
  }

  /// @}
  /// @name Lie Group Static Functions
  /// @{

  /// The type of the Lie algebra (matrix representation)
  using LieAlgebra = Matrix5;

  // Helper functions for accessing tangent vector components by name
  // Input xi is [rho(3), nu(3), theta(3), t_tan(1)]
  // *** CORRECTED to use .block<rows, cols>(startRow, startCol) syntax ***
  static Eigen::Block<Vector10, 3, 1> rho(Vector10& v) { return v.block<3, 1>(0, 0); }
  static Eigen::Block<Vector10, 3, 1> nu(Vector10& v) { return v.block<3, 1>(3, 0); }
  static Eigen::Block<Vector10, 3, 1> theta(Vector10& v) { return v.block<3, 1>(6, 0); }
  static Eigen::Block<Vector10, 1, 1> t_tan(Vector10& v) { return v.block<1, 1>(9, 0); }
  // Const versions
  static Eigen::Block<const Vector10, 3, 1> rho(const Vector10& v) { return v.block<3, 1>(0, 0); }
  static Eigen::Block<const Vector10, 3, 1> nu(const Vector10& v) { return v.block<3, 1>(3, 0); }
  static Eigen::Block<const Vector10, 3, 1> theta(const Vector10& v) { return v.block<3, 1>(6, 0); }
  static Eigen::Block<const Vector10, 1, 1> t_tan(const Vector10& v) { return v.block<1, 1>(9, 0); }

  /// Exponential map at identity: tangent vector xi -> manifold element g
  /// xi = [rho, nu, theta, t_tan] (10x1)
  static Gal3 Expmap(const Vector10& xi, OptionalJacobian<10, 10> Hxi = {});

  /// Logarithmic map at identity: manifold element g -> tangent vector xi
  /// Returns xi = [rho, nu, theta, t_tan] (10x1)
  static Vector10 Logmap(const Gal3& g, OptionalJacobian<10, 10> Hg = {});

  /// Calculate Adjoint map Ad_g, transforming a tangent vector in the identity frame
  /// to the frame of this element g. Ad_g = [Ad_R, 0; Ad_r*Ad_R, Ad_R] structure adapted for Gal3.
  Matrix10 AdjointMap() const;

  /// Apply this element's AdjointMap Ad_g to a tangent vector xi_base at identity.
  /// Result is xi_this = Ad_g * xi_base.
  /// H_g = d(Ad_g * xi_base)/dg, H_xi = d(Ad_g * xi_base)/dxi_base = Ad_g
  Vector10 Adjoint(const Vector10& xi_base, OptionalJacobian<10, 10> H_g = {}, OptionalJacobian<10, 10> H_xi = {}) const;

  /// The adjoint action `ad(xi, y)` = `adjointMap(xi) * y`.
  /// This is the Lie bracket [xi, y] = ad_xi(y).
  static Vector10 adjoint(const Vector10& xi, const Vector10& y, OptionalJacobian<10, 10> Hxi = {}, OptionalJacobian<10, 10> Hy = {});

  /// Compute the 10x10 adjoint map `ad(xi)` associated with tangent vector xi.
  /// Acts on a tangent vector y: `ad(xi, y) = ad(xi) * y`.
  static Matrix10 adjointMap(const Vector10& xi);

  /// Derivative of Expmap(xi) w.r.t. xi evaluated at xi.
  /// Currently implemented numerically.
  static Matrix10 ExpmapDerivative(const Vector10& xi);

  /// Derivative of Logmap(g) w.r.t. g (represented as tangent vector at identity).
  /// Currently implemented numerically.
  static Matrix10 LogmapDerivative(const Gal3& g);

  /// Chart at origin, uses Expmap/Logmap for Retract/Local
  struct ChartAtOrigin {
    static Gal3 Retract(const Vector10& xi, ChartJacobian Hxi = {});
    static Vector10 Local(const Gal3& g, ChartJacobian Hg = {});
  };

  /// Hat operator: maps tangent vector xi to Lie algebra matrix (Manif convention).
  /// xi = [rho, nu, theta, t_tan] -> [ skew(theta) nu rho ; 0 0 t_tan ; 0 0 0 ]
  static Matrix5 Hat(const Vector10& xi);

  /// Vee operator: maps Lie algebra matrix to tangent vector xi (Manif convention).
  /// [ S N R ; 0 0 T ; 0 0 0 ] -> [ R ; N ; vee(S) ; T ] = [rho; nu; theta; t_tan]
  static Vector10 Vee(const Matrix5& X);

  /// @}

 private:
  /// @name Serialization
  /// @{
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // Serialize member variables
    ar& BOOST_SERIALIZATION_NVP(R_);
    ar& BOOST_SERIALIZATION_NVP(r_);
    ar& BOOST_SERIALIZATION_NVP(v_);
    ar& BOOST_SERIALIZATION_NVP(t_);
  }
#endif
  /// @}

}; // class Gal3

/// Traits specialization for Gal3
template <>
struct traits<Gal3> : public internal::LieGroup<Gal3> {};

template <>
struct traits<const Gal3> : public internal::LieGroup<Gal3> {};

} // namespace gtsam
