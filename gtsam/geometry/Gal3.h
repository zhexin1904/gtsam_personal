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
 * @authors Matt Kielo, Scott Baker, Frank Dellaert
 * @date    April 30, 2025
 */

#pragma once

#include <gtsam/geometry/Pose3.h> // Includes Rot3, Point3
#include <gtsam/geometry/Event.h>
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
// Define Matrix5 for Lie Algebra matrix representation
using Matrix5 = Eigen::Matrix<double, 5, 5>;
// Define Matrix10 for Jacobians
using Matrix10 = Eigen::Matrix<double, 10, 10>;

/**
 * Represents an element of the 3D Galilean group SGal(3).
 * Internal state: rotation, translation, velocity, time.
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

  /// Construct from attitude, position, velocity, time
  Gal3(const Rot3& R, const Point3& r, const Velocity3& v, double t) :
    R_(R), r_(r), v_(v), t_(t) {}

  /// Construct from a 5x5 matrix representation
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

  // Convenience accessors matching NavState names
  const Rot3& attitude(OptionalJacobian<3, 10> H = {}) const { return rotation(H); }
  const Point3& position(OptionalJacobian<3, 10> H = {}) const { return translation(H); }

  /// @}
  /// @name Derived quantities
  /// @{

  /// Return rotation matrix (Matrix3)
  Matrix3 R() const { return R_.matrix(); }

  /// Return position as Vector3
  Vector3 r() const { return Vector3(r_); } // Conversion from Point3

  /// Return velocity as Vector3
  const Vector3& v() const { return v_; }

  /// Return time scalar
  const double& t() const { return t_; }

  /// Return 5x5 homogeneous matrix representation
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

  /// Group composition operator
  Gal3 operator*(const Gal3& other) const;

  /// @}
  /// @name Group Action
  /// @{

  /**
   * Apply Galilean transform to a spacetime Event
   * @param e Input event (location, time)
   * @param Hself Optional Jacobian wrt this Gal3 element's tangent space
   * @param He Optional Jacobian wrt the input event's tangent space
   * @return Transformed event
   */
  Event act(const Event& e, OptionalJacobian<4, 10> Hself = {},
            OptionalJacobian<4, 4> He = {}) const;

  /// @}
  /// @name Lie Group Static Functions
  /// @{

  /// Exponential map at identity: tangent vector xi -> manifold element g
  static Gal3 Expmap(const Vector10& xi, OptionalJacobian<10, 10> Hxi = {});

  /// Logarithmic map at identity: manifold element g -> tangent vector xi
  static Vector10 Logmap(const Gal3& g, OptionalJacobian<10, 10> Hg = {});

  /// Calculate Adjoint map Ad_g
  Matrix10 AdjointMap() const;

  /// Apply this element's AdjointMap Ad_g to a tangent vector xi_base at identity
  Vector10 Adjoint(const Vector10& xi_base, OptionalJacobian<10, 10> H_g = {},
                  OptionalJacobian<10, 10> H_xi = {}) const;

  /// The adjoint action `ad(xi, y)` = `adjointMap(xi) * y`
  static Vector10 adjoint(const Vector10& xi, const Vector10& y,
                         OptionalJacobian<10, 10> Hxi = {},
                         OptionalJacobian<10, 10> Hy = {});

  /// Compute the adjoint map `ad(xi)` associated with tangent vector xi
  static Matrix10 adjointMap(const Vector10& xi);

  /// Derivative of Expmap(xi) w.r.t. xi evaluated at xi
  static Matrix10 ExpmapDerivative(const Vector10& xi);

  /// Derivative of Logmap(g) w.r.t. g
  static Matrix10 LogmapDerivative(const Gal3& g);

  /// Chart at origin, uses Expmap/Logmap for Retract/Local
  struct ChartAtOrigin {
    static Gal3 Retract(const Vector10& xi, ChartJacobian Hxi = {});
    static Vector10 Local(const Gal3& g, ChartJacobian Hg = {});
  };

  /// Hat operator: maps tangent vector xi to Lie algebra matrix
  static Matrix5 Hat(const Vector10& xi);

  /// Vee operator: maps Lie algebra matrix to tangent vector xi
  static Vector10 Vee(const Matrix5& X);

  /// @}

 private:
  /// @name Serialization
  /// @{
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(R_);
    ar & BOOST_SERIALIZATION_NVP(r_);
    ar & BOOST_SERIALIZATION_NVP(v_);
    ar & BOOST_SERIALIZATION_NVP(t_);
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
