/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file    InvariantEKF.h
  * @brief   Left-Invariant Extended Kalman Filter implementation.
  *
  * This file defines the InvariantEKF class template, inheriting from LieGroupEKF,
  * which specifically implements the Left-Invariant EKF formulation. It restricts
  * prediction methods to only those based on group composition (state-independent
  * motion models), hiding the state-dependent prediction variants from LieGroupEKF.
  *
  * @date    April 24, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#pragma once

#include <gtsam/navigation/LieGroupEKF.h> // Include the base class
#include <gtsam/base/Lie.h> // For traits (needed for AdjointMap, Expmap)


namespace gtsam {

  /**
   * @class InvariantEKF
   * @brief Left-Invariant Extended Kalman Filter on a Lie group G.
   *
   * @tparam G Lie group type (must satisfy LieGroup concept).
   *
   * This filter inherits from LieGroupEKF but restricts the prediction interface
   * to only the left-invariant prediction methods:
   * 1. Prediction via group composition: `predict(const G& U, const Covariance& Q)`
   * 2. Prediction via tangent control vector: `predict(const TangentVector& u, double dt, const Covariance& Q)`
   *
   * The state-dependent prediction methods from LieGroupEKF are hidden.
   * The update step remains the same as in ManifoldEKF/LieGroupEKF.
   * Covariances (P, Q) are `Eigen::Matrix<double, Dim, Dim>`.
   */
  template <typename G>
  class InvariantEKF : public LieGroupEKF<G> {
  public:
    using Base = LieGroupEKF<G>; ///< Base class type
    using TangentVector = typename Base::TangentVector; ///< Tangent vector type
    /// Jacobian for group-specific operations like AdjointMap. Eigen::Matrix<double, Dim, Dim>.
    using Jacobian = typename Base::Jacobian;
    /// Covariance matrix type. Eigen::Matrix<double, Dim, Dim>.
    using Covariance = typename Base::Covariance;


    /**
     * Constructor: forwards to LieGroupEKF constructor.
     * @param X0 Initial state on Lie group G.
     * @param P0_input Initial covariance in the tangent space at X0 (dynamic gtsam::Matrix).
     */
    InvariantEKF(const G& X0, const Matrix& P0_input) : Base(X0, P0_input) {}

    // We hide state-dependent predict methods from LieGroupEKF by only providing the
    // invariant predict methods below.

    /**
     * Predict step via group composition (Left-Invariant):
     *   X_{k+1} = X_k * U
     *   P_{k+1} = Ad_{U^{-1}} P_k Ad_{U^{-1}}^T + Q
     * where Ad_{U^{-1}} is the Adjoint map of U^{-1}.
     *
     * @param U Lie group element representing the motion increment.
     * @param Q Process noise covariance (Eigen::Matrix<double, Dim, Dim>).
     */
    void predict(const G& U, const Covariance& Q) {
      this->X_ = this->X_.compose(U);
      // TODO(dellaert): traits<G>::AdjointMap should exist
      const Jacobian A = traits<G>::Inverse(U).AdjointMap();
      // P_ is Covariance. A is Jacobian. Q is Covariance.
      // All are Eigen::Matrix<double,Dim,Dim>.
      this->P_ = A * this->P_ * A.transpose() + Q;
    }

    /**
     * Predict step via tangent control vector:
     *   U = Expmap(u * dt)
     * Then calls predict(U, Q).
     *
     * @param u Tangent space control vector.
     * @param dt Time interval.
     * @param Q Process noise covariance matrix (Eigen::Matrix<double, Dim, Dim>).
     */
    void predict(const TangentVector& u, double dt, const Covariance& Q) {
      const G U = traits<G>::Expmap(u * dt);
      predict(U, Q); // Call the group composition predict
    }

  }; // InvariantEKF

} // namespace gtsam