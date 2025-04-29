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
  * This file defines the InvariantEKF class template, inheriting from GroupEKF,
  * which specifically implements the Left-Invariant EKF formulation. It restricts
  * prediction methods to only those based on group composition (state-independent
  * motion models), hiding the state-dependent prediction variants from GroupEKF.
  *
  * @date    April 24, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#pragma once

#include <gtsam/navigation/GroupEKF.h> // Include the base class

namespace gtsam {

  /**
   * @class InvariantEKF
   * @brief Left-Invariant Extended Kalman Filter on a Lie group G.
   *
   * @tparam G Lie group type (must satisfy LieGroup concept).
   *
   * This filter inherits from GroupEKF but restricts the prediction interface
   * to only the left-invariant prediction methods:
   * 1. Prediction via group composition: `predict(const G& U, const Matrix& Q)`
   * 2. Prediction via tangent control vector: `predict(const TangentVector& u, double dt, const Matrix& Q)`
   *
   * The state-dependent prediction methods from GroupEKF are hidden.
   * The update step remains the same as in ManifoldEKF/GroupEKF.
   */
  template <typename G>
  class InvariantEKF : public GroupEKF<G> {
  public:
    using Base = GroupEKF<G>; ///< Base class type
    using TangentVector = typename Base::TangentVector; ///< Tangent vector type
    using MatrixN = typename Base::MatrixN; ///< Square matrix type for covariance etc.

    /// Constructor: forwards to GroupEKF constructor
    InvariantEKF(const G& X0, const MatrixN& P0) : Base(X0, P0) {}

    // --- Expose only the Invariant Prediction Methods ---

    /**
     * Predict step via group composition (Left-Invariant):
     *   X_{k+1} = X_k * U
     *   P_{k+1} = Ad_{U^{-1}} P_k Ad_{U^{-1}}^T + Q
     * Calls the base class implementation.
     *
     * @param U Lie group element representing the motion increment.
     * @param Q Process noise covariance in the tangent space (size nxn).
     */
    void predict(const G& U, const Matrix& Q) {
      Base::predict(U, Q);
    }

    /**
     * Predict step via tangent control vector:
     *   U = Expmap(u * dt)
     * Then calls predict(U, Q). Calls the base class implementation.
     *
     * @param u Tangent space control vector.
     * @param dt Time interval.
     * @param Q Process noise covariance matrix (size nxn).
     */
    void predict(const TangentVector& u, double dt, const Matrix& Q) {
      Base::predict(u, dt, Q);
    }

  }; // InvariantEKF

} // namespace gtsam