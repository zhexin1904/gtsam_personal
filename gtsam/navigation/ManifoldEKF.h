/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file    ManifoldEKF.h
  * @brief   Extended Kalman Filter base class on a generic manifold M
  *
  * This file defines the ManifoldEKF class template for performing prediction
  * and update steps of an Extended Kalman Filter on states residing in a
  * differentiable manifold. It relies on the manifold's retract and
  * localCoordinates operations.
  *
  * Works with manifolds M that may have fixed or dynamic tangent space dimensions.
  * Covariances and Jacobians are handled as `Matrix` (dynamic-size Eigen matrices).
  *
  * @date    April 24, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h> // Include for traits and IsManifold

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <type_traits>

namespace gtsam {

  /**
     * @class ManifoldEKF
     * @brief Extended Kalman Filter on a generic manifold M
     *
     * @tparam M  Manifold type providing:
     *           - `traits<M>` specialization must exist, defining
     *             `dimension` (compile-time or Eigen::Dynamic),
     *             `TangentVector`, `Retract`, and `LocalCoordinates`.
     *             If `dimension` is Eigen::Dynamic, `GetDimension(const M&)`
     *             must be provided by traits.
     *
     * This filter maintains a state X in the manifold M and covariance P in the
     * tangent space at X. The covariance P is always stored as a gtsam::Matrix.
     * Prediction requires providing the predicted next state and the state transition Jacobian F.
     * Updates apply a measurement function h and correct the state using the tangent space error.
     */
  template <typename M>
  class ManifoldEKF {
  public:
    /// Tangent vector type for the manifold M, as defined by its traits.
    using TangentVector = typename traits<M>::TangentVector;

    /**
     * Constructor: initialize with state and covariance.
     * @param X0 Initial state on manifold M.
     * @param P0 Initial covariance in the tangent space at X0. Must be a square
     *           Matrix whose dimensions match the tangent space dimension of X0.
     */
    ManifoldEKF(const M& X0, const Matrix& P0) : X_(X0), P_(P0) {
      static_assert(IsManifold<M>::value,
        "Template parameter M must be a GTSAM Manifold.");

      // Determine tangent space dimension n_ at runtime.
      if constexpr (traits<M>::dimension == Eigen::Dynamic) {
        // If M::dimension is dynamic, traits<M>::GetDimension(M) must exist.
        n_ = traits<M>::GetDimension(X0);
      }
      else {
        n_ = traits<M>::dimension;
      }

      // Validate dimensions of initial covariance P0.
      if (P0.rows() != n_ || P0.cols() != n_) {
        throw std::invalid_argument(
          "ManifoldEKF: Initial covariance P0 dimensions (" +
          std::to_string(P0.rows()) + "x" + std::to_string(P0.cols()) +
          ") do not match state's tangent space dimension (" +
          std::to_string(n_) + ").");
      }
    }

    virtual ~ManifoldEKF() = default;

    /// @return current state estimate on manifold M.
    const M& state() const { return X_; }

    /// @return current covariance estimate in the tangent space (always a Matrix).
    const Matrix& covariance() const { return P_; }

    /**
     * Basic predict step: Updates state and covariance given the predicted
     * next state and the state transition Jacobian F.
     *   X_{k+1} = X_next
     *   P_{k+1} = F P_k F^T + Q
     * where F = d(local(X_{k+1})) / d(local(X_k)) is the Jacobian of the
     * state transition in local coordinates around X_k.
     *
     * @param X_next The predicted state at time k+1 on manifold M.
     * @param F The state transition Jacobian (size nxn).
     * @param Q Process noise covariance matrix in the tangent space (size nxn).
     */
    void predict(const M& X_next, const Matrix& F, const Matrix& Q) {
      if (F.rows() != n_ || F.cols() != n_) {
        throw std::invalid_argument(
          "ManifoldEKF::predict: Jacobian F dimensions (" +
          std::to_string(F.rows()) + "x" + std::to_string(F.cols()) +
          ") must be " + std::to_string(n_) + "x" + std::to_string(n_) + ".");
      }
      if (Q.rows() != n_ || Q.cols() != n_) {
        throw std::invalid_argument(
          "ManifoldEKF::predict: Noise Q dimensions (" +
          std::to_string(Q.rows()) + "x" + std::to_string(Q.cols()) +
          ") must be " + std::to_string(n_) + "x" + std::to_string(n_) + ".");
      }
      X_ = X_next;
      P_ = F * P_ * F.transpose() + Q;
    }

    /**
     * Measurement update: Corrects the state and covariance using a pre-calculated
     * predicted measurement and its Jacobian.
     *
     * @tparam Measurement Type of the measurement vector (e.g., VectorN<m>, Vector).
     * @param prediction Predicted measurement.
     * @param H Jacobian of the measurement function h w.r.t. local(X), H = dh/dlocal(X).
     *          Its dimensions must be m x n.
     * @param z Observed measurement.
     * @param R Measurement noise covariance (size m x m).
     */
    template <typename Measurement>
    void update(const Measurement& prediction,
      const Matrix& H,
      const Measurement& z,
      const Matrix& R) {

      static_assert(IsManifold<Measurement>::value,
        "Template parameter Measurement must be a GTSAM Manifold for LocalCoordinates.");

      int m; // Measurement dimension
      if constexpr (traits<Measurement>::dimension == Eigen::Dynamic) {
        m = traits<Measurement>::GetDimension(z);
        if (traits<Measurement>::GetDimension(prediction) != m) {
          throw std::invalid_argument(
            "ManifoldEKF::update: Dynamic measurement 'prediction' and 'z' have different dimensions.");
        }
      }
      else {
        m = traits<Measurement>::dimension;
      }

      if (H.rows() != m || H.cols() != n_) {
        throw std::invalid_argument(
          "ManifoldEKF::update: Jacobian H dimensions (" +
          std::to_string(H.rows()) + "x" + std::to_string(H.cols()) +
          ") must be " + std::to_string(m) + "x" + std::to_string(n_) + ".");
      }
      if (R.rows() != m || R.cols() != m) {
        throw std::invalid_argument(
          "ManifoldEKF::update: Noise R dimensions (" +
          std::to_string(R.rows()) + "x" + std::to_string(R.cols()) +
          ") must be " + std::to_string(m) + "x" + std::to_string(m) + ".");
      }

      // Innovation: y = z - h(x_pred). In tangent space: local(h(x_pred), z)
      // This is `log(prediction.inverse() * z)` if Measurement is a Lie group.
      typename traits<Measurement>::TangentVector innovation =
        traits<Measurement>::Local(prediction, z);

      // Innovation covariance: S = H P H^T + R
      const Matrix S = H * P_ * H.transpose() + R; // S is m x m

      // Kalman Gain: K = P H^T S^-1
      const Matrix K = P_ * H.transpose() * S.inverse(); // K is n_ x m

      // Correction vector in tangent space of M: delta_xi = K * innovation
      const TangentVector delta_xi = K * innovation; // delta_xi is n_ x 1

      // Update state using retract: X_new = retract(X_old, delta_xi)
      X_ = traits<M>::Retract(X_, delta_xi);

      // Update covariance using Joseph form for numerical stability
      const auto I_n = Matrix::Identity(n_, n_);
      const Matrix I_KH = I_n - K * H; // I_KH is n x n
      P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    }

    /**
     * Measurement update: Corrects the state and covariance using a measurement
     * model function.
     *
     * @tparam Measurement Type of the measurement vector (e.g., VectorN<m>, Vector).
     * @tparam MeasurementFunction Functor/lambda with signature compatible with:
     *        `Measurement h(const M& x, Jac& H_jacobian)`
     *        where `Jac` can be `Matrix&` or `OptionalJacobian<m, n_>&`.
     *        The Jacobian H should be d(h)/d(local(X)).
     * @param h Measurement model function.
     * @param z Observed measurement.
     * @param R Measurement noise covariance (must be an m x m Matrix).
     */
    template <typename Measurement, typename MeasurementFunction>
    void update(MeasurementFunction&& h, const Measurement& z, const Matrix& R) {
      static_assert(IsManifold<Measurement>::value,
        "Template parameter Measurement must be a GTSAM Manifold.");

      int m; // Measurement dimension
      if constexpr (traits<Measurement>::dimension == Eigen::Dynamic) {
        m = traits<Measurement>::GetDimension(z);
      }
      else {
        m = traits<Measurement>::dimension;
      }

      // Predict measurement and get Jacobian H = dh/dlocal(X)
      Matrix H(m, n_);
      Measurement prediction = h(X_, H);

      // Call the other update function
      update(prediction, H, z, R);
    }

  protected:
    M X_;      ///< Manifold state estimate.
    Matrix P_; ///< Covariance in tangent space at X_ (always a dynamic Matrix).
    int n_;    ///< Tangent space dimension of M, determined at construction.
  };

}  // namespace gtsam