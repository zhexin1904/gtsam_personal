/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LIEKF.h
 * @brief Base and classes for Left Invariant Extended Kalman Filters
 *
 * Templates are implemented for a Left Invariant Extended Kalman Filter
 * operating on Lie Groups.
 *
 *
 * @date April 24, 2025
 * @author Scott Baker
 * @author Matt Kielo
 * @author Frank Dellaert
 */

#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>

#include <Eigen/Dense>
#include <functional>
namespace gtsam {

/**
 * @brief Base class for Left Invariant Extended Kalman Filter (LIEKF)
 *
 * This class provides the prediction and update structure based on control
 * inputs and a measurement function.
 *
 * @tparam G  Lie group used for state representation (e.g., Pose2,
 * Pose3, NavState)
 * @tparam Measurement Type of measurement (e.g. Vector3 for a GPS measurement
 * for 3D position)
 */

template <typename G>
class LIEKF {
 public:
  static constexpr int n = traits<G>::dimension;  ///< Dimension of the state.

  using MatrixN =
      Eigen::Matrix<double, n, n>;  ///< Typedef for the identity matrix.

  /**
   * @brief Construct with a measurement function
   * @param X0 Initial State
   * @param P0 Initial Covariance
   * @param h Measurement function
   */
  LIEKF(const G& X0, const Matrix& P0) : X(X0), P(P0) {}

  /**
   * @brief Get current state estimate.
   * @return Const reference to the state estimate.
   */
  const G& state() const { return X; }

  /**
   * @brief Get current covariance estimate.
   * @return Const reference to the covariance estimate.
   */
  const Matrix& covariance() const { return P; }

  /**
   * @brief Prediction stage with a Lie group element U.
   * @param U Lie group control input
   * @param Q Process noise covariance matrix.
   */
  void predict(const G& U, const Matrix& Q) {
    typename G::Jacobian A;
    X = X.compose(U, A);
    P = A * P * A.transpose() + Q;
  }

  /**
   * @brief Prediction stage with a control vector u and a time interval dt.
   * @param u Control vector element
   * @param dt Time interval
   * @param Q Process noise covariance matrix.
   */
  void predict(const Vector& u, double dt, const Matrix& Q) {
    predict(G::Expmap(u * dt), Q);
  }

  /**
   * @brief Prediction stage with a dynamics function that calculates the
   * tangent vector xi in the tangent space.
   * @tparam _p The dimension of the control vector.
   * @tparam Dynamics : (G, VectorP, OptionalJacobian<n,n>) -> TangentVector
   * @param f Dynamics function that depends on state and control vector
   * @param u Control vector element
   * @param dt Time interval
   * @param Q Process noise covariance matrix.
   */
  template <size_t _p, typename Dynamics>
  void predict(Dynamics&& f, const Eigen::Matrix<double, _p, 1>& u, double dt,
               const Matrix& Q) {
    typename G::Jacobian F;
    const typename G::TangentVector xi = f(X, u, F);
    G U = G::Expmap(xi * dt);
    auto A = U.inverse().AdjointMap() * F;
    X = X.compose(U);
    P = A * P * A.transpose() + Q;
  }

  /**
   * @brief Update stage using a measurement and measurement covariance.
   * @tparam Measurement
   * @tparam Prediction : (G, OptionalJacobian<m,n>) -> Measurement
   * @param z Measurement
   * @param R Measurement noise covariance matrix.
   */
  template <typename Measurement, typename Prediction>
  void update(Prediction&& h, const Measurement& z, const Matrix& R) {
    Eigen::Matrix<double, traits<Measurement>::dimension, n> H;
    Vector y = h(X, H) - z;
    Matrix S = H * P * H.transpose() + R;
    Matrix K = P * H.transpose() * S.inverse();
    X = X.expmap(-K * y);
    P = (I_n - K * H) * P;  // move Identity to be a constant.
  }

 protected:
  G X;       ///< Current state estimate.
  Matrix P;  ///< Current covariance estimate.

 private:
  static const MatrixN
      I_n;  ///< A nxn identity matrix used in the update stage of the LIEKF.
};

/// Create the static identity matrix I_n of size nxn for use in update
template <typename G>
const typename LIEKF<G>::MatrixN LIEKF<G>::I_n = LIEKF<G>::MatrixN::Identity();

}  // namespace gtsam
