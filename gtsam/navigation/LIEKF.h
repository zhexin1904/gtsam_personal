/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LIEKF.h
 * @brief   Left-Invariant Extended Kalman Filter (LIEKF) implementation
 *
 * This file defines the LIEKF class template for performing prediction and
 * update steps of an Extended Kalman Filter on states residing in a Lie group.
 * The class supports state evolution via group composition and dynamics
 * functions, along with measurement updates using tangent-space corrections.
 *
 * @date    April 24, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>

#include <Eigen/Dense>


namespace gtsam {

/**
 * @class LIEKF
 * @brief Left-Invariant Extended Kalman Filter (LIEKF) on a Lie group G
 *
 * @tparam G  Lie group type providing:
 *           - static int dimension = tangent dimension
 *           - using TangentVector = Eigen::Vector...
 *           - using Jacobian    = Eigen::Matrix...
 *           - methods: Expmap(), expmap(), compose(), inverse().AdjointMap()
 *
 * This filter maintains a state X in the group G and covariance P in the
 * tangent space. Prediction steps are performed via group composition or a
 * user-supplied dynamics function. Updates apply a measurement function h
 * returning both predicted measurement and its Jacobian H, and correct state
 * using the left-invariant error in the tangent space.
 */
template <typename G>
class LIEKF {
 public:
  /// Tangent-space dimension
  static constexpr int n = traits<G>::dimension;

  /// Square matrix of size n for covariance and Jacobians
  using MatrixN = Eigen::Matrix<double, n, n>;

  /// Constructor: initialize with state and covariance
  LIEKF(const G& X0, const Matrix& P0) : X_(X0), P_(P0) {}

  /// @return current state estimate
  const G& state() const { return X_; }

  /// @return current covariance estimate
  const Matrix& covariance() const { return P_; }

  /**
   * Predict step via group composition:
   *   X_{k+1} = X_k * U
   *   P_{k+1} = A P_k A^T + Q
   * where A = Ad_{U^{-1}}. i.e., d(X.compose(U))/dX evaluated at X_k.
   *
   * @param U  Lie group increment (e.g., Expmap of control * dt)
   * @param Q  process noise covariance in tangent space
   */
  void predict(const G& U, const Matrix& Q) {
    typename G::Jacobian A;
    X_ = X_.compose(U, A);
    P_ = A * P_ * A.transpose() + Q;
  }

  /**
   * Predict step via tangent control vector:
   *   U = Expmap(u * dt)
   * @param u tangent control vector
   * @param dt Time interval
   * @param Q Process noise covariance matrix.
   *
   * @note Use this if your dynamics does not depend on the state, e.g.
   * predict(f(u), dt, q) where u is a control input
   */
  void predict(const Vector& u, double dt, const Matrix& Q) {
    predict(G::Expmap(u * dt), Q);
  }

  /**
   * Predict step with state-dependent dynamics:
   *   xi = f(X, F)
   *   U  = Expmap(xi * dt)
   *   A  = Ad_{U^{-1}} * F
   *
   * @tparam Dynamics  signature: G f(const G&,  OptionalJacobian<n,n>&)
   *
   * @param f   dynamics functor depending on state and control
   * @param dt  time step
   * @param Q   process noise covariance
   */
  template <typename Dynamics>
  void predict(Dynamics&& f, double dt, const Matrix& Q) {
    typename G::Jacobian F;
    auto xi = f(X_, F);
    G U = G::Expmap(xi * dt);
    auto A = U.inverse().AdjointMap() * F;
    X_ = X_.compose(U);
    P_ = A * P_ * A.transpose() + Q;
  }

  /**
   * Predict step with state and control input dynamics:
   *   xi = f(X, u, F)
   *   U  = Expmap(xi * dt)
   *   A  = Ad_{U^{-1}} * F
   *
   * @tparam Control   control input type
   * @tparam Dynamics  signature: G f(const G&, const Control&,
   *                                  OptionalJacobian<n,n>&)
   *
   * @param f   dynamics functor depending on state and control
   * @param u   control input
   * @param dt  time step
   * @param Q   process noise covariance
   */
  template <typename Control, typename Dynamics>
  void predict(Dynamics&& f, const Control& u, double dt, const Matrix& Q) {
    typename G::Jacobian F;
    auto xi = f(X_, u, F);
    G U = G::Expmap(xi * dt);
    auto A = U.inverse().AdjointMap() * F;
    X_ = X_.compose(U);
    P_ = A * P_ * A.transpose() + Q;
  }

  /**
   * Measurement update:
   *   z_pred, H = h(X)
   *   K = P H^T (H P H^T + R)^{-1}
   *   X <- Expmap(-K (z_pred - z)) * X
   *   P <- (I - K H) P
   *
   * @tparam Measurement  measurement type (e.g., Vector)
   * @tparam Prediction   functor signature: Measurement h(const G&,
   * OptionalJacobian<m,n>&)
   *
   * @param h  measurement model returning predicted z and Jacobian H
   * @param z  observed measurement
   * @param R  measurement noise covariance
   */
  template <typename Measurement, typename Prediction>
  void update(Prediction&& h, const Measurement& z, const Matrix& R) {
    Eigen::Matrix<double, traits<Measurement>::dimension, n> H;
    auto z_pred = h(X_, H);
    auto y = z_pred - z;
    Matrix S = H * P_ * H.transpose() + R;
    Matrix K = P_ * H.transpose() * S.inverse();
    X_ = X_.expmap(-K * y);
    P_ = (I_n - K * H) * P_;
  }

 protected:
  G X_;       ///< group state estimate
  Matrix P_;  ///< covariance in tangent space

 private:
  /// Identity matrix of size n
  static const MatrixN I_n;
};

// Define static identity I_n
template <typename G>
const typename LIEKF<G>::MatrixN LIEKF<G>::I_n = LIEKF<G>::MatrixN::Identity();

}  // namespace gtsam
