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
 * @tparam LieGroup  Lie group used for state representation (e.g., Pose2,
 * Pose3, NavState)
 * @tparam Measurement Type of measurement (e.g. Vector3 for a GPS measurement
 * for 3D position)
 */

template <typename LieGroup, typename Measurement>
class LIEKF {
 public:
  static constexpr int n =
      traits<LieGroup>::dimension;  ///< Dimension of the state.
  static constexpr int m =
      traits<Measurement>::dimension;  ///< Dimension of the measurement.

  using MeasurementFunction = std::function<Measurement(
      const LieGroup&,
      OptionalJacobian<m, n>)>;  ///< Typedef for the measurement function.
  using MatrixN =
      Eigen::Matrix<double, n, n>;  ///< Typedef for the identity matrix.

  /**
   * @brief Construct with a measurement function
   * @param X0 Initial State
   * @param P0 Initial Covariance
   * @param h Measurement function
   */
  LIEKF(const LieGroup& X0, const Matrix& P0, MeasurementFunction& h)  // X_ P_
      : X(X0), P(P0), h_(h) {}

  /**
   * @brief Get current state estimate.
   * @return Const reference to the state estiamte.
   */
  const LieGroup& state() const { return X; }

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
  void predict(const LieGroup& U, const Matrix& Q) {
    LieGroup::Jacobian A;
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
    predict(LieGroup::Expmap(u * dt), Q);
  }

  /**
   * @brief Update stage using a measurement and measurement covariance.
   * @param z Measurement
   * @param R Measurement noise covariance matrix.
   */
  void update(const Measurement& z, const Matrix& R) {
    Matrix H(m, n);
    Vector y = h_(X, H) - z;
    Matrix S = H * P * H.transpose() + R;
    Matrix K = P * H.transpose() * S.inverse();
    X = X.expmap(-K * y);
    P = (I_n - K * H) * P;  // move Identity to be a constant.
  }

 protected:
  LieGroup X;  ///< Current state estimate.
  Matrix P;    ///< Current covariance estimate.

 private:
  MeasurementFunction h_;  ///< Measurement function
  static const MatrixN
      I_n;  ///< A nxn identity matrix used in the update stage of the LIEKF.
};

/**
 * @brief Create the static identity matrix I_n of size nxn for use in the
 * update stage.
 * @tparam LieGroup  Lie group used for state representation (e.g., Pose2,
 * Pose3, NavState)
 * @tparam Measurement Type of measurement (e.g. Vector3 for a GPS measurement
 * for 3D position)
 */
template <typename LieGroup, typename Measurement>
const typename LIEKF<LieGroup, Measurement>::MatrixN
    LIEKF<LieGroup, Measurement>::I_n =
        typename LIEKF<LieGroup, Measurement>::MatrixN::Identity();

/**
 * @brief General Left Invariant Extended Kalman Filter with dynamics function.
 *
 * This class extends the LIEKF class to include a dynamics function f. The
 * dynamics maps a state and control vector to a tangent vector xi.
 *
 * @tparam LieGroup The Lie group type for the state.
 * @tparam Measurement The type of the measurement.
 * @tparam _p The dimension of the control vector.
 */
template <typename LieGroup, typename Measurement, size_t _p>
class GeneralLIEKF : public LIEKF<LieGroup, Measurement> {
 public:
  using Control =
      Eigen::Matrix<double, _p, 1>;  ///< Typedef for the control vector.
  using TangentVector =
      typename traits<LieGroup>::TangentVector;  ///< Typedef for the tangent
                                                 ///< vector.
  using Dynamics = std::function<TangentVector(
      const LieGroup&, const Control&,  ///< Typedef for the dynamics function.
      OptionalJacobian<n, n>)>;

  /**
   * @brief Construct with general dynamics
   * @param X0 Initial State
   * @param P0 Initial Covariance
   * @param f Dynamics function that depends on state and control vector
   * @param h Measurement function
   */
  GeneralLIEKF(const LieGroup& X0, const Matrix& P0, Dynamics& f,
               MeasurementFunction& h)
      : LIEKF(X0, P0, h), f_(f) {}

  /**
   * @brief Prediction stage with a dynamics function that calculates the
   * tangent vector xi in the tangent space.
   * @param u Control vector element
   * @param dt Time interval
   * @param Q Process noise covariance matrix.
   */
  void predict(const Control& u, double dt, const Matrix& Q) {
    LieGroup::Jacobian H;
    const TangentVector xi = f_(X, u, H);
    LieGroup U = LieGroup::Expmap(xi * dt);
    auto A = U.inverse().AdjointMap() * H;
    X = X.compose(U);
    P = A * P * A.transpose() + Q;
  }

 private:
  Dynamics f_;  ///< Dynamics function.
};

}  // namespace gtsam
