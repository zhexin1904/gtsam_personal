/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2024, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * @file TransferFactor.h
 * @brief TransferFactor class
 * @author Frank Dellaert
 * @date October 24, 2024
 */

#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/inference/EdgeKey.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Base class that holds the EdgeKeys and provides the getMatrices method.
 */
template <typename F>
struct TransferEdges {
  EdgeKey edge1, edge2;  ///< The two EdgeKeys.

  TransferEdges(EdgeKey edge1, EdgeKey edge2) : edge1(edge1), edge2(edge2) {}

  // Create Matrix3 objects based on EdgeKey configurations.
  std::pair<Matrix3, Matrix3> getMatrices(const F& F1, const F& F2) const {
    // Fill Fca and Fcb based on EdgeKey configurations.
    if (edge1.i() == edge2.i()) {
      return {F1.matrix(), F2.matrix()};
    } else if (edge1.i() == edge2.j()) {
      return {F1.matrix(), F2.matrix().transpose()};
    } else if (edge1.j() == edge2.i()) {
      return {F1.matrix().transpose(), F2.matrix()};
    } else if (edge1.j() == edge2.j()) {
      return {F1.matrix().transpose(), F2.matrix().transpose()};
    } else {
      throw std::runtime_error(
          "TransferEdges: invalid EdgeKey configuration between edge1 (" +
          std::string(edge1) + ") and edge2 (" + std::string(edge2) + ").");
    }
  }
};

/**
 * Binary factor in the context of Structure from Motion (SfM).
 * It is used to transfer transfer corresponding points from two views to a
 * third based on two fundamental matrices. The factor computes the error
 * between the transferred points `pa` and `pb`, and the actual point `pc` in
 * the target view. Jacobians are done using numerical differentiation.
 */
template <typename F>
class TransferFactor : public NoiseModelFactorN<F, F>, public TransferEdges<F> {
  using Base = NoiseModelFactorN<F, F>;
  using Triplet = std::tuple<Point2, Point2, Point2>;
  std::vector<Triplet> triplets_;  ///< Point triplets.

 public:
  /**
   * @brief Constructor for a single triplet of points.
   *
   * @note Batching all points for the same transfer will be much faster.
   *
   * @param edge1 First EdgeKey specifying F1: (a, c) or (c, a).
   * @param edge2 Second EdgeKey specifying F2: (b, c) or (c, b).
   * @param pa The point in the first view (a).
   * @param pb The point in the second view (b).
   * @param pc The point in the third (and transfer target) view (c).
   * @param model An optional SharedNoiseModel that defines the noise model
   *              for this factor. Defaults to nullptr.
   */
  TransferFactor(EdgeKey edge1, EdgeKey edge2, const Point2& pa,
                 const Point2& pb, const Point2& pc,
                 const SharedNoiseModel& model = nullptr)
      : Base(model, edge1, edge2),
        TransferEdges<F>(edge1, edge2),
        triplets_({std::make_tuple(pa, pb, pc)}) {}

  /**
   * @brief Constructor that accepts a vector of point triplets.
   *
   * @param edge1 First EdgeKey specifying F1: (a, c) or (c, a).
   * @param edge2 Second EdgeKey specifying F2: (b, c) or (c, b).
   * @param triplets A vector of triplets containing (pa, pb, pc).
   * @param model An optional SharedNoiseModel that defines the noise model
   *              for this factor. Defaults to nullptr.
   */
  TransferFactor(EdgeKey edge1, EdgeKey edge2,
                 const std::vector<Triplet>& triplets,
                 const SharedNoiseModel& model = nullptr)
      : Base(model, edge1, edge2),
        TransferEdges<F>(edge1, edge2),
        triplets_(triplets) {}

  /// Vector of errors returns 2*N vector.
  Vector evaluateError(const F& F1, const F& F2,
                       OptionalMatrixType H1 = nullptr,
                       OptionalMatrixType H2 = nullptr) const override {
    std::function<Vector(const F&, const F&)> errorFunction = [&](const F& F1,
                                                                  const F& F2) {
      Vector errors(2 * triplets_.size());
      size_t idx = 0;
      auto [Fca, Fcb] = this->getMatrices(F1, F2);
      for (const auto& tuple : triplets_) {
        const auto& [pa, pb, pc] = tuple;
        Point2 transferredPoint = EpipolarTransfer(Fca, pa, Fcb, pb);
        Vector2 error = transferredPoint - pc;
        errors.segment<2>(idx) = error;
        idx += 2;
      }
      return errors;
    };
    if (H1) *H1 = numericalDerivative21<Vector, F, F>(errorFunction, F1, F2);
    if (H2) *H2 = numericalDerivative22<Vector, F, F>(errorFunction, F1, F2);
    return errorFunction(F1, F2);

  }
};

}  // namespace gtsam