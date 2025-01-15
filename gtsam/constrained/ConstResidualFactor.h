/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstResidualFactor.h
 * @brief Factor with added bias vector.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {
class GTSAM_EXPORT ConstResidualFactor : public NonlinearFactor {
 public:
  typedef NonlinearFactor Base;
  typedef ConstResidualFactor This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  double residual_;

 public:
  ConstResidualFactor(const double residual) : Base(KeyVector()), residual_(residual) {}

  double error(const Values& c) const override { return residual_; }

  size_t dim() const override { return 1; }

  virtual std::shared_ptr<GaussianFactor> linearize(const Values& c) const override {
    Vector b = Vector1(sqrt(2 * residual_));
    return std::make_shared<JacobianFactor>(b);
  }
};
}  // namespace gtsam