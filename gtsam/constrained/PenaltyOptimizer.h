/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PenaltyOptimizer.h
 * @brief   Penalty method optimizer for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

/// Parameters for penalty method
class GTSAM_EXPORT PenaltyOptimizerParams : public ConstrainedOptimizerParams {
 public:
  typedef ConstrainedOptimizerParams Base;
  typedef PenaltyOptimizerParams This;
  typedef std::shared_ptr<This> shared_ptr;

  double initial_mu_e = 1.0;      // initial penalty parameter
  double initial_mu_i = 1.0;      // initial penalty parameter
  double mu_e_increase_rate = 2;  // increase rate of penalty parameter
  double mu_i_increase_rate = 2;
  InequalityPenaltyFunction::shared_ptr i_penalty_function = nullptr;
  LevenbergMarquardtParams lm_params;
  std::vector<LevenbergMarquardtParams>
      iters_lm_params;  // use different lm parameters for different iterations.

  /** Constructor. */
  PenaltyOptimizerParams() : Base() {}
};

/// Details for each iteration.
class GTSAM_EXPORT PenaltyOptimizerState : public ConstrainedOptimizerState {
 public:
  typedef ConstrainedOptimizerState Base;
  typedef PenaltyOptimizerState This;
  typedef std::shared_ptr<This> shared_ptr;

  double mu_e = 0.0;
  double mu_i = 0.0;
  size_t unconstrained_iters = 0;

  using Base::Base;
};

/// Penalty method only considering equality constraints.
class GTSAM_EXPORT PenaltyOptimizer : public ConstrainedOptimizer {
 public:
  typedef ConstrainedOptimizer Base;
  typedef PenaltyOptimizer This;
  typedef std::shared_ptr<This> shared_ptr;

  typedef PenaltyOptimizerParams Params;
  typedef PenaltyOptimizerState State;
  typedef std::vector<PenaltyOptimizerState> Progress;

 protected:
  Params::shared_ptr p_;
  mutable Progress progress_;

 public:
  /// Constructor.
  PenaltyOptimizer(const ConstrainedOptProblem& problem,
                   const Values& init_values,
                   Params::shared_ptr p = std::make_shared<Params>())
      : Base(problem, init_values), p_(p), progress_() {}

  /// Constructor that packs costs and constraints into a single factor graph.
  PenaltyOptimizer(const NonlinearFactorGraph& graph,
                   const Values& init_values,
                   Params::shared_ptr p = std::make_shared<Params>())
      : Base(graph, init_values), p_(p), progress_() {}

  /// Run optimization with equality constraints only.
  Values optimize() const override;

  State iterate(const State& state) const;

  /// Return progress of iterations.
  const Progress& progress() const { return progress_; }

 protected:
  /// Merit function in the form
  ///  m(x) = f(x) + 0.5 mu_e * ||h(x)||^2 + 0.5 mu_i ||g(x)_+||^2
  NonlinearFactorGraph meritFunction(const double mu_e, const double mu_i) const;

  SharedOptimizer createUnconstrainedOptimizer(const NonlinearFactorGraph& graph,
                                               const Values& values) const;

  void LogInit(const State& state) const;

  void LogIter(const State& state) const;
};

}  // namespace gtsam
