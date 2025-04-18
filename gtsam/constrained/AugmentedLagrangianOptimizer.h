/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AugmentedLagrangianOptimizer.h
 * @brief   Augmented Lagrangian method for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>

namespace gtsam {

/// Parameters for Augmented Lagrangian method
class GTSAM_EXPORT AugmentedLagrangianParams : public PenaltyOptimizerParams {
 public:
  typedef PenaltyOptimizerParams Base;
  typedef AugmentedLagrangianParams This;
  typedef std::shared_ptr<AugmentedLagrangianParams> shared_ptr;

  double max_dual_step_size_e = 10;  // maximum step size for dual ascent
  double max_dual_step_size_i = 10;  // maximum step size for dual ascent
  double dual_step_size_factor_e = 1.0;
  double dual_step_size_factor_i = 1.0;
  double mu_increase_threshold = 0.25;

  using Base::Base;
};

/// Details for each iteration.
class GTSAM_EXPORT AugmentedLagrangianState : public PenaltyOptimizerState {
 public:
  typedef PenaltyOptimizerState Base;
  typedef AugmentedLagrangianState This;
  typedef std::shared_ptr<This> shared_ptr;

  std::vector<Vector> lambda_e;  // Lagrange multipliers for e-constraints
  std::vector<double> lambda_i;  // Lagrange multipliers for i-constraints

  /** Initialize Lagrange multipliers as zeros. */
  void initializeLagrangeMultipliers(const ConstrainedOptProblem& problem);

  using Base::Base;
};

/** Augmented Lagrangian method to solve constrained nonlinear least squares
 * problems. The implementation follows
 * https://www.seas.ucla.edu/~vandenbe/133B/lectures/nllseq.pdf for problems
 * with equality constraints only. We further generalize the implementation to
 * incorporate inequality constraints with reference to
 * https://www.stat.cmu.edu/~ryantibs/convexopt/scribes/dual-decomp-scribed.pdf.
 */
class GTSAM_EXPORT AugmentedLagrangianOptimizer : public ConstrainedOptimizer {
 public:
  typedef ConstrainedOptimizer Base;
  typedef PenaltyOptimizer This;
  typedef std::shared_ptr<This> shared_ptr;

  typedef AugmentedLagrangianParams Params;
  typedef AugmentedLagrangianState State;
  typedef std::vector<AugmentedLagrangianState> Progress;

 protected:
  Params::shared_ptr p_;
  mutable Progress progress_;

 public:
  /// Constructor.
  AugmentedLagrangianOptimizer(
      const ConstrainedOptProblem& problem, const Values& init_values,
      Params::shared_ptr p = std::make_shared<Params>())
      : Base(problem, init_values), p_(p), progress_() {}

  /// Constructor that packs costs and constraints into a single factor graph.
  AugmentedLagrangianOptimizer(
      const NonlinearFactorGraph& graph, const Values& init_values,
      Params::shared_ptr p = std::make_shared<Params>())
      : Base(graph, init_values), p_(p), progress_() {}

  /// Run optimization with equality constraints only.
  Values optimize() const override;

  std::tuple<State, double, double> iterate(const State& state,
                                            const double mu_e,
                                            const double mu_i) const;

  /// Return progress of iterations.
  const Progress& progress() const { return progress_; }

  /**
   * Lagrange dual function for equality constraints and inequality constraints
   *   m(x) = 0.5 * ||f(x)||^2 - lambda_e * h(x) + 0.5 * mu_e * ||h(x)||^2
   *                           - lambda_i * g(x) + 0.5 * mu_i * ||g(x)_-||^2
   * To express in nonlinear least squares form, it is rewritten as
   *     m(x) + 0.5d * ||g(x)||^2
   *   = 0.5 * ||f(x)||^2 - lambda_e * h(x) + 0.5 * mu_e * ||h(x)||^2
   *     + 0.5d * ||g(x)||^2 - lambda_i * g(x) + 0.5 * mu_i * ||g(x)_-||^2
   *   = 0.5 * ||f(x)||^2
   *     + 0.5mu_e * ||h(x)- lambda_e/mu_e||^2
   *     + 0.5d * ||g(x)-lambda_i/mu_i||^2
   *     + 0.5mu_i * ||g(x)_-||^2
   *     - c
   * where
   *   c = ||lambda_e||^2 / (2 * mu_e) + ||lambda_i||^2 / (2 * d)
   * is a constant term.
   * Notice: a term (0.5 d * ||g(x)||^2) is added to incorporate Lagrange
   * multiplier terms in the nonlinear least squares form, with d very small.
   * @return: factor graph representing m(x) + 0.5d * ||g(x)||^2 + c
   */
  NonlinearFactorGraph LagrangeDualFunction(const State& state,
                                            const double epsilon = 1.0) const;

 protected:
  SharedOptimizer createUnconstrainedOptimizer(
      const NonlinearFactorGraph& graph, const Values& values) const;

  /** Update the Lagrange multipliers using dual ascent. */
  void updateLagrangeMultiplier(const State& prev_state, State& state) const;

  /// Set the penalty parameters for the state using results from prev state.
  std::pair<double, double> updatePenaltyParameter(const State& prev_state,
                                                   const State& state) const;

  void LogInit(const State& state) const;

  void LogIter(const State& state) const;
};

}  // namespace gtsam
