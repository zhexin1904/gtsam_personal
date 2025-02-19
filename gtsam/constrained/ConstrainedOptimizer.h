/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptimizer.h
 * @brief Base class constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/** Base class for constrained optimization parameters. */
class GTSAM_EXPORT ConstrainedOptimizerParams {
 public:
  typedef std::shared_ptr<ConstrainedOptimizerParams> shared_ptr;

  size_t max_iterations = 20;  // maximum number of iterations
  double absolute_violation_tolerance = 1e-5;
  double relative_violation_tolerance = 1e-5;
  double absolute_cost_tolerance = 1e-5;
  double relative_cost_tolerance = 1e-5;
  bool verbose = false;
  bool store_opt_progress = false;  // Store detailed info of each iter
  bool store_iter_details = false;  // Store iterations by LM

  /** Constructor. */
  ConstrainedOptimizerParams() {}
};

/** Base class for constrained optimizer state. */
class GTSAM_EXPORT ConstrainedOptimizerState {
 public:
  size_t iteration = 0;
  Values values;
  double cost = 0;
  double violation_e = 0;
  double violation_i = 0;
  double time = 0.0;

  ConstrainedOptimizerState() {}

  ConstrainedOptimizerState(const size_t _iteration) : iteration(_iteration) {}

  ConstrainedOptimizerState(const size_t _iteration, const Values& _values,
                            const ConstrainedOptProblem& problem)
      : iteration(_iteration), values(_values) {
    std::tie(cost, violation_e, violation_i) = problem.evaluate(_values);
  }

  void setValues(const Values& _values, const ConstrainedOptProblem& problem) {
    values = _values;
    std::tie(cost, violation_e, violation_i) = problem.evaluate(_values);
  }

  double violation() const {
    return sqrt(pow(violation_e, 2) + pow(violation_i, 2));
  }
};

/** Base class for constrained optimizer. */
class GTSAM_EXPORT ConstrainedOptimizer {
 public:
  typedef std::shared_ptr<NonlinearOptimizer> SharedOptimizer;
  typedef ConstrainedOptimizerParams Params;
  typedef ConstrainedOptimizerState State;

 protected:
  ConstrainedOptProblem problem_;
  Values init_values_;

 public:
  /** Default constructor. */
  ConstrainedOptimizer() {}

  /** Constructor. */
  ConstrainedOptimizer(const ConstrainedOptProblem& problem,
                       const Values& init_values)
      : problem_(problem), init_values_(init_values) {}

  /** Constructor that packs the cost and constraints into a single factor
   * graph. */
  ConstrainedOptimizer(const NonlinearFactorGraph& graph,
                       const Values& init_values)
      : problem_(graph), init_values_(init_values) {}

  virtual ~ConstrainedOptimizer() {}

  /// Solve a constrained optimization problem.
  virtual Values optimize() const = 0;

 protected:
  virtual bool checkConvergence(const State& state, const State& prev_state,
                                const Params& params) const {
    if (state.iteration >= params.max_iterations) {
      return true;
    }

    if (state.violation() < params.absolute_violation_tolerance &&
        state.cost < params.absolute_cost_tolerance) {
      return true;
    }

    if (abs(state.violation() - prev_state.violation()) <
            params.relative_violation_tolerance &&
        abs(state.cost - prev_state.cost) < params.relative_cost_tolerance) {
      return true;
    }

    return false;
  }
};

}  // namespace gtsam
