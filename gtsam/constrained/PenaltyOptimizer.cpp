/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PenaltyOptimizer.cpp
 * @brief Penalty method optimization routines.
 * @author: Yetong Zhang
 */

#include <iomanip>
#include <gtsam/constrained/PenaltyOptimizer.h>

using std::setw, std::cout, std::endl, std::setprecision;

namespace gtsam {

/* ********************************************************************************************* */
PenaltyOptimizer::State PenaltyOptimizer::iterate(const State& state) const {
  State new_state(state.iteration + 1);

  // Update penalty parameter.
  new_state.mu_e = state.iteration == 0 ? p_->initial_mu_e : state.mu_e * p_->mu_e_increase_rate;
  new_state.mu_i = state.iteration == 0 ? p_->initial_mu_i : state.mu_i * p_->mu_i_increase_rate;

  // Construct merit function.
  NonlinearFactorGraph merit_graph = meritFunction(new_state.mu_e, new_state.mu_i);

  // Run unconstrained optimization.
  auto optimizer = createUnconstrainedOptimizer(merit_graph, state.values);
  new_state.setValues(optimizer->optimize(), problem_);
  new_state.unconstrained_iters = optimizer->iterations();

  return new_state;
}

/* ********************************************************************************************* */
Values PenaltyOptimizer::optimize() const {
  /// Construct initial state
  State prev_state;
  State state(0, init_values_, problem_);
  LogInit(state);

  do {
    prev_state = std::move(state);
    state = iterate(prev_state);
    LogIter(state);
  } while (!checkConvergence(state, prev_state, *p_));

  return state.values;
}

/* ********************************************************************************************* */
NonlinearFactorGraph PenaltyOptimizer::meritFunction(const double mu_e, const double mu_i) const {
  NonlinearFactorGraph graph = problem_.costs();
  graph.add(problem_.eConstraints().penaltyGraph(mu_e));
  graph.add(problem_.iConstraints().penaltyGraphCustom(p_->i_penalty_function, mu_i));
  return graph;
}

/* ********************************************************************************************* */
PenaltyOptimizer::SharedOptimizer PenaltyOptimizer::createUnconstrainedOptimizer(
    const NonlinearFactorGraph& graph, const Values& values) const {
  // TODO(yetong): make compatible with all NonlinearOptimizers.
  return std::make_shared<LevenbergMarquardtOptimizer>(graph, values, p_->lm_params);
}

/* ********************************************************************************************* */
void PenaltyOptimizer::LogInit(const State& state) const {
  if (p_->verbose) {
    // Log title line.
    cout << setw(10) << "Iter"
         << "|" << setw(10) << "mu_e"
         << "|" << setw(10) << "mu_i"
         << "|" << setw(10) << "cost"
         << "|" << setw(10) << "vio_e"
         << "|" << setw(10) << "vio_i"
         << "|" << setw(10) << "uopt_iters"
         << "|" << setw(10) << "time"
         << "|" << endl;

    // Log initial value line.
    cout << setw(10) << state.iteration;
    cout << "|" << setw(10) << "-";
    cout << "|" << setw(10) << "-";
    cout << "|" << setw(10) << setprecision(4) << state.cost;
    cout << "|" << setw(10) << setprecision(4) << state.violation_e;
    cout << "|" << setw(10) << setprecision(4) << state.violation_i;
    cout << "|" << setw(10) << "-";
    cout << "|" << setw(10) << "-";
    cout << "|" << endl;
  }

  // Store state
  if (p_->store_opt_progress) {
    progress_.emplace_back(state);
  }
}

/* ********************************************************************************************* */
void PenaltyOptimizer::LogIter(const State& state) const {
  if (p_->verbose) {
    cout << setw(10) << state.iteration;
    cout << "|" << setw(10) << state.mu_e;
    cout << "|" << setw(10) << state.mu_i;
    cout << "|" << setw(10) << setprecision(4) << state.cost;
    cout << "|" << setw(10) << setprecision(4) << state.violation_e;
    cout << "|" << setw(10) << setprecision(4) << state.violation_i;
    cout << "|" << setw(10) << state.unconstrained_iters;
    cout << "|" << setw(10) << state.time;
    cout << "|" << endl;
  }

  // Store state
  if (p_->store_opt_progress) {
    progress_.emplace_back(state);
  }
}

}  // namespace gtsam
