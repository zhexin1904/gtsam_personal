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

#include <iomanip>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/BiasedFactor.h>
#include <gtsam/slam/AntiFactor.h>

using std::setw, std::cout, std::endl, std::setprecision;

namespace gtsam {

/* ************************************************************************* */
void AugmentedLagrangianState::initializeLagrangeMultipliers(const ConstrainedOptProblem& problem) {
  lambda_e = std::vector<Vector>();
  lambda_e.reserve(problem.eConstraints().size());
  for (const auto& constraint : problem.eConstraints()) {
    lambda_e.push_back(Vector::Zero(constraint->dim()));
  }
  lambda_i = std::vector<double>(problem.iConstraints().size(), 0);
}

/* ************************************************************************* */
std::tuple<AugmentedLagrangianOptimizer::State, double, double>
AugmentedLagrangianOptimizer::iterate(const State& state,
                                      const double mu_e,
                                      const double mu_i) const {
  State new_state(state.iteration + 1);
  new_state.mu_e = mu_e;
  new_state.mu_i = mu_i;

  // Update Lagrangian multipliers.
  updateLagrangeMultiplier(state, new_state);

  // Construct merit function.
  NonlinearFactorGraph dual_graph = LagrangeDualFunction(new_state);

  // Run unconstrained optimization.
  auto optimizer = createUnconstrainedOptimizer(dual_graph, state.values);
  new_state.setValues(optimizer->optimize(), problem_);
  new_state.unconstrained_iters = optimizer->iterations();

  // Update penalty parameters for next iteration.
  double next_mu_e, next_mu_i;
  std::tie(next_mu_e, next_mu_i) = updatePenaltyParameter(state, new_state);

  return {new_state, next_mu_e, next_mu_i};
}

/* ************************************************************************* */
Values AugmentedLagrangianOptimizer::optimize() const {
  /// Construct initial state
  State prev_state;
  State state(0, init_values_, problem_);
  state.initializeLagrangeMultipliers(problem_);
  LogInit(state);

  /// Set penalty parameters for the first iteration.
  double mu_e = p_->initial_mu_e;
  double mu_i = p_->initial_mu_i;

  /// iterates
  do {
    prev_state = std::move(state);
    std::tie(state, mu_e, mu_i) = iterate(prev_state, mu_e, mu_i);
    LogIter(state);
  } while (!checkConvergence(state, prev_state, *p_));

  return state.values;
}

/* ************************************************************************* */
NonlinearFactorGraph AugmentedLagrangianOptimizer::LagrangeDualFunction(
    const State& state, const double epsilon) const {
  // Initialize by adding in cost factors.
  NonlinearFactorGraph graph = problem_.costs();

  // Create factors corresponding to equality constraints.
  const NonlinearEqualityConstraints& e_constraints = problem_.eConstraints();
  const double& mu_e = state.mu_e;
  for (size_t i = 0; i < e_constraints.size(); i++) {
    const auto& constraint = e_constraints.at(i);
    Vector bias = state.lambda_e[i] / mu_e * constraint->sigmas();
    auto penalty_l2 = constraint->penaltyFactor(mu_e);
    graph.emplace_shared<BiasedFactor>(penalty_l2, bias);
  }

  // Create factors corresponding to penalty terms of inequality constraints.
  const NonlinearInequalityConstraints& i_constraints = problem_.iConstraints();
  const double& mu_i = state.mu_i;
  graph.add(i_constraints.penaltyGraphCustom(p_->i_penalty_function, mu_i));

  // Create factors corresponding to Lagrange multiplier terms of i-constraints.
  for (size_t i = 0; i < i_constraints.size(); i++) {
    const auto& constraint = i_constraints.at(i);
    Vector bias = state.lambda_i[i] / epsilon * constraint->sigmas();
    auto penalty_l2 = constraint->penaltyFactorEquality(epsilon);
    graph.emplace_shared<BiasedFactor>(penalty_l2, bias);
    graph.emplace_shared<AntiFactor>(penalty_l2);
  }

  return graph;
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::updateLagrangeMultiplier(const State& prev_state,
                                                            State& state) const {
  // Perform dual ascent on Lagrange multipliers for e-constriants.
  const NonlinearEqualityConstraints& e_constraints = problem_.eConstraints();
  state.lambda_e.resize(e_constraints.size());
  for (size_t i = 0; i < e_constraints.size(); i++) {
    const auto& constraint = e_constraints.at(i);
    // Compute constraint violation as the gradient of the dual function.
    Vector violation = constraint->whitenedError(prev_state.values);
    double step_size =
        std::min(p_->max_dual_step_size_e, prev_state.mu_e * p_->dual_step_size_factor_e);
    state.lambda_e[i] = prev_state.lambda_e[i] + step_size * violation;
  }

  // Perform dual ascent on Lagrange multipliers for i-constriants.
  const NonlinearInequalityConstraints& i_constraints = problem_.iConstraints();
  state.lambda_i.resize(i_constraints.size());
  // Update Lagrangian multipliers.
  for (size_t i = 0; i < i_constraints.size(); i++) {
    const auto& constraint = i_constraints.at(i);
    double violation = constraint->whitenedExpr(prev_state.values)(0);
    double step_size =
        std::min(p_->max_dual_step_size_i, prev_state.mu_i * p_->dual_step_size_factor_i);
    state.lambda_i[i] = std::max(0.0, prev_state.lambda_i[i] + step_size * violation);
  }
}

/* ************************************************************************* */
std::pair<double, double> AugmentedLagrangianOptimizer::updatePenaltyParameter(
    const State& prev_state, const State& state) const {
  double mu_e = state.mu_e;
  if (problem_.eConstraints().size() > 0 &&
      state.violation_e >= p_->mu_increase_threshold * prev_state.violation_e) {
    mu_e *= p_->mu_e_increase_rate;
  }

  double mu_i = state.mu_i;
  if (problem_.iConstraints().size() > 0 &&
      state.violation_i >= p_->mu_increase_threshold * prev_state.violation_i) {
    mu_i *= p_->mu_i_increase_rate;
  }
  return {mu_e, mu_i};
}

/* ************************************************************************* */
ConstrainedOptimizer::SharedOptimizer AugmentedLagrangianOptimizer::createUnconstrainedOptimizer(
    const NonlinearFactorGraph& graph, const Values& values) const {
  // TODO(yetong): make compatible with all NonlinearOptimizers.
  return std::make_shared<LevenbergMarquardtOptimizer>(graph, values, p_->lm_params);
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::LogInit(const State& state) const {
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

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::LogIter(const State& state) const {
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
