/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    MeritFunction.h
 * @brief   Merit function for nonlinear constrained optimization problem.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptProblem.h>

namespace gtsam {

/** Base class for merit function. */
class GTSAM_EXPORT MeritFunction {
 public:
  typedef MeritFunction This;
  typedef std::shared_ptr<This> shared_ptr;

  MeritFunction() {}
  virtual ~MeritFunction() {}

  /** construct the merit function of a given constrained optimization problem. */
  virtual NonlinearFactorGraph construct(const ConstrainedOptProblem& problem) const {
    return problem.costs();
  }

  virtual double evaluate(const ConstrainedOptProblem& problem, const Values& values) const {
    return construct(problem).error(values);
  }
};

class GTSAM_EXPORT MeritFunctionL2 : public MeritFunction {
 public:
  typedef MeritFunction Base;
  typedef MeritFunctionL2 This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  double mu_e_ = 0.0;
  double mu_i_ = 0.0;
  InequalityPenaltyFunction::shared_ptr i_penalty_function_ = nullptr;

 public:
  MeritFunctionL2(double mu_e,
                  double mu_i,
                  InequalityPenaltyFunction::shared_ptr i_penalty_function = nullptr)
      : Base(), mu_e_(mu_e), mu_i_(mu_i), i_penalty_function_(i_penalty_function) {}

  virtual NonlinearFactorGraph construct(const ConstrainedOptProblem& problem) const override;
};

class GTSAM_EXPORT MeritFunctionL1L2 : public MeritFunction {
 public:
  typedef MeritFunction Base;
  typedef MeritFunctionL2 This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  double mu_e_l1_;
  double mu_e_l2_;
  double mu_i_l1_;
  double mu_i_l2_;

 public:
  virtual NonlinearFactorGraph construct(const ConstrainedOptProblem& problem) const override;
};

class GTSAM_EXPORT AugmentedLagrangianFunction : public MeritFunction {
 public:
  typedef MeritFunction Base;
  typedef MeritFunctionL2 This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  double mu_e_;
  double mu_i_;
};

}  // namespace gtsam
