/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteBoundaryFactor.cpp
 *  @date Mach, 2025
 *  @author Varun Agrawal
 */

#include <gtsam/hybrid/DiscreteBoundaryFactor.h>

namespace gtsam {

/* ************************************************************************ */
DecisionTreeFactor DiscreteFactorFromErrors(
    const DiscreteKeys& discreteKeys,
    const AlgebraicDecisionTree<Key>& errors) {
  double min_log = errors.min();
  AlgebraicDecisionTree<Key> potentials(
      errors, [&min_log](const double x) { return exp(-(x - min_log)); });

  return DecisionTreeFactor(discreteKeys, potentials);
}

/* ************************************************************************ */
DecisionTreeFactor DiscreteBoundaryFactor::ComputeDiscreteBoundary(
    const DiscreteKeys& dkeys,
    const HybridNonlinearFactor::FactorValuePairs& factors,
    const gtsam::Values& values) {
  auto calculateError = [&](const NonlinearFactorValuePair& pair) -> double {
    if (pair.first) {
      auto gaussianNoiseModel = std::dynamic_pointer_cast<noiseModel::Gaussian>(
          pair.first->noiseModel());
      // `error` has the following contributions:
      // - the scalar is the sum of all mode-dependent constants
      // - factor->error(initial) is the error on the initial values
      // - negLogK is log normalization constant from the noise model
      return pair.second + pair.first->error(values) +
             gaussianNoiseModel->negLogConstant();
    } else {
      // If the factor has been pruned, return infinite error
      return std::numeric_limits<double>::infinity();
    }
  };
  AlgebraicDecisionTree<Key> errors(factors, calculateError);
  return DiscreteFactorFromErrors(dkeys, errors);
}

/* ************************************************************************ */
void DiscreteBoundaryFactor::print(const std::string& s,
                                   const KeyFormatter& formatter) const {
  Base::print(s, formatter);
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr DiscreteBoundaryFactor::operator*(double s) const {
  auto dtf = std::static_pointer_cast<DecisionTreeFactor>(Base::operator*(s));
  return std::make_shared<DiscreteBoundaryFactor>(discreteKeys(), *dtf);
}

/* ************************************************************************ */
DiscreteBoundaryFactor DiscreteBoundaryFactor::operator*(
    const DiscreteBoundaryFactor& f) const {
  return apply(f, Ring::mul);
}

/* ************************************************************************ */
DiscreteBoundaryFactor DiscreteBoundaryFactor::operator/(
    const DiscreteBoundaryFactor& f) const {
  return apply(f, safe_div);
}

/* ************************************************************************ */
DiscreteBoundaryFactor DiscreteBoundaryFactor::apply(Unary op) const {
  // apply operand
  ADT result = ADT::apply(op);
  // Make a new factor
  return DiscreteBoundaryFactor(discreteKeys(), result);
}

/* ************************************************************************ */
DiscreteBoundaryFactor DiscreteBoundaryFactor::apply(UnaryAssignment op) const {
  // apply operand
  ADT result = ADT::apply(op);
  // Make a new factor
  return DiscreteBoundaryFactor(discreteKeys(), result);
}

/* ************************************************************************ */
DiscreteBoundaryFactor DiscreteBoundaryFactor::apply(
    const DiscreteBoundaryFactor& f, Binary op) const {
  DecisionTreeFactor result = Base::apply(f, op);
  return DiscreteBoundaryFactor(discreteKeys(), result);
}

}  // namespace gtsam