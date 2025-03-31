/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDCSAM.cpp
 * @brief   Tests for the DCSAM optimization algorithm
 * @author  Kevin Doherty
 * @author  Varun Agrawal
 * @date    March 2025
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/inference/BayesNet-inst.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

// #include <iomanip>

// // Our custom DCSAM includes
// #include "dcsam/DCContinuousFactor.h"
// #include "dcsam/DCDiscreteFactor.h"
// #include "dcsam/DCEMFactor.h"
// #include "dcsam/DCMaxMixtureFactor.h"
// #include "dcsam/DCMixtureFactor.h"
// #include "dcsam/DCSAM.h"
// #include "dcsam/DiscretePriorFactor.h"
// #include "dcsam/SemanticBearingRangeFactor.h"
// #include "dcsam/SmartDiscretePriorFactor.h"

using namespace gtsam;

const double tol = 1e-7;

/******************************************************************************/

/*
 * Test simple DiscretePriorFactor. We construct a DiscreteFactor factor
 * graph with a single binary variable d1 and a single custom
 * DiscretePriorFactor p(d1) specifying the distribution:
 * p(d1 = 0) = 0.1, p(d1 = 1) = 0.9.
 *
 * After constructing the factor graph, we validate that the marginals are
 * correct (i.e. they match the input p(d1)) and the most probable estimate
 * (computed by maximizing p(d1) over possible values of d1) is 1.
 */
TEST(DCSAM, DiscretePriorFactor) {
  // Make an empty discrete factor graph
  DiscreteFactorGraph dfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(Symbol('d', 1), cardinality);
  const std::vector<double> probs{0.1, 0.9};

  // Make a discrete prior factor and add it to the graph
  DecisionTreeFactor dpf(dk, probs);
  dfg.push_back(dpf);

  // Solve
  DiscreteValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  const size_t mpeD = mostProbableEstimate.at(dk.first);

  // Get the marginals
  DiscreteMarginals discreteMarginals(dfg);
  Vector margProbs = discreteMarginals.marginalProbabilities(dk);

  // Verify that each marginal probability is within `tol` of the true marginal
  for (size_t i = 0; i < dk.second; i++) {
    bool margWithinTol = (abs(margProbs[i] - probs[i]) < tol);
    EXPECT(margWithinTol);
  }

  // Ensure that the most probable estimate is correct
  EXPECT_LONGS_EQUAL(mpeD, 1);
}

DecisionTreeFactor DiscreteFactorFromErrors(
    const DiscreteKeys &discreteKeys,
    const AlgebraicDecisionTree<Key> &errors) {
  double min_log = errors.min();
  AlgebraicDecisionTree<Key> potentials(
      errors, [&min_log](const double x) { return exp(-(x - min_log)); });

  return DecisionTreeFactor(discreteKeys, potentials);
}

/*
 * Test discrete optimization using a simple mixture.
 *
 * Construct a single factor (for a single variable) consisting of a
 * discrete-conditional mixture.
 * Here we have a "null hypothesis" consisting of
 * a Gaussian with large variance and an "alternative hypothesis"
 * consisting of a Gaussian with smaller variance.
 * After initializing the continuous variable far away from
 * the ground-truth solution (x1 = 0), the discrete hypothesis selector
 * will initially choose the null hypothesis.
 */
TEST(DCSAM, DiscreteMixture) {
  // Make an empty hybrid factor graph
  HybridNonlinearFactorGraph dcfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(Symbol('d', 1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  Symbol x1 = Symbol('x', 1);
  KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis
  const double loc = 0.0;
  const double sigma1 = 1.0;
  auto prior_noise1 = noiseModel::Isotropic::Sigma(1, sigma1);
  auto f1 = std::make_shared<PriorFactor<double>>(x1, loc, prior_noise1);

  // Make a factor for null hypothesis
  const double sigmaNullHypo = 8.0;
  auto prior_noiseNullHypo = noiseModel::Isotropic::Sigma(1, sigmaNullHypo);
  auto fNullHypo =
      std::make_shared<PriorFactor<double>>(x1, loc, prior_noiseNullHypo);

  std::vector<NoiseModelFactor::shared_ptr> factorComponents{f1, fNullHypo};

  HybridNonlinearFactor dcMixture(dk, factorComponents);
  dcfg.push_back(dcMixture);

  DiscreteKey dkTest = dcMixture.discreteKeys()[0];
  std::cout << "DK 1st: " << DefaultKeyFormatter(dkTest.first) << std::endl;
  std::cout << "DK 2nd: " << dkTest.second << std::endl;

  // Let's make an initial guess for the continuous values
  Values initialGuess;
  double initVal = -2.5;
  initialGuess.insert(x1, initVal);

  // We also need an initial guess for the discrete variables (this will only be
  // used if it is needed by your factors), here it is ignored.
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 0;

  // Let's make a discrete factor graph
  DiscreteFactorGraph dfg;

  auto calculateError = [&](const NonlinearFactorValuePair &pair) -> double {
    if (pair.first) {
      auto gaussianNoiseModel = std::dynamic_pointer_cast<noiseModel::Gaussian>(
          pair.first->noiseModel());
      // `error` has the following contributions:
      // - the scalar is the sum of all mode-dependent constants
      // - factor->error(initial) is the error on the initial values
      // - negLogK is log normalization constant from the noise model
      return pair.second + pair.first->error(initialGuess) +
             gaussianNoiseModel->negLogConstant();
    } else {
      // If the factor has been pruned, return infinite error
      return std::numeric_limits<double>::infinity();
    }
  };
  AlgebraicDecisionTree<Key> errors(dcMixture.factors(), calculateError);
  DecisionTreeFactor dtf =
      DiscreteFactorFromErrors(dcMixture.discreteKeys(), errors);

  dfg.push_back(dtf);

  // Solve for discrete given continuous
  DiscreteValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  const size_t mpeD = mostProbableEstimate.at(dk.first);

  // Get the marginals
  DiscreteMarginals newDiscreteMarginals(dfg);
  Vector newMargProbs = newDiscreteMarginals.marginalProbabilities(dk);

  // Ensure that the prediction is correct
  EXPECT_LONGS_EQUAL(mpeD, 1);
}

/*
 * Test continuous optimization using a simple mixture
 *
 * Construct a single factor (for a single variable) consisting of a
 * discrete-conditional mixture. Here we have a "null hypothesis" consisting
 of
 * a Gaussian with large variance and an "alternative hypothesis" consisting
 of
 * a Gaussian with smaller variance. After initializing the continuous
 variable
 * far away from the ground-truth solution (x1 = 0), the discrete hypothesis
 * selector will initially choose the null hypothesis.
 *
 * After a step of continuous optimization, the continuous solution will move
 to
 * x1 = 0 (since the problem is linear, we have convergence in one step).
 * Finally, updating the discrete value will show that the correct discrete
 * hypothesis will shift to the "alternative hypothesis."
 */
TEST(DCSAM, ContinuousMixture) {
  // Make an empty hybrid factor graph
  HybridNonlinearFactorGraph dcfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(Symbol('d', 1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  Symbol x1 = Symbol('x', 1);
  KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis
  const double loc = 0.0;
  const double sigma1 = 1.0;
  auto prior_noise1 = noiseModel::Isotropic::Sigma(1, sigma1);
  auto f1 = std::make_shared<PriorFactor<double>>(x1, loc, prior_noise1);

  // Make a factor for null hypothesis
  const double sigmaNullHypo = 8.0;
  auto prior_noiseNullHypo = noiseModel::Isotropic::Sigma(1, sigmaNullHypo);
  auto fNullHypo =
      std::make_shared<PriorFactor<double>>(x1, loc, prior_noiseNullHypo);

  std::vector<NonlinearFactorValuePair> factorComponents{
      {f1, prior_noise1->negLogConstant()},
      {fNullHypo, prior_noiseNullHypo->negLogConstant()}};

  HybridNonlinearFactor dcMixture(dk, factorComponents);
  dcfg.push_back(dcMixture);

  // Test calculation of negative log probability
  DiscreteValues dv1, dvNH;
  dv1[dk.first] = 0;
  dvNH[dk.first] = 1;
  Values xvals;
  xvals.insert(x1, 0.0);
  const double negLogProb_1 = dcMixture.error(xvals, dv1);
  const double negLogProb_NH = dcMixture.error(xvals, dvNH);

  // As calculated in MATLAB using -log(normpdf(0,0,1)), -log(normpdf(0,0,8))
  EXPECT_DOUBLES_EQUAL(0.9189, negLogProb_1, 1e-3);
  EXPECT_DOUBLES_EQUAL(2.9984, negLogProb_NH, 1e-3);

  // Let's make an initial guess for the continuous values
  Values values;
  double initVal = -2.5;
  values.insert(x1, initVal);

  // We also need an initial guess for the discrete variables (this will only be
  // used if it is needed by your factors), here it is ignored.
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 0;

  // Let's make some factor graphs
  DiscreteFactorGraph dfg;
  NonlinearFactorGraph nfg;

  // Create DiscreteFactor from initial guess
  auto calculateError =
      [&values](const NonlinearFactorValuePair &pair) -> double {
    if (pair.first) {
      auto gaussianNoiseModel = std::dynamic_pointer_cast<noiseModel::Gaussian>(
          pair.first->noiseModel());
      // `error` has the following contributions:
      // - the scalar is the sum of all mode-dependent constants
      // - factor->error(initial) is the error on the initial values
      // - negLogK is log normalization constant from the noise model
      return pair.second + pair.first->error(values);
    } else {
      // If the factor has been pruned, return infinite error
      return std::numeric_limits<double>::infinity();
    }
  };
  AlgebraicDecisionTree<Key> errors(dcMixture.factors(), calculateError);
  DecisionTreeFactor dtf =
      DiscreteFactorFromErrors(dcMixture.discreteKeys(), errors);

  dfg.push_back(dtf);

  // Solve for discrete given continuous
  DiscreteValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  size_t mpeD = mostProbableEstimate.at(dk.first);

  // From previous test we know this is == 1
  EXPECT_LONGS_EQUAL(1, mpeD);

  // Get the marginals
  DiscreteMarginals newDiscreteMarginals(dfg);
  Vector newMargProbs = newDiscreteMarginals.marginalProbabilities(dk);

  // Use discrete info to get nonlinear factors
  nfg.push_back(dcMixture.factors()(mostProbableEstimate).first);

  // Setup isam
  ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_params.setOptimizationParams(ISAM2DoglegParams());
  ISAM2 isam(isam_params);
  isam.update(nfg, values);

  // Solve for updated continuous value
  values = isam.calculateEstimate();

  // Now update the continuous info in the discrete solver
  errors = AlgebraicDecisionTree<Key>(dcMixture.factors(), calculateError);
  dtf = DiscreteFactorFromErrors(dcMixture.discreteKeys(), errors);

  dfg.resize(0);
  dfg.push_back(dtf);

  // Re-solve discrete to verify that output has switched
  mostProbableEstimate = dfg.optimize();
  mpeD = mostProbableEstimate.at(dk.first);

  // Ensure that the prediction is correct
  EXPECT_LONGS_EQUAL(0, mpeD);
}

/* *************************************************************************
 */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* *************************************************************************
 */
