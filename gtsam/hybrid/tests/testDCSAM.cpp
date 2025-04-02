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
#include <gtsam/hybrid/DCSAM.h>
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

using namespace gtsam;

using symbol_shorthand::C;
using symbol_shorthand::D;
using symbol_shorthand::L;
using symbol_shorthand::X;

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
  DiscreteKey dk(D(1), cardinality);
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
  DiscreteKey dk(D(1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  Key x1 = X(1);
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
  DiscreteKey dk(D(1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  Key x1 = X(1);
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

/*
 * Test full DCSAM solve on DCMixtureFactor for 1D case.
 *
 * This is essentially identical to the `DCContinuousFactor` test above,
 but the
 * solution is obtained by calling the `DCSAM::update` functions (rather
 than
 * implemented manually as above).
 */
TEST(DCSAM, simple_mixture_factor) {
  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(D(1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  Key x1 = X(1);
  KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis
  double loc = 0.0;
  double sigma1 = 1.0;
  auto prior_noise1 = noiseModel::Isotropic::Sigma(1, sigma1);
  auto f1 = std::make_shared<PriorFactor<double>>(x1, loc, prior_noise1);

  // Make a factor for null hypothesis
  double sigmaNullHypo = 8.0;
  auto prior_noiseNullHypo = noiseModel::Isotropic::Sigma(1, sigmaNullHypo);
  auto fNullHypo =
      std::make_shared<PriorFactor<double>>(x1, loc, prior_noiseNullHypo);

  std::vector<NonlinearFactorValuePair> factorComponents{
      {f1, prior_noise1->negLogConstant()},
      {fNullHypo, prior_noiseNullHypo->negLogConstant()}};

  HybridNonlinearFactor dcMixture(dk, factorComponents);

  // Make an empty hybrid factor graph
  HybridNonlinearFactorGraph hfg;

  hfg.push_back(dcMixture);

  // Let's make an initial guess
  Values initialGuess;
  double initVal = -2.5;
  initialGuess.insert(x1, initVal);

  // We also need an initial guess for the discrete variables
  // (this will only be used if it is needed by your factors),
  // here it is ignored.
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 0;

  // Let's make a solver
  DCSAM dcsam;

  // Add the HybridFactorGraph to DCSAM
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  // Solve
  HybridValues dcvals = dcsam.calculateEstimate();

  // Run another iteration
  dcsam.update();

  // Update HybridValues
  dcvals = dcsam.calculateEstimate();

  // Ensure that the prediction is correct
  const size_t mpeD = dcvals.discrete().at(dk.first);
  EXPECT_LONGS_EQUAL(0, mpeD);
}

/**
 * This is a basic (qualitative) octagonal pose graph SLAM test
 * to verify that DCSAM works on standard SLAM examples.
 */
TEST(DCSAM, SimpleSlamBatch) {
  // Make a hybrid factor graph
  HybridNonlinearFactorGraph graph;

  // Values for initial guess
  Values initialGuess;

  Key x0 = X(0);
  Pose2 pose0(0, 0, 0);
  Pose2 dx(1, 0, 0.78539816);
  const double prior_sigma = 0.1;
  const double meas_sigma = 1.0;

  auto prior_noise = noiseModel::Isotropic::Sigma(3, prior_sigma);
  auto meas_noise = noiseModel::Isotropic::Sigma(3, meas_sigma);

  PriorFactor<Pose2> p0(x0, pose0, prior_noise);
  graph.push_back(p0);

  initialGuess.insert(x0, pose0);

  // Setup dcsam
  DCSAM dcsam;

  Pose2 odom(pose0);
  Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    Key xi = X(i);
    Key xj = X(i + 1);

    Pose2 meas = dx * noise;

    BetweenFactor<Pose2> bw(xi, xj, dx * noise, meas_noise);
    graph.push_back(bw);

    odom = odom * meas;
    initialGuess.insert(xj, odom);
  }

  BetweenFactor<Pose2> bw(x0, X(7), dx * noise, meas_noise);

  dcsam.update(graph, initialGuess);
  HybridValues dcvals = dcsam.calculateEstimate();

  // Values from the DCSAM repo
  Values expectedContinuous;
  expectedContinuous.insert<Pose2>(
      X(0), Pose2(-3.53708e-33, -8.97559e-34, 4.1955e-33));
  expectedContinuous.insert<Pose2>(X(1), Pose2(1, 0.0141421, 0.795398));
  expectedContinuous.insert<Pose2>(X(2), Pose2(1.6899, 0.738184, 1.5908));
  expectedContinuous.insert<Pose2>(X(3), Pose2(1.65576, 1.7377, 2.38619));
  expectedContinuous.insert<Pose2>(X(4), Pose2(0.918069, 2.41298, -3.10159));
  expectedContinuous.insert<Pose2>(X(5), Pose2(-0.0805657, 2.35886, -2.30619));
  expectedContinuous.insert<Pose2>(X(6), Pose2(-0.740961, 1.60781, -1.5108));
  expectedContinuous.insert<Pose2>(X(7), Pose2(-0.66688, 0.61046, -0.715398));

  EXPECT(assert_equal(expectedContinuous, dcvals.nonlinear(), 1e-5));
}

/**
 * This is a basic qualitative octagonal pose graph SLAM test
 * to verify that DCSAM works on standard SLAM examples
 * in the *incremental* setting.
 */
TEST(DCSAM, SimpleSlamIncremental) {
  // Make a factor graph
  HybridNonlinearFactorGraph graph;

  // Values for initial guess
  Values initialGuess;

  Key x0 = X(0);
  Pose2 pose0(0, 0, 0);
  Pose2 dx(1, 0, 0.78539816);
  const double prior_sigma = 0.1;
  const double meas_sigma = 1.0;

  auto prior_noise = noiseModel::Isotropic::Sigma(3, prior_sigma);
  auto meas_noise = noiseModel::Isotropic::Sigma(3, meas_sigma);

  PriorFactor<Pose2> p0(x0, pose0, prior_noise);

  initialGuess.insert(x0, pose0);
  graph.push_back(p0);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(graph, initialGuess);

  graph.resize(0);
  initialGuess.clear();

  Pose2 odom(pose0);
  Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    Key xi = X(i);
    Key xj = X(i + 1);

    Pose2 meas = dx * noise;

    BetweenFactor<Pose2> bw(xi, xj, dx * noise, meas_noise);
    graph.push_back(bw);

    odom = odom * meas;
    initialGuess.insert(xj, odom);
    dcsam.update(graph, initialGuess);

    graph.resize(0);
    initialGuess.clear();
  }

  BetweenFactor<Pose2> bw(x0, X(7), dx * noise, meas_noise);

  dcsam.update(graph, initialGuess);
  HybridValues dcvals = dcsam.calculateEstimate();

  // Values from the DCSAM repo
  Values expectedContinuous;
  expectedContinuous.insert<Pose2>(
      X(0), Pose2(-3.53708e-33, -8.97559e-34, 4.1955e-33));
  expectedContinuous.insert<Pose2>(X(1), Pose2(1, 0.0141421, 0.795398));
  expectedContinuous.insert<Pose2>(X(2), Pose2(1.6899, 0.738184, 1.5908));
  expectedContinuous.insert<Pose2>(X(3), Pose2(1.65576, 1.7377, 2.38619));
  expectedContinuous.insert<Pose2>(X(4), Pose2(0.918069, 2.41298, -3.10159));
  expectedContinuous.insert<Pose2>(X(5), Pose2(-0.0805657, 2.35886, -2.30619));
  expectedContinuous.insert<Pose2>(X(6), Pose2(-0.740961, 1.60781, -1.5108));
  expectedContinuous.insert<Pose2>(X(7), Pose2(-0.66688, 0.61046, -0.715398));

  EXPECT(assert_equal(expectedContinuous, dcvals.nonlinear(), 1e-5));
}

/**
 * This test is a sanity check that DCSAM works correctly for solely
 discrete problems
 *
 */
TEST(DCSAM, SimpleDiscrete) {
  // Create a DCSAM instance
  DCSAM dcsam;

  // Make an empty hybrid factor graph
  HybridNonlinearFactorGraph hfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(D(1), cardinality);
  const std::vector<double> probs{0.1, 0.9};

  // Make a discrete prior factor and add it to the graph
  DecisionTreeFactor dpf(dk, probs);
  hfg.push_back(dpf);

  // Initial guess for discrete values (only used in certain circumstances)
  DiscreteValues initialGuessDiscrete;
  initialGuessDiscrete[dk.first] = 1;

  // Update DCSAM with the new factor
  dcsam.update(hfg, initialGuessDiscrete);

  // Solve
  HybridValues dcvals = dcsam.calculateEstimate();

  // Get the most probable estimate
  const size_t mpeD = dcvals.discrete().at(dk.first);

  // Ensure that the prediction is correct
  EXPECT_LONGS_EQUAL(1, mpeD);
}

/**
 * This is a basic qualitative octagonal pose graph SLAM test with two
 * *semantic* landmarks to verify that DCSAM works on standard SLAM
 examples in
 * the *incremental* setting
 */
TEST(DCSAM, SimpleSemanticSlam) {
  // Make a factor graph
  HybridNonlinearFactorGraph hfg;

  // Values for initial guess
  Values initialGuess;
  // Initial guess for discrete values (only used in certain circumstances)
  DiscreteValues initialGuessDiscrete;

  Key x0 = X(0);
  Key l1 = L(1);
  Key lc1 = C(1);
  // Create a discrete key for landmark 1 class with cardinality 2.
  DiscreteKey lm1_class(lc1, 2);
  Pose2 pose0(0, 0, 0);
  Pose2 dx(1, 0, 0.78539816);
  const double prior_sigma = 0.1;
  const double meas_sigma = 1.0;
  const double circumradius = (std::sqrt(4 + 2 * std::sqrt(2))) / 2.0;
  Point2 landmark1(circumradius, circumradius);

  auto prior_noise = noiseModel::Isotropic::Sigma(3, prior_sigma);
  auto prior_lm_noise = noiseModel::Isotropic::Sigma(2, prior_sigma);
  auto meas_noise = noiseModel::Isotropic::Sigma(3, meas_sigma);

  // 0.1 rad std on bearing, 10cm on range
  auto br_noise = noiseModel::Isotropic::Sigma(2, 0.1);

  std::vector<double> prior_lm1_class;
  prior_lm1_class.push_back(0.9);
  prior_lm1_class.push_back(0.1);

  PriorFactor<Pose2> p0(x0, pose0, prior_noise);
  // PriorFactor<Point2> pl1(l1, landmark1, prior_lm_noise);

  // add prior on discrete landmark
  DecisionTreeFactor plc1(lm1_class, prior_lm1_class);

  initialGuess.insert(x0, pose0);
  initialGuess.insert(l1, landmark1);

  // Set initial guess for discrete class var (ignored internally; only used for
  // MaxMixtures and SumMixtures)
  initialGuessDiscrete[lm1_class.first] = 0;

  hfg.push_back(p0);
  hfg.push_back(plc1);

  // Setup dcsam
  DCSAM dcsam;
  dcsam.update(hfg, initialGuess, initialGuessDiscrete);

  HybridValues dcval_start = dcsam.calculateEstimate();

  hfg.resize(0);
  initialGuess.clear();
  initialGuessDiscrete.clear();

  Pose2 odom(pose0);
  Pose2 noise(0.01, 0.01, 0.01);
  for (size_t i = 0; i < 7; i++) {
    Key xi = X(i);
    Key xj = X(i + 1);

    Pose2 meas = dx * noise;

    BetweenFactor<Pose2> bw(xi, xj, meas, meas_noise);
    hfg.push_back(bw);

    // Add bearing range measurement to landmark in center
    Rot2 bearing1 = Rot2::fromDegrees(67.5);
    const double range1 = circumradius;
    hfg.push_back(
        BearingRangeFactor<Pose2, Point2>(xi, l1, bearing1, range1, br_noise));

    // Add semantic measurement to landmark in center
    // For the first couple measurements, pick class=0, later pick class=1
    std::vector<double> semantic_meas;
    if (i < 2) {
      semantic_meas.push_back(0.9);
      semantic_meas.push_back(0.1);
    } else {
      semantic_meas.push_back(0.1);
      semantic_meas.push_back(0.9);
    }
    DecisionTreeFactor dpf(lm1_class, semantic_meas);
    hfg.push_back(dpf);

    odom = odom * meas;
    initialGuess.insert(xj, odom);
    dcsam.update(hfg, initialGuess);

    HybridValues dcvals = dcsam.calculateEstimate();

    hfg.resize(0);
    initialGuess.clear();
  }

  BetweenFactor<Pose2> bw(x0, X(7), dx * noise, meas_noise);

  hfg.push_back(bw);
  dcsam.update(hfg, initialGuess);

  HybridValues dcvals = dcsam.calculateEstimate();

  // Values copied from the DCSAM repo
  DiscreteValues dv;
  dv[lc1] = 1;

  Values nonlinear;
  nonlinear.insert(L(1), Point2(0.501542, 1.20546));
  nonlinear.insert(X(0), Pose2(0.00324255, 0.00959038, -0.0170418));
  nonlinear.insert(X(1), Pose2(1.04653, 0.020688, 0.826187));
  nonlinear.insert(X(2), Pose2(1.7428, 0.798357, 1.6467));
  nonlinear.insert(X(3), Pose2(1.64173, 1.8505, 2.47626));
  nonlinear.insert(X(4), Pose2(0.802545, 2.48895, -2.98168));
  nonlinear.insert(X(5), Pose2(-0.240369, 2.30667, -2.15643));
  nonlinear.insert(X(6), Pose2(-0.819911, 1.4135, -1.3317));
  nonlinear.insert(X(7), Pose2(-0.689416, 0.344402, -0.456848));

  HybridValues expectedValues(VectorValues(), dv, nonlinear);
  EXPECT(assert_equal(expectedValues, dcvals, 1e-5));
}

/* *************************************************************************
 */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* *************************************************************************
 */
