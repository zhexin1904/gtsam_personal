/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridNonlinearFactorGraph.cpp
 * @brief   Unit tests for HybridNonlinearFactorGraph
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/TableDistribution.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ****************************************************************************
 * Test that any linearizedFactorGraph gaussian factors are appended to the
 * existing gaussian factor graph in the hybrid factor graph.
 */
TEST(HybridNonlinearFactorGraph, GaussianFactorGraph) {
  HybridNonlinearFactorGraph fg;

  // Add a simple prior factor to the nonlinear factor graph
  fg.emplace_shared<PriorFactor<double>>(X(0), 0, Isotropic::Sigma(1, 0.1));

  // Linearization point
  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);

  // Linearize the factor graph.
  HybridGaussianFactorGraph ghfg = *fg.linearize(linearizationPoint);
  EXPECT_LONGS_EQUAL(1, ghfg.size());

  // Check that the error is the same for the nonlinear values.
  const VectorValues zero{{X(0), Vector1(0)}};
  const HybridValues hybridValues{zero, {}, linearizationPoint};
  EXPECT_DOUBLES_EQUAL(fg.error(hybridValues), ghfg.error(hybridValues), 1e-9);
}

/***************************************************************************
 * Test equality for Hybrid Nonlinear Factor Graph.
 */
TEST(HybridNonlinearFactorGraph, Equals) {
  HybridNonlinearFactorGraph graph1;
  HybridNonlinearFactorGraph graph2;

  // Test empty factor graphs
  EXPECT(assert_equal(graph1, graph2));

  auto f0 = std::make_shared<PriorFactor<Pose2>>(
      1, Pose2(), noiseModel::Isotropic::Sigma(3, 0.001));
  graph1.push_back(f0);
  graph2.push_back(f0);

  auto f1 = std::make_shared<BetweenFactor<Pose2>>(
      1, 2, Pose2(), noiseModel::Isotropic::Sigma(3, 0.1));
  graph1.push_back(f1);
  graph2.push_back(f1);

  // Test non-empty graphs
  EXPECT(assert_equal(graph1, graph2));
}

/***************************************************************************
 * Test that the resize method works correctly for a HybridNonlinearFactorGraph.
 */
TEST(HybridNonlinearFactorGraph, Resize) {
  HybridNonlinearFactorGraph fg;
  auto nonlinearFactor = std::make_shared<BetweenFactor<double>>();
  fg.push_back(nonlinearFactor);

  auto discreteFactor = std::make_shared<DecisionTreeFactor>();
  fg.push_back(discreteFactor);

  auto dcFactor = std::make_shared<HybridNonlinearFactor>();
  fg.push_back(dcFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 3);

  fg.resize(0);
  EXPECT_LONGS_EQUAL(fg.size(), 0);
}

/***************************************************************************/
namespace test_motion {
gtsam::DiscreteKey m1(M(1), 2);
auto noise_model = noiseModel::Isotropic::Sigma(1, 1.0);
std::vector<NoiseModelFactor::shared_ptr> components = {
    std::make_shared<MotionModel>(X(0), X(1), 0.0, noise_model),
    std::make_shared<MotionModel>(X(0), X(1), 1.0, noise_model)};
}  // namespace test_motion

/***************************************************************************
 * Test that the resize method works correctly for a
 * HybridGaussianFactorGraph.
 */
TEST(HybridGaussianFactorGraph, Resize) {
  using namespace test_motion;

  HybridNonlinearFactorGraph hnfg;
  auto nonlinearFactor = std::make_shared<BetweenFactor<double>>(
      X(0), X(1), 0.0, Isotropic::Sigma(1, 0.1));
  hnfg.push_back(nonlinearFactor);
  auto discreteFactor = std::make_shared<DecisionTreeFactor>();
  hnfg.push_back(discreteFactor);

  auto dcFactor = std::make_shared<HybridNonlinearFactor>(m1, components);
  hnfg.push_back(dcFactor);

  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);
  linearizationPoint.insert<double>(X(1), 1);

  // Generate `HybridGaussianFactorGraph` by linearizing
  HybridGaussianFactorGraph gfg = *hnfg.linearize(linearizationPoint);

  EXPECT_LONGS_EQUAL(gfg.size(), 3);

  gfg.resize(0);
  EXPECT_LONGS_EQUAL(gfg.size(), 0);
}

/*****************************************************************************
 * Test push_back on HFG makes the correct distinction.
 */
TEST(HybridNonlinearFactorGraph, PushBack) {
  HybridNonlinearFactorGraph fg;

  auto nonlinearFactor = std::make_shared<BetweenFactor<double>>();
  fg.push_back(nonlinearFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  fg = HybridNonlinearFactorGraph();

  auto discreteFactor = std::make_shared<DecisionTreeFactor>();
  fg.push_back(discreteFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  fg = HybridNonlinearFactorGraph();

  auto dcFactor = std::make_shared<HybridNonlinearFactor>();
  fg.push_back(dcFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  // Now do the same with HybridGaussianFactorGraph
  HybridGaussianFactorGraph ghfg;

  auto gaussianFactor = std::make_shared<JacobianFactor>();
  ghfg.push_back(gaussianFactor);

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  ghfg = HybridGaussianFactorGraph();
  ghfg.push_back(discreteFactor);

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  ghfg = HybridGaussianFactorGraph();
  ghfg.push_back(dcFactor);

  HybridGaussianFactorGraph hgfg2;
  hgfg2.push_back(ghfg.begin(), ghfg.end());

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  HybridNonlinearFactorGraph hnfg;
  NonlinearFactorGraph factors;
  auto noise = noiseModel::Isotropic::Sigma(3, 1.0);
  factors.emplace_shared<PriorFactor<Pose2>>(0, Pose2(0, 0, 0), noise);
  factors.emplace_shared<PriorFactor<Pose2>>(1, Pose2(1, 0, 0), noise);
  factors.emplace_shared<PriorFactor<Pose2>>(2, Pose2(2, 0, 0), noise);
  // TODO(Varun) This does not currently work. It should work once HybridFactor
  // becomes a base class of NoiseModelFactor.
  // hnfg.push_back(factors.begin(), factors.end());

  // EXPECT_LONGS_EQUAL(3, hnfg.size());
}

/* ****************************************************************************/
// Test hybrid nonlinear factor graph errorTree
TEST(HybridNonlinearFactorGraph, ErrorTree) {
  Switching s(3);

  const HybridNonlinearFactorGraph &graph = s.nonlinearFactorGraph();
  const Values &values = s.linearizationPoint;

  auto error_tree = graph.errorTree(s.linearizationPoint);

  auto dkeys = graph.discreteKeys();
  DiscreteKeys discrete_keys(dkeys.begin(), dkeys.end());

  // Compute the sum of errors for each factor.
  auto assignments = DiscreteValues::CartesianProduct(discrete_keys);
  std::vector<double> leaves(assignments.size());
  for (auto &&factor : graph) {
    for (size_t i = 0; i < assignments.size(); ++i) {
      leaves[i] +=
          factor->error(HybridValues(VectorValues(), assignments[i], values));
    }
  }
  // Swap i=1 and i=2 to give correct ordering.
  double temp = leaves[1];
  leaves[1] = leaves[2];
  leaves[2] = temp;
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, leaves);

  EXPECT(assert_equal(expected_error, error_tree, 1e-7));
}

/****************************************************************************
 * Test construction of switching-like hybrid factor graph.
 */
TEST(HybridNonlinearFactorGraph, Switching) {
  Switching self(3);

  EXPECT_LONGS_EQUAL(7, self.nonlinearFactorGraph().size());
  EXPECT_LONGS_EQUAL(7, self.linearizedFactorGraph().size());
}

/****************************************************************************
 * Test linearization on a switching-like hybrid factor graph.
 */
TEST(HybridNonlinearFactorGraph, Linearization) {
  Switching self(3);

  // Linearize here:
  HybridGaussianFactorGraph actualLinearized =
      *self.nonlinearFactorGraph().linearize(self.linearizationPoint);

  EXPECT_LONGS_EQUAL(7, actualLinearized.size());
}

/****************************************************************************
 * Test elimination tree construction
 */
TEST(HybridNonlinearFactorGraph, EliminationTree) {
  Switching self(3);

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering.push_back(X(k));

  // Create elimination tree.
  HybridEliminationTree etree(self.linearizedFactorGraph(), ordering);
  EXPECT_LONGS_EQUAL(1, etree.roots().size())
}

/****************************************************************************
 *Test elimination function by eliminating x0 in *-x0-*-x1 graph.
 */
TEST(GaussianElimination, Eliminate_x0) {
  Switching self(3);

  // Gather factors on x0, has a simple Gaussian and a hybrid factor.
  HybridGaussianFactorGraph factors;
  // Add gaussian prior
  factors.push_back(self.linearUnaryFactors[0]);
  // Add first hybrid factor
  factors.push_back(self.linearBinaryFactors[0]);

  // Eliminate x0
  const Ordering ordering{X(0)};

  auto result = EliminateHybrid(factors, ordering);
  CHECK(result.first);
  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);
  // Has two keys, x2 and m1
  EXPECT_LONGS_EQUAL(2, result.second->size());
}

/****************************************************************************
 * Test elimination function by eliminating x1 in x0-*-x1-*-x2 chain.
 *                                                m0/      \m1
 */
TEST(HybridsGaussianElimination, Eliminate_x1) {
  Switching self(3);

  // Gather factors on x1, will be two hybrid factors (with x0 and x2, resp.).
  HybridGaussianFactorGraph factors;
  factors.push_back(self.linearBinaryFactors[0]);  // involves m0
  factors.push_back(self.linearBinaryFactors[1]);  // involves m1

  // Eliminate x1
  const Ordering ordering{X(1)};

  auto result = EliminateHybrid(factors, ordering);
  CHECK(result.first);
  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);
  // Note: separator keys should include m1, m2.
  EXPECT_LONGS_EQUAL(4, result.second->size());
}

/****************************************************************************
 * Helper method to generate gaussian factor graphs with a specific mode.
 */
GaussianFactorGraph::shared_ptr batchGFG(double between,
                                         Values linearizationPoint) {
  NonlinearFactorGraph graph;
  graph.addPrior<double>(X(0), 0, Isotropic::Sigma(1, 0.1));

  auto between_x0_x1 = std::make_shared<MotionModel>(X(0), X(1), between,
                                                     Isotropic::Sigma(1, 1.0));

  graph.push_back(between_x0_x1);

  return graph.linearize(linearizationPoint);
}

/****************************************************************************
 * Test elimination function by eliminating x0 and x1 in graph.
 */
TEST(HybridGaussianElimination, EliminateHybrid_2_Variable) {
  Switching self(2, 1.0, 0.1);

  auto factors = self.linearizedFactorGraph();

  // Eliminate x0
  const Ordering ordering{X(0), X(1)};

  const auto [hybridConditional, factorOnModes] =
      EliminateHybrid(factors, ordering);

  auto hybridGaussianConditional =
      dynamic_pointer_cast<HybridGaussianConditional>(
          hybridConditional->inner());

  CHECK(hybridGaussianConditional);
  // Frontals = [x0, x1]
  EXPECT_LONGS_EQUAL(2, hybridGaussianConditional->nrFrontals());
  // 1 parent, which is the mode
  EXPECT_LONGS_EQUAL(1, hybridGaussianConditional->nrParents());

  // This is now a discreteFactor
  auto discreteFactor = dynamic_pointer_cast<TableFactor>(factorOnModes);
  CHECK(discreteFactor);
  EXPECT_LONGS_EQUAL(1, discreteFactor->discreteKeys().size());
}

/****************************************************************************
 * Test partial elimination
 */
TEST(HybridNonlinearFactorGraph, Partial_Elimination) {
  Switching self(3);

  // Create ordering of only continuous variables.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering.push_back(X(k));

  // Eliminate partially i.e. only continuous part.
  const auto [hybridBayesNet, remainingFactorGraph] =
      self.linearizedFactorGraph().eliminatePartialSequential(ordering);

  CHECK(hybridBayesNet);
  EXPECT_LONGS_EQUAL(3, hybridBayesNet->size());
  EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(0)});
  EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(1), M(0)}));
  EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({X(2), M(0), M(1)}));
  EXPECT(hybridBayesNet->at(2)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet->at(2)->parents() == KeyVector({M(0), M(1)}));

  CHECK(remainingFactorGraph);
  EXPECT_LONGS_EQUAL(3, remainingFactorGraph->size());
  EXPECT(remainingFactorGraph->at(0)->keys() == KeyVector({M(0)}));
  EXPECT(remainingFactorGraph->at(1)->keys() == KeyVector({M(1), M(0)}));
  EXPECT(remainingFactorGraph->at(2)->keys() == KeyVector({M(0), M(1)}));
}

/* ****************************************************************************/
TEST(HybridNonlinearFactorGraph, Error) {
  Switching self(3);
  HybridNonlinearFactorGraph fg = self.nonlinearFactorGraph();

  {
    HybridValues values(VectorValues(), DiscreteValues{{M(0), 0}, {M(1), 0}},
                        self.linearizationPoint);
    // regression
    EXPECT_DOUBLES_EQUAL(152.791759469, fg.error(values), 1e-9);
  }
  {
    HybridValues values(VectorValues(), DiscreteValues{{M(0), 0}, {M(1), 1}},
                        self.linearizationPoint);
    // regression
    EXPECT_DOUBLES_EQUAL(151.598612289, fg.error(values), 1e-9);
  }
  {
    HybridValues values(VectorValues(), DiscreteValues{{M(0), 1}, {M(1), 0}},
                        self.linearizationPoint);
    // regression
    EXPECT_DOUBLES_EQUAL(151.703972804, fg.error(values), 1e-9);
  }
  {
    HybridValues values(VectorValues(), DiscreteValues{{M(0), 1}, {M(1), 1}},
                        self.linearizationPoint);
    // regression
    EXPECT_DOUBLES_EQUAL(151.609437912, fg.error(values), 1e-9);
  }
}

/* ****************************************************************************/
TEST(HybridNonlinearFactorGraph, PrintErrors) {
  Switching self(3);

  // Get nonlinear factor graph and add linear factors to be holistic.
  // TODO(Frank): ???
  HybridNonlinearFactorGraph fg = self.nonlinearFactorGraph();
  fg.add(self.linearizedFactorGraph());

  // Optimize to get HybridValues
  HybridBayesNet::shared_ptr bn =
      self.linearizedFactorGraph().eliminateSequential();
  HybridValues hv = bn->optimize();

  // Print and verify
  // fg.print();
  // std::cout << "\n\n\n" << std::endl;
  // fg.printErrors(
  //     HybridValues(hv.continuous(), DiscreteValues(),
  //     self.linearizationPoint));
}

/****************************************************************************
 * Test full elimination
 */
TEST(HybridNonlinearFactorGraph, Full_Elimination) {
  Switching self(3);

  // We first do a partial elimination
  DiscreteBayesNet discreteBayesNet;

  {
    // Create ordering.
    Ordering ordering;
    for (size_t k = 0; k < self.K; k++) ordering.push_back(X(k));

    // Eliminate partially.
    const auto [hybridBayesNet_partial, remainingFactorGraph_partial] =
        self.linearizedFactorGraph().eliminatePartialSequential(ordering);

    DiscreteFactorGraph discrete_fg =
        remainingFactorGraph_partial->discreteFactors();

    ordering.clear();
    for (size_t k = 0; k < self.K - 1; k++) ordering.push_back(M(k));
    discreteBayesNet =
        *discrete_fg.eliminateSequential(ordering, EliminateDiscrete);
  }

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering.push_back(X(k));
  for (size_t k = 0; k < self.K - 1; k++) ordering.push_back(M(k));

  // Eliminate partially.
  HybridBayesNet::shared_ptr hybridBayesNet =
      self.linearizedFactorGraph().eliminateSequential(ordering);

  CHECK(hybridBayesNet);
  EXPECT_LONGS_EQUAL(5, hybridBayesNet->size());
  // p(x0 | x1, m0)
  EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(0)});
  EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(1), M(0)}));
  // p(x1 | x2, m0, m1)
  EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({X(2), M(0), M(1)}));
  // p(x2 | m0, m1)
  EXPECT(hybridBayesNet->at(2)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet->at(2)->parents() == KeyVector({M(0), M(1)}));
  // P(m0 | m1)
  EXPECT(hybridBayesNet->at(3)->frontals() == KeyVector{M(0)});
  EXPECT(hybridBayesNet->at(3)->parents() == KeyVector({M(1)}));
  EXPECT(
      dynamic_pointer_cast<DiscreteConditional>(hybridBayesNet->at(3)->inner())
          ->equals(*discreteBayesNet.at(0)));
  // P(m1)
  EXPECT(hybridBayesNet->at(4)->frontals() == KeyVector{M(1)});
  EXPECT_LONGS_EQUAL(0, hybridBayesNet->at(4)->nrParents());
  TableDistribution dtc =
      *hybridBayesNet->at(4)->asDiscrete<TableDistribution>();
  EXPECT(DiscreteConditional(dtc.nrFrontals(), dtc.toDecisionTreeFactor())
             .equals(*discreteBayesNet.at(1)));
}

/****************************************************************************
 * Test printing
 */
TEST(HybridNonlinearFactorGraph, Printing) {
  Switching self(3);

  auto linearizedFactorGraph = self.linearizedFactorGraph();

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering.push_back(X(k));

  // Eliminate partially.
  const auto [hybridBayesNet, remainingFactorGraph] =
      linearizedFactorGraph.eliminatePartialSequential(ordering);

  // Set precision so we are consistent on all platforms
  std::cout << std::setprecision(6);

#ifdef GTSAM_DT_MERGING
  string expected_hybridFactorGraph = R"(
size: 7
Factor 0
GaussianFactor:

  A[x0] = [
	10
]
  b = [ -10 ]
  No noise model

Factor 1
GaussianFactor:

  A[x1] = [
	10
]
  b = [ -10 ]
  No noise model

Factor 2
GaussianFactor:

  A[x2] = [
	10
]
  b = [ -10 ]
  No noise model

Factor 3
HybridGaussianFactor:
Hybrid [x0 x1; m0]{
 Choice(m0) 
 0 Leaf :
  A[x0] = [
	-1
]
  A[x1] = [
	1
]
  b = [ -1 ]
  No noise model
scalar: 0.918939

 1 Leaf :
  A[x0] = [
	-1
]
  A[x1] = [
	1
]
  b = [ -0 ]
  No noise model
scalar: 0.918939

}

Factor 4
HybridGaussianFactor:
Hybrid [x1 x2; m1]{
 Choice(m1) 
 0 Leaf :
  A[x1] = [
	-1
]
  A[x2] = [
	1
]
  b = [ -1 ]
  No noise model
scalar: 0.918939

 1 Leaf :
  A[x1] = [
	-1
]
  A[x2] = [
	1
]
  b = [ -0 ]
  No noise model
scalar: 0.918939

}

Factor 5
DiscreteFactor:
 P( m0 ):
 Leaf  0.5


Factor 6
DiscreteFactor:
 P( m1 | m0 ):
 Choice(m1) 
 0 Choice(m0) 
 0 0 Leaf 0.33333333
 0 1 Leaf  0.6
 1 Choice(m0) 
 1 0 Leaf 0.66666667
 1 1 Leaf  0.4


)";
#else
  string expected_hybridFactorGraph = R"(
size: 7
Factor 0
GaussianFactor:

  A[x0] = [
	10
]
  b = [ -10 ]
  No noise model

Factor 1
GaussianFactor:

  A[x1] = [
	10
]
  b = [ -10 ]
  No noise model

Factor 2
GaussianFactor:

  A[x2] = [
	10
]
  b = [ -10 ]
  No noise model

Factor 3
HybridGaussianFactor:
Hybrid [x0 x1; m0]{
 Choice(m0) 
 0 Leaf :
  A[x0] = [
	-1
]
  A[x1] = [
	1
]
  b = [ -1 ]
  No noise model
scalar: 0.918939

 1 Leaf :
  A[x0] = [
	-1
]
  A[x1] = [
	1
]
  b = [ -0 ]
  No noise model
scalar: 0.918939

}

Factor 4
HybridGaussianFactor:
Hybrid [x1 x2; m1]{
 Choice(m1) 
 0 Leaf :
  A[x1] = [
	-1
]
  A[x2] = [
	1
]
  b = [ -1 ]
  No noise model
scalar: 0.918939

 1 Leaf :
  A[x1] = [
	-1
]
  A[x2] = [
	1
]
  b = [ -0 ]
  No noise model
scalar: 0.918939

}

Factor 5
DiscreteFactor:
 P( m0 ):
 Choice(m0) 
 0 Leaf  0.5
 1 Leaf  0.5


Factor 6
DiscreteFactor:
 P( m1 | m0 ):
 Choice(m1) 
 0 Choice(m0) 
 0 0 Leaf 0.33333333
 0 1 Leaf  0.6
 1 Choice(m0) 
 1 0 Leaf 0.66666667
 1 1 Leaf  0.4


)";
#endif

  EXPECT(assert_print_equal(expected_hybridFactorGraph, linearizedFactorGraph));

  // Expected output for hybridBayesNet.
  string expected_hybridBayesNet = R"(
size: 3
conditional 0:  P( x0 | x1 m0)
 Discrete Keys = (m0, 2), 
 logNormalizationConstant: 1.38862

 Choice(m0) 
 0 Leaf p(x0 | x1)
  R = [ 10.0499 ]
  S[x1] = [ -0.0995037 ]
  d = [ -9.85087 ]
  logNormalizationConstant: 1.38862
  No noise model

 1 Leaf p(x0 | x1)
  R = [ 10.0499 ]
  S[x1] = [ -0.0995037 ]
  d = [ -9.95037 ]
  logNormalizationConstant: 1.38862
  No noise model

conditional 1:  P( x1 | x2 m0 m1)
 Discrete Keys = (m0, 2), (m1, 2), 
 logNormalizationConstant: 1.3935

 Choice(m1) 
 0 Choice(m0) 
 0 0 Leaf p(x1 | x2)
  R = [ 10.099 ]
  S[x2] = [ -0.0990196 ]
  d = [ -9.99901 ]
  logNormalizationConstant: 1.3935
  No noise model

 0 1 Leaf p(x1 | x2)
  R = [ 10.099 ]
  S[x2] = [ -0.0990196 ]
  d = [ -9.90098 ]
  logNormalizationConstant: 1.3935
  No noise model

 1 Choice(m0) 
 1 0 Leaf p(x1 | x2)
  R = [ 10.099 ]
  S[x2] = [ -0.0990196 ]
  d = [ -10.098 ]
  logNormalizationConstant: 1.3935
  No noise model

 1 1 Leaf p(x1 | x2)
  R = [ 10.099 ]
  S[x2] = [ -0.0990196 ]
  d = [ -10 ]
  logNormalizationConstant: 1.3935
  No noise model

conditional 2:  P( x2 | m0 m1)
 Discrete Keys = (m0, 2), (m1, 2), 
 logNormalizationConstant: 1.38857

 Choice(m1) 
 0 Choice(m0) 
 0 0 Leaf p(x2)
  R = [ 10.0494 ]
  d = [ -10.1489 ]
  mean: 1 elements
  x2: -1.0099
  logNormalizationConstant: 1.38857
  No noise model

 0 1 Leaf p(x2)
  R = [ 10.0494 ]
  d = [ -10.1479 ]
  mean: 1 elements
  x2: -1.0098
  logNormalizationConstant: 1.38857
  No noise model

 1 Choice(m0) 
 1 0 Leaf p(x2)
  R = [ 10.0494 ]
  d = [ -10.0504 ]
  mean: 1 elements
  x2: -1.0001
  logNormalizationConstant: 1.38857
  No noise model

 1 1 Leaf p(x2)
  R = [ 10.0494 ]
  d = [ -10.0494 ]
  mean: 1 elements
  x2: -1
  logNormalizationConstant: 1.38857
  No noise model

)";
  EXPECT(assert_print_equal(expected_hybridBayesNet, *hybridBayesNet));
}

/****************************************************************************
 * Simple PlanarSLAM example test with 2 poses and 2 landmarks (each pose
 * connects to 1 landmark) to expose issue with default decision tree creation
 * in hybrid elimination. The hybrid factor is between the poses X0 and X1.
 * The issue arises if we eliminate a landmark variable first since it is not
 * connected to a HybridFactor.
 */
TEST(HybridNonlinearFactorGraph, DefaultDecisionTree) {
  HybridNonlinearFactorGraph fg;

  // Add a prior on pose x0 at the origin.
  // A prior factor consists of a mean and a noise model (covariance matrix)
  Pose2 prior(0.0, 0.0, 0.0);  // prior mean is at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector3(0.3, 0.3, 0.1));  // 30cm std on x,y, 0.1 rad on theta
  fg.emplace_shared<PriorFactor<Pose2>>(X(0), prior, priorNoise);

  using PlanarMotionModel = BetweenFactor<Pose2>;

  // Add odometry factor
  Pose2 odometry(2.0, 0.0, 0.0);
  auto noise_model = noiseModel::Isotropic::Sigma(3, 1.0);
  std::vector<NoiseModelFactor::shared_ptr> motion_models = {
      std::make_shared<PlanarMotionModel>(X(0), X(1), Pose2(0, 0, 0),
                                          noise_model),
      std::make_shared<PlanarMotionModel>(X(0), X(1), odometry, noise_model)};
  fg.emplace_shared<HybridNonlinearFactor>(DiscreteKey{M(1), 2}, motion_models);

  // Add Range-Bearing measurements to from X0 to L0 and X1 to L1.
  // create a noise model for the landmark measurements
  auto measurementNoise = noiseModel::Diagonal::Sigmas(
      Vector2(0.1, 0.2));  // 0.1 rad std on bearing, 20cm on range
  // create the measurement values - indices are (pose id, landmark id)
  Rot2 bearing11 = Rot2::fromDegrees(45), bearing22 = Rot2::fromDegrees(90);
  double range11 = std::sqrt(4.0 + 4.0), range22 = 2.0;

  // Add Bearing-Range factors
  fg.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
      X(0), L(0), bearing11, range11, measurementNoise);
  fg.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
      X(1), L(1), bearing22, range22, measurementNoise);

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  initialEstimate.insert(X(0), Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(X(1), Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(L(0), Point2(1.8, 2.1));
  initialEstimate.insert(L(1), Point2(4.1, 1.8));

  // We want to eliminate variables not connected to DCFactors first.
  const Ordering ordering{L(0), L(1), X(0), X(1)};

  HybridGaussianFactorGraph linearized = *fg.linearize(initialEstimate);

  // This should NOT fail
  const auto [hybridBayesNet, remainingFactorGraph] =
      linearized.eliminatePartialSequential(ordering);
  EXPECT_LONGS_EQUAL(4, hybridBayesNet->size());
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph->size());
}

namespace test_relinearization {
/**
 * @brief Create a Factor Graph by directly specifying all
 * the factors instead of creating conditionals first.
 * This way we can directly provide the likelihoods and
 * then perform (re-)linearization.
 *
 * @param means The means of the HybridGaussianFactor components.
 * @param sigmas The covariances of the HybridGaussianFactor components.
 * @param m1 The discrete key.
 * @param x0_measurement A measurement on X0
 * @return HybridGaussianFactorGraph
 */
static HybridNonlinearFactorGraph CreateFactorGraph(
    const std::vector<double> &means, const std::vector<double> &sigmas,
    DiscreteKey &m1, double x0_measurement, double measurement_noise = 1e-3) {
  auto model0 = noiseModel::Isotropic::Sigma(1, sigmas[0]);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigmas[1]);
  auto prior_noise = noiseModel::Isotropic::Sigma(1, measurement_noise);

  auto f0 =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), means[0], model0);
  auto f1 =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), means[1], model1);

  // Create HybridNonlinearFactor
  // We take negative since we want
  // the underlying scalar to be log(\sqrt(|2πΣ|))
  std::vector<NonlinearFactorValuePair> factors{{f0, 0.0}, {f1, 0.0}};

  HybridNonlinearFactor mixtureFactor(m1, factors);

  HybridNonlinearFactorGraph hfg;
  hfg.push_back(mixtureFactor);

  hfg.push_back(PriorFactor<double>(X(0), x0_measurement, prior_noise));

  return hfg;
}
}  // namespace test_relinearization

/* ************************************************************************* */
/**
 * @brief Test components with differing means but the same covariances.
 * The factor graph is
 *     *-X1-*-X2
 *          |
 *          M1
 */
TEST(HybridNonlinearFactorGraph, DifferentMeans) {
  using namespace test_relinearization;

  DiscreteKey m1(M(1), 2);

  Values values;
  double x0 = 0.0, x1 = 1.75;
  values.insert(X(0), x0);
  values.insert(X(1), x1);

  std::vector<double> means = {0.0, 2.0}, sigmas = {1e-0, 1e-0};

  HybridNonlinearFactorGraph hfg = CreateFactorGraph(means, sigmas, m1, x0);

  {
    auto bn = hfg.linearize(values)->eliminateSequential();
    HybridValues actual = bn->optimize();

    HybridValues expected(
        VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(-1.75)}},
        DiscreteValues{{M(1), 0}});

    EXPECT(assert_equal(expected, actual));

    DiscreteValues dv0{{M(1), 0}};
    VectorValues cont0 = bn->optimize(dv0);
    double error0 = bn->error(HybridValues(cont0, dv0));

    // regression
    EXPECT_DOUBLES_EQUAL(0.69314718056, error0, 1e-9);

    DiscreteValues dv1{{M(1), 1}};
    VectorValues cont1 = bn->optimize(dv1);
    double error1 = bn->error(HybridValues(cont1, dv1));
    EXPECT_DOUBLES_EQUAL(error0, error1, 1e-9);
  }

  {
    // Add measurement on x1
    auto prior_noise = noiseModel::Isotropic::Sigma(1, 1e-3);
    hfg.push_back(PriorFactor<double>(X(1), means[1], prior_noise));

    auto bn = hfg.linearize(values)->eliminateSequential();
    HybridValues actual = bn->optimize();

    HybridValues expected(
        VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(0.25)}},
        DiscreteValues{{M(1), 1}});

    EXPECT(assert_equal(expected, actual));

    {
      DiscreteValues dv{{M(1), 0}};
      VectorValues cont = bn->optimize(dv);
      double error = bn->error(HybridValues(cont, dv));
      // regression
      EXPECT_DOUBLES_EQUAL(2.12692448787, error, 1e-9);
    }
    {
      DiscreteValues dv{{M(1), 1}};
      VectorValues cont = bn->optimize(dv);
      double error = bn->error(HybridValues(cont, dv));
      // regression
      EXPECT_DOUBLES_EQUAL(0.126928487854, error, 1e-9);
    }
  }
}

/* ************************************************************************* */
/**
 * @brief Test components with differing covariances but the same means.
 * The factor graph is
 *     *-X1-*-X2
 *          |
 *          M1
 */
TEST(HybridNonlinearFactorGraph, DifferentCovariances) {
  using namespace test_relinearization;

  DiscreteKey m1(M(1), 2);

  Values values;
  double x0 = 1.0, x1 = 1.0;
  values.insert(X(0), x0);
  values.insert(X(1), x1);

  std::vector<double> means = {0.0, 0.0}, sigmas = {1e2, 1e-2};

  // Create FG with HybridNonlinearFactor and prior on X1
  HybridNonlinearFactorGraph hfg = CreateFactorGraph(means, sigmas, m1, x0);
  // Linearize
  auto hgfg = hfg.linearize(values);
  // and eliminate
  auto hbn = hgfg->eliminateSequential();

  VectorValues cv;
  cv.insert(X(0), Vector1(0.0));
  cv.insert(X(1), Vector1(0.0));

  DiscreteValues dv0{{M(1), 0}};
  DiscreteValues dv1{{M(1), 1}};

  TableDistribution expected_m1(m1, "0.5 0.5");
  TableDistribution actual_m1 = *(hbn->at(2)->asDiscrete<TableDistribution>());

  EXPECT(assert_equal(expected_m1, actual_m1));
}

TEST(HybridNonlinearFactorGraph, Relinearization) {
  using namespace test_relinearization;

  DiscreteKey m1(M(1), 2);

  Values values;
  double x0 = 0.0, x1 = 0.8;
  values.insert(X(0), x0);
  values.insert(X(1), x1);

  std::vector<double> means = {0.0, 1.0}, sigmas = {1e-2, 1e-2};

  double prior_sigma = 1e-2;
  // Create FG with HybridNonlinearFactor and prior on X1
  HybridNonlinearFactorGraph hfg =
      CreateFactorGraph(means, sigmas, m1, 0.0, prior_sigma);
  hfg.push_back(PriorFactor<double>(
      X(1), 1.2, noiseModel::Isotropic::Sigma(1, prior_sigma)));

  // Linearize
  auto hgfg = hfg.linearize(values);
  // and eliminate
  auto hbn = hgfg->eliminateSequential();

  HybridValues delta = hbn->optimize();
  values = values.retract(delta.continuous());

  Values expected_first_result;
  expected_first_result.insert(X(0), 0.0666666666667);
  expected_first_result.insert(X(1), 1.13333333333);
  EXPECT(assert_equal(expected_first_result, values));

  // Re-linearize
  hgfg = hfg.linearize(values);
  // and eliminate
  hbn = hgfg->eliminateSequential();
  delta = hbn->optimize();
  HybridValues result(delta.continuous(), delta.discrete(),
                      values.retract(delta.continuous()));

  HybridValues expected_result(
      VectorValues{{X(0), Vector1(0)}, {X(1), Vector1(0)}},
      DiscreteValues{{M(1), 1}}, expected_first_result);
  EXPECT(assert_equal(expected_result, result));
}

/* *************************************************************************
 */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* *************************************************************************
 */
