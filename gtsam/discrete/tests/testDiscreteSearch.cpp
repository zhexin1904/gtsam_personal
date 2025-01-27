/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteSearch.cpp
 *
 *  @date January, 2025
 *  @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteSearch.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "AsiaExample.h"

using namespace gtsam;

/* ************************************************************************* */
TEST(DiscreteBayesNet, EmptyKBest) {
  DiscreteBayesNet net;  // no factors
  DiscreteSearch search(net, 3);
  auto solutions = search.run();
  // Expect one solution with empty assignment, error=0
  EXPECT_LONGS_EQUAL(1, solutions.size());
  EXPECT_DOUBLES_EQUAL(0, std::fabs(solutions[0].error), 1e-9);
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, AsiaKBest) {
  using namespace asia_example;
  DiscreteBayesNet asia = createAsiaExample();

  // Ask for the MPE
  DiscreteSearch search1(asia);
  auto mpe = search1.run();

  // print numExpansions
  std::cout << "Number of expansions: " << search1.numExpansions << std::endl;

  EXPECT_LONGS_EQUAL(1, mpe.size());
  // Regression test: check the MPE solution
  EXPECT_DOUBLES_EQUAL(1.236627, std::fabs(mpe[0].error), 1e-5);

  DiscreteSearch search(asia, 4);
  auto solutions = search.run();

  // print numExpansions
  std::cout << "Number of expansions: " << search.numExpansions << std::endl;

  EXPECT_LONGS_EQUAL(4, solutions.size());
  // Regression test: check the first and last solution
  EXPECT_DOUBLES_EQUAL(1.236627, std::fabs(solutions[0].error), 1e-5);
  EXPECT_DOUBLES_EQUAL(2.201708, std::fabs(solutions[3].error), 1e-5);
}

/* ************************************************************************* */
TEST(DiscreteBayesTree, EmptyTree) {
  DiscreteBayesTree bt;

  DiscreteSearch search(bt, 3);
  auto solutions = search.run();

  // We expect exactly 1 solution with error = 0.0 (the empty assignment).
  assert(solutions.size() == 1 && "There should be exactly one empty solution");
  EXPECT_LONGS_EQUAL(1, solutions.size());
  EXPECT_DOUBLES_EQUAL(0, std::fabs(solutions[0].error), 1e-9);
}

/* ************************************************************************* */
TEST(DiscreteBayesTree, AsiaTreeKBest) {
  using namespace asia_example;
  DiscreteFactorGraph asia(createAsiaExample());
  const Ordering ordering{D, X, B, E, L, T, S, A};
  DiscreteBayesTree::shared_ptr bt = asia.eliminateMultifrontal(ordering);

  // Ask for top 4 solutions
  DiscreteSearch search1(*bt);
  auto mpe = search1.run();

  // print numExpansions
  std::cout << "Number of expansions: " << search1.numExpansions << std::endl;

  EXPECT_LONGS_EQUAL(1, mpe.size());
  // Regression test: check the MPE solution
  EXPECT_DOUBLES_EQUAL(1.236627, std::fabs(mpe[0].error), 1e-5);

  // Ask for top 4 solutions
  DiscreteSearch search(*bt, 4);
  auto solutions = search.run();

  // print numExpansions
  std::cout << "Number of expansions: " << search.numExpansions << std::endl;

  EXPECT_LONGS_EQUAL(4, solutions.size());
  // Regression test: check the first and last solution
  EXPECT_DOUBLES_EQUAL(1.236627, std::fabs(solutions[0].error), 1e-5);
  EXPECT_DOUBLES_EQUAL(2.201708, std::fabs(solutions[3].error), 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
