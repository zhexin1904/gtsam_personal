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

TEST(DiscreteBayesNet, EmptyKBest) {
  DiscreteBayesNet net;  // no factors
  DiscreteSearch search(net, 3);
  auto solutions = search.run();
  // Expect one solution with empty assignment, error=0
  EXPECT_LONGS_EQUAL(1, solutions.size());
  EXPECT_DOUBLES_EQUAL(0, std::fabs(solutions[0].error), 1e-9);
}

TEST(DiscreteBayesNet, AsiaKBest) {
  using namespace asia_example;
  DiscreteBayesNet asia = createAsiaExample();
  DiscreteSearch search(asia, 4);
  auto solutions = search.run();
  EXPECT(!solutions.empty());
  // Regression test: check the first solution
  EXPECT_DOUBLES_EQUAL(1.236627, std::fabs(solutions[0].error), 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
