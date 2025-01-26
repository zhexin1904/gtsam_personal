/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteBayesNet.cpp
 *
 *  @date Feb 27, 2011
 *  @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/debug.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/inference/Symbol.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;

namespace keys {
static const Key D = Symbol('D', 1), X = Symbol('X', 2), E = Symbol('E', 3),
                 B = Symbol('B', 4), L = Symbol('L', 5), T = Symbol('T', 6),
                 S = Symbol('S', 7), A = Symbol('A', 8);
}

static const DiscreteKey Dyspnea(keys::D, 2), XRay(keys::X, 2),
    Either(keys::E, 2), Bronchitis(keys::B, 2), LungCancer(keys::L, 2),
    Tuberculosis(keys::T, 2), Smoking(keys::S, 2), Asia(keys::A, 2);

using ADT = AlgebraicDecisionTree<Key>;

// Function to construct the Asia example
DiscreteBayesNet constructAsiaExample() {
  DiscreteBayesNet asia;

  // Add in topological sort order, parents last:
  asia.add((Dyspnea | Either, Bronchitis) = "9/1 2/8 3/7 1/9");
  asia.add(XRay | Either = "95/5 2/98");
  asia.add((Either | Tuberculosis, LungCancer) = "F T T T");
  asia.add(Bronchitis | Smoking = "70/30 40/60");
  asia.add(LungCancer | Smoking = "99/1 90/10");
  asia.add(Tuberculosis | Asia = "99/1 95/5");
  asia.add(Smoking % "50/50");  // Signature version
  asia.add(Asia, "99/1");

  return asia;
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, bayesNet) {
  DiscreteBayesNet bayesNet;
  DiscreteKey Parent(0, 2), Child(1, 2);

  auto prior = std::make_shared<DiscreteConditional>(Parent % "6/4");
  CHECK(assert_equal(ADT({Parent}, "0.6 0.4"), (ADT)*prior));
  bayesNet.push_back(prior);

  auto conditional =
      std::make_shared<DiscreteConditional>(Child | Parent = "7/3 8/2");
  EXPECT_LONGS_EQUAL(1, *(conditional->beginFrontals()));
  ADT expected(Child & Parent, "0.7 0.8 0.3 0.2");
  CHECK(assert_equal(expected, (ADT)*conditional));
  bayesNet.push_back(conditional);

  DiscreteFactorGraph fg(bayesNet);
  LONGS_EQUAL(2, fg.back()->size());

  // Check the marginals
  const double expectedMarginal[2]{0.4, 0.6 * 0.3 + 0.4 * 0.2};
  DiscreteMarginals marginals(fg);
  for (size_t j = 0; j < 2; j++) {
    Vector FT = marginals.marginalProbabilities(DiscreteKey(j, 2));
    EXPECT_DOUBLES_EQUAL(expectedMarginal[j], FT[1], 1e-3);
    EXPECT_DOUBLES_EQUAL(FT[0], 1.0 - FT[1], 1e-9);
  }
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, Asia) {
  DiscreteBayesNet asia = constructAsiaExample();

  // Convert to factor graph
  DiscreteFactorGraph fg(asia);
  LONGS_EQUAL(1, fg.back()->size());

  // Check the marginals we know (of the parent-less nodes)
  DiscreteMarginals marginals(fg);
  Vector2 va(0.99, 0.01), vs(0.5, 0.5);
  EXPECT(assert_equal(va, marginals.marginalProbabilities(Asia)));
  EXPECT(assert_equal(vs, marginals.marginalProbabilities(Smoking)));

  // Create solver and eliminate
  const Ordering ordering{keys::A, keys::D, keys::T, keys::X,
                          keys::S, keys::E, keys::L, keys::B};
  DiscreteBayesNet::shared_ptr chordal = fg.eliminateSequential(ordering);
  DiscreteConditional expected2(Bronchitis % "11/9");
  EXPECT(assert_equal(expected2, *chordal->back()));

  // Check evaluate and logProbability
  auto result = fg.optimize();
  EXPECT_DOUBLES_EQUAL(asia.logProbability(result),
                       std::log(asia.evaluate(result)), 1e-9);

  // add evidence, we were in Asia and we have dyspnea
  fg.add(Asia, "0 1");
  fg.add(Dyspnea, "0 1");

  // solve again, now with evidence
  DiscreteBayesNet::shared_ptr chordal2 = fg.eliminateSequential(ordering);
  EXPECT(assert_equal(expected2, *chordal->back()));

  // now sample from it
  DiscreteValues expectedSample{{Asia.first, 1},       {Dyspnea.first, 1},
                                {XRay.first, 1},       {Tuberculosis.first, 0},
                                {Smoking.first, 1},    {Either.first, 1},
                                {LungCancer.first, 1}, {Bronchitis.first, 0}};
  SETDEBUG("DiscreteConditional::sample", false);
  auto actualSample = chordal2->sample();
  EXPECT(assert_equal(expectedSample, actualSample));
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, Sugar) {
  DiscreteKey T(0, 2), L(1, 2), E(2, 2), C(8, 3), S(7, 2);

  DiscreteBayesNet bn;

  // try logic
  bn.add((E | T, L) = "OR");
  bn.add((E | T, L) = "AND");

  // try multivalued
  bn.add(C % "1/1/2");
  bn.add(C | S = "1/1/2 5/2/3");
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, Dot) {
  DiscreteBayesNet fragment;
  fragment.add(Asia % "99/1");
  fragment.add(Smoking % "50/50");

  fragment.add(Tuberculosis | Asia = "99/1 95/5");
  fragment.add(LungCancer | Smoking = "99/1 90/10");
  fragment.add((Either | Tuberculosis, LungCancer) = "F T T T");

  string actual = fragment.dot();
  EXPECT(actual ==
         "digraph {\n"
         "  size=\"5,5\";\n"
         "\n"
         "  var4683743612465315848[label=\"A8\"];\n"
         "  var4971973988617027587[label=\"E3\"];\n"
         "  var5476377146882523141[label=\"L5\"];\n"
         "  var5980780305148018695[label=\"S7\"];\n"
         "  var6052837899185946630[label=\"T6\"];\n"
         "\n"
         "  var6052837899185946630->var4971973988617027587\n"
         "  var5476377146882523141->var4971973988617027587\n"
         "  var5980780305148018695->var5476377146882523141\n"
         "  var4683743612465315848->var6052837899185946630\n"
         "}");
}

/* ************************************************************************* */
// Check markdown representation looks as expected.
TEST(DiscreteBayesNet, markdown) {
  DiscreteBayesNet fragment;
  fragment.add(Asia % "99/1");
  fragment.add(Smoking | Asia = "8/2 7/3");

  string expected =
      "`DiscreteBayesNet` of size 2\n"
      "\n"
      " *P(Asia):*\n\n"
      "|Asia|value|\n"
      "|:-:|:-:|\n"
      "|0|0.99|\n"
      "|1|0.01|\n"
      "\n"
      " *P(Smoking|Asia):*\n\n"
      "|*Asia*|0|1|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0.8|0.2|\n"
      "|1|0.7|0.3|\n\n";
  auto formatter = [](Key key) { return key == keys::A ? "Asia" : "Smoking"; };
  string actual = fragment.markdown(formatter);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

using Value = size_t;

// ----------------------------------------------------------------------------
// 1) SearchNode: store partial assignment and next factor to expand
// ----------------------------------------------------------------------------
struct SearchNode {
  DiscreteValues assignment;
  double error;
  double bound;
  int nextConditional;  // index into conditionals

  /// if nextConditional < 0, we've assigned everything.
  bool isComplete() const { return nextConditional < 0; }

  /// lower bound on final error for unassigned variables. Stub=0.
  double computeBound() const {
    // Real code might do partial factor analysis or heuristics.
    return 0.0;
  }

  /// Expand this node by assigning the next variable
  SearchNode expand(const DiscreteConditional& conditional,
                    const DiscreteValues& fa) const {
    // Combine the new frontal assignment with the current partial assignment
    SearchNode child;
    child.assignment = assignment;
    for (auto& kv : fa) {
      child.assignment[kv.first] = kv.second;
    }

    // Compute the incremental error for this factor
    child.error = error + conditional.error(child.assignment);

    // Compute new bound
    child.bound = child.error + computeBound();

    // Next factor index
    child.nextConditional = nextConditional - 1;

    return child;
  }

  friend std::ostream& operator<<(std::ostream& os, const SearchNode& sn) {
    os << "[ error=" << sn.error << " bound=" << sn.bound
       << " nextConditional=" << sn.nextConditional << " assignment={"
       << sn.assignment << "}]";
    return os;
  }
};

// ----------------------------------------------------------------------------
// 2) Priority functor to make a min-heap by bound
// ----------------------------------------------------------------------------
struct CompareByBound {
  bool operator()(const SearchNode& a, const SearchNode& b) const {
    return a.bound > b.bound;  // smallest bound -> highest priority
  }
};

// ----------------------------------------------------------------------------
// 4) A Solution
// ----------------------------------------------------------------------------
struct Solution {
  double error;
  DiscreteValues assignment;
  Solution(double err, const DiscreteValues& assign)
      : error(err), assignment(assign) {}
  friend std::ostream& operator<<(std::ostream& os, const Solution& sn) {
    os << "[ error=" << sn.error << " assignment={" << sn.assignment << "}]";
    return os;
  }
};

struct CompareByError {
  bool operator()(const Solution& a, const Solution& b) const {
    return a.error < b.error;
  }
};

// Define the Solutions class
class Solutions {
 private:
  size_t maxSize_;
  std::priority_queue<Solution, std::vector<Solution>, CompareByError> pq_;

 public:
  Solutions(size_t maxSize) : maxSize_(maxSize) {}

  /// Add a solution to the priority queue, possibly evicting the worst one.
  /// Return true if we added the solution.
  bool maybeAdd(double error, const DiscreteValues& assignment) {
    const bool full = pq_.size() == maxSize_;
    if (full && error >= pq_.top().error) return false;
    if (full) pq_.pop();
    pq_.emplace(error, assignment);
    return true;
  }

  /// Check if we have any solutions
  bool empty() const { return pq_.empty(); }

  // Method to print all solutions
  void print() const {
    auto pq = pq_;
    while (!pq.empty()) {
      const Solution& best = pq.top();
      std::cout << "Error: " << best.error << ", Values: " << best.assignment
                << std::endl;
      pq.pop();
    }
  }

  /// Check if (partial) solution with given bound can be pruned. If we have
  /// room, we never prune. Otherwise, prune if lower bound on error is worse
  /// than our current worst error.
  bool prune(double bound) const {
    if (pq_.size() < maxSize_) return false;
    double worstError = pq_.top().error;
    return (bound >= worstError);
  }

  // Method to extract solutions in ascending order of error
  std::vector<Solution> extractSolutions() {
    std::vector<Solution> result;
    while (!pq_.empty()) {
      result.push_back(pq_.top());
      pq_.pop();
    }
    std::sort(
        result.begin(), result.end(),
        [](const Solution& a, const Solution& b) { return a.error < b.error; });
    return result;
  }
};

// ----------------------------------------------------------------------------
// 5) The main branch-and-bound routine for DiscreteBayesNet
// ----------------------------------------------------------------------------
std::vector<Solution> branchAndBoundKSolutions(const DiscreteBayesNet& bayesNet,
                                               size_t K) {
  // Copy out the conditionals
  std::vector<std::shared_ptr<DiscreteConditional>> conditionals;
  for (auto& factor : bayesNet) {
    conditionals.push_back(factor);
  }

  // Min-heap of expansions by bound
  std::priority_queue<SearchNode, std::vector<SearchNode>, CompareByBound>
      expansions;

  // Max-heap of best solutions found so far, keyed by error.
  // We keep the largest error on top.
  Solutions solutions(K);

  // 1) Create the root node: no variables assigned, nextConditional = last.
  SearchNode root{.assignment = DiscreteValues(),
                  .error = 0.0,
                  .nextConditional = static_cast<int>(conditionals.size()) - 1};
  root.bound = root.computeBound();
  std::cout << "Root: " << root << std::endl;
  expansions.push(root);

  // 2) Main loop
  size_t numExpansions = 0;
  while (!expansions.empty()) {
    // Pop the partial assignment with the smallest bound
    SearchNode current = expansions.top();
    expansions.pop();
    numExpansions++;
    std::cout << "Expanding: " << current << std::endl;

    // If we already have K solutions, prune if we cannot beat the worst one.
    if (solutions.prune(current.bound)) {
      std::cout << "Pruning: bound=" << current.bound << std::endl;
      continue;
    }

    // Check if we have a complete assignment
    if (current.isComplete()) {
      const bool added = solutions.maybeAdd(current.error, current.assignment);
      if (added) {
        std::cout << "Best solutions so far:" << std::endl;
        solutions.print();
      }
      continue;
    }

    // Expand on the next factor
    const auto& conditional = conditionals[current.nextConditional];

    for (auto& fa : conditional->frontalAssignments()) {
      std::cout << "Frontal assignment: " << fa << std::endl;
      auto childNode = current.expand(*conditional, fa);

      // Again, prune if we cannot beat the worst solution
      if (solutions.prune(current.bound)) {
        std::cout << "Pruning: bound=" << childNode.bound << std::endl;
        continue;
      }

      expansions.push(childNode);
    }
  }

  std::cout << "Expansions: " << numExpansions << std::endl;

  // 3) Extract solutions from bestSolutions in ascending order of error
  return solutions.extractSolutions();
}

// ----------------------------------------------------------------------------
// Example “Unit Tests” (trivial stubs)
// ----------------------------------------------------------------------------

TEST_DISABLED(DiscreteBayesNet, EmptyKBest) {
  DiscreteBayesNet net;  // no factors
  auto solutions = branchAndBoundKSolutions(net, 3);
  // Expect one solution with empty assignment, error=0
  EXPECT_LONGS_EQUAL(1, solutions.size());
  EXPECT_DOUBLES_EQUAL(0, std::fabs(solutions[0].error), 1e-9);
}

TEST(DiscreteBayesNet, AsiaKBest) {
  DiscreteBayesNet net = constructAsiaExample();

  auto solutions = branchAndBoundKSolutions(net, 4);
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
