/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DiscreteSearch.cpp
 *
 * @date January, 2025
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteBayesTree.h>

#include <queue>

namespace gtsam {

/**
 * DiscreteSearch: Search for the K best solutions.
 */
class GTSAM_EXPORT DiscreteSearch {
 public:
  /// We structure the search as a set of slots, each with a factor and
  /// a set of variable assignments that need to be chosen. In addition, each
  /// slot has a heuristic associated with it.
  struct Slot {
    /// The factors in the search problem,
    ///  e.g., [P(B|A),P(A)]
    DiscreteFactor::shared_ptr factor;

    /// The assignments for each factor,
    /// e.g., [[B0,B1] [A0,A1]]
    std::vector<DiscreteValues> assignments;

    /// A lower bound on the cost-to-go for each slot, e.g.,
    /// [-log(max_B P(B|A)), -log(max_A P(A))]
    double heuristic;
  };

  /// A solution is then a set of assignments, covering all the slots.
  /// as well as an associated error = -log(probability)
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

 public:
  /**
   * Construct from a DiscreteFactorGraph.
   */
  DiscreteSearch(const DiscreteFactorGraph& bayesNet);

  /**
   * Construct from a DiscreteBayesNet.
   */
  DiscreteSearch(const DiscreteBayesNet& bayesNet);

  /**
   * Construct from a DiscreteBayesTree.
   */
  DiscreteSearch(const DiscreteBayesTree& bayesTree);

  /**
   * @brief Search for the K best solutions.
   *
   * This method performs a search to find the K best solutions for the given
   * DiscreteBayesNet. It uses a priority queue to manage the search nodes,
   * expanding nodes with the smallest bound first. The search continues until
   * all possible nodes have been expanded or pruned.
   *
   * @return A vector of the K best solutions found during the search.
   */
  std::vector<Solution> run(size_t K = 1) const;

 private:
  /// Compute the cumulative lower-bound cost-to-go for each slot.
  void computeHeuristic();

  std::vector<Slot> slots_;
};
}  // namespace gtsam
