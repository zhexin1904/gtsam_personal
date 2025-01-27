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

#include <gtsam/discrete/DiscreteSearch.h>

namespace gtsam {

using Slot = DiscreteSearch::Slot;
using Solution = DiscreteSearch::Solution;

/**
 * @brief Represents a node in the search tree for discrete search algorithms.
 *
 * @details Each SearchNode contains a partial assignment of discrete variables,
 * the current error, a bound on the final error, and the index of the next
 * conditional to be assigned.
 */
struct SearchNode {
  DiscreteValues assignment;  ///< Partial assignment of discrete variables.
  double error;               ///< Current error for the partial assignment.
  double bound;  ///< Lower bound on the final error for unassigned variables.
  int nextConditional;  ///< Index of the next conditional to be assigned.

  /**
   * @brief Construct the root node for the search.
   */
  static SearchNode Root(size_t numConditionals, double bound) {
    return {DiscreteValues(), 0.0, bound,
            static_cast<int>(numConditionals) - 1};
  }

  struct Compare {
    bool operator()(const SearchNode& a, const SearchNode& b) const {
      return a.bound > b.bound;  // smallest bound -> highest priority
    }
  };

  /**
   * @brief Checks if the node represents a complete assignment.
   *
   * @return True if all variables have been assigned, false otherwise.
   */
  inline bool isComplete() const { return nextConditional < 0; }

  /**
   * @brief Expands the node by assigning the next variable.
   *
   * @param factor The discrete factor associated with the next variable
   * to be assigned.
   * @param fa The frontal assignment for the next variable.
   * @return A new SearchNode representing the expanded state.
   */
  SearchNode expand(const DiscreteFactor& factor,
                    const DiscreteValues& fa) const {
    // Combine the new frontal assignment with the current partial assignment
    DiscreteValues newAssignment = assignment;
    for (auto& [key, value] : fa) {
      newAssignment[key] = value;
    }

    return {newAssignment, error + factor.error(newAssignment), 0.0,
            nextConditional - 1};
  }

  /**
   * @brief Prints the SearchNode to an output stream.
   *
   * @param os The output stream.
   * @param node The SearchNode to be printed.
   * @return The output stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const SearchNode& node) {
    os << "SearchNode(error=" << node.error << ", bound=" << node.bound << ")";
    return os;
  }
};

struct CompareSolution {
  bool operator()(const Solution& a, const Solution& b) const {
    return a.error < b.error;
  }
};

// Define the Solutions class
class Solutions {
 private:
  size_t maxSize_;
  std::priority_queue<Solution, std::vector<Solution>, CompareSolution> pq_;

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
  friend std::ostream& operator<<(std::ostream& os, const Solutions& sn) {
    os << "Solutions (top " << sn.pq_.size() << "):\n";
    auto pq = sn.pq_;
    while (!pq.empty()) {
      os << pq.top() << "\n";
      pq.pop();
    }
    return os;
  }

  /// Check if (partial) solution with given bound can be pruned. If we have
  /// room, we never prune. Otherwise, prune if lower bound on error is worse
  /// than our current worst error.
  bool prune(double bound) const {
    if (pq_.size() < maxSize_) return false;
    return bound >= pq_.top().error;
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

DiscreteSearch::DiscreteSearch(const DiscreteFactorGraph& factorGraph) {
  slots_.reserve(factorGraph.size());
  for (auto& factor : factorGraph) {
    slots_.emplace_back(factor, std::vector<DiscreteValues>{}, 0.0);
  }
  computeHeuristic();
}

DiscreteSearch::DiscreteSearch(const DiscreteBayesNet& bayesNet) {
  slots_.reserve(bayesNet.size());
  for (auto& conditional : bayesNet) {
    slots_.emplace_back(conditional, conditional->frontalAssignments(), 0.0);
  }
  computeHeuristic();
}

DiscreteSearch::DiscreteSearch(const DiscreteBayesTree& bayesTree) {
  std::function<void(const DiscreteBayesTree::sharedClique&)>
      collectConditionals = [&](const auto& clique) {
        if (!clique) return;
        for (const auto& child : clique->children) collectConditionals(child);
        auto conditional = clique->conditional();
        slots_.emplace_back(conditional, conditional->frontalAssignments(),
                            0.0);
      };

  slots_.reserve(bayesTree.size());
  for (const auto& root : bayesTree.roots()) collectConditionals(root);
  computeHeuristic();
}

struct SearchNodeQueue
    : public std::priority_queue<SearchNode, std::vector<SearchNode>,
                                 SearchNode::Compare> {
  void expandNextNode(const SearchNode& current, const Slot& slot,
                      Solutions* solutions) {
    // If we already have K solutions, prune if we cannot beat the worst one.
    if (solutions->prune(current.bound)) {
      return;
    }

    // Check if we have a complete assignment
    if (current.isComplete()) {
      solutions->maybeAdd(current.error, current.assignment);
      return;
    }

    for (auto& fa : slot.assignments) {
      auto childNode = current.expand(*slot.factor, fa);
      if (childNode.nextConditional >= 0)
        // TODO(frank): this might be wrong !
        childNode.bound = childNode.error + slot.heuristic;

      // Again, prune if we cannot beat the worst solution
      if (!solutions->prune(childNode.bound)) {
        emplace(childNode);
      }
    }
  }
};

std::vector<Solution> DiscreteSearch::run(size_t K) const {
  Solutions solutions(K);
  SearchNodeQueue expansions;
  expansions.push(SearchNode::Root(slots_.size(), slots_.back().heuristic));

#ifdef DISCRETE_SEARCH_DEBUG
  size_t numExpansions = 0;
#endif

  // Perform the search
  while (!expansions.empty()) {
    // Pop the partial assignment with the smallest bound
    SearchNode current = expansions.top();
    expansions.pop();

    // Get the next slot to expand
    const auto& slot = slots_[current.nextConditional];
    expansions.expandNextNode(current, slot, &solutions);
#ifdef DISCRETE_SEARCH_DEBUG
    ++numExpansions;
#endif
  }

#ifdef DISCRETE_SEARCH_DEBUG
  std::cout << "Number of expansions: " << numExpansions << std::endl;
#endif

  // Extract solutions from bestSolutions in ascending order of error
  return solutions.extractSolutions();
}

// We have a number of factors, each with a max value, and we want to compute
// the a lower-bound on the cost-to-go for each slot. For the first slot, this
// -log(max(factor[0])), as we only have one factor to resolve. For the second
// slot, we need to add -log(max(factor[1])) to it, etc...
void DiscreteSearch::computeHeuristic() {
  double error = 0.0;
  for (size_t i = 0; i < slots_.size(); ++i) {
    const auto& factor = slots_[i].factor;
    Ordering ordering(factor->begin(), factor->end());
    auto maxx = factor->max(ordering);
    error -= std::log(maxx->evaluate({}));
    slots_[i].heuristic = error;
  }
}

}  // namespace gtsam
