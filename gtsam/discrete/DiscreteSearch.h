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

namespace gtsam {

using Value = size_t;

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
    return {.assignment = DiscreteValues(),
            .error = 0.0,
            .bound = bound,
            .nextConditional = static_cast<int>(numConditionals) - 1};
  }

  struct CompareByBound {
    bool operator()(const SearchNode& a, const SearchNode& b) const {
      return a.bound > b.bound;  // smallest bound -> highest priority
    }
  };

  /**
   * @brief Checks if the node represents a complete assignment.
   *
   * @return True if all variables have been assigned, false otherwise.
   */
  bool isComplete() const { return nextConditional < 0; }

  /**
   * @brief Expands the node by assigning the next variable.
   *
   * @param conditional The discrete conditional representing the next variable
   * to be assigned.
   * @param fa The frontal assignment for the next variable.
   * @return A new SearchNode representing the expanded state.
   */
  SearchNode expand(const DiscreteConditional& conditional,
                    const DiscreteValues& fa) const {
    // Combine the new frontal assignment with the current partial assignment
    DiscreteValues newAssignment = assignment;
    for (auto& kv : fa) {
      newAssignment[kv.first] = kv.second;
    }

    return {.assignment = newAssignment,
            .error = error + conditional.error(newAssignment),
            .bound = 0.0,
            .nextConditional = nextConditional - 1};
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
  friend std::ostream& operator<<(std::ostream& os, const Solutions& sn) {
    auto pq = sn.pq_;
    while (!pq.empty()) {
      const Solution& best = pq.top();
      os << "Error: " << best.error << ", Values: " << best.assignment
         << std::endl;
      pq.pop();
    }
    return os;
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

/**
 * DiscreteSearch: Search for the K best solutions.
 */
class DiscreteSearch {
 public:
  size_t numExpansions = 0;

  /**
   * Construct from a DiscreteBayesNet and K.
   */
  DiscreteSearch(const DiscreteBayesNet& bayesNet, size_t K) : solutions_(K) {
    // Copy out the conditionals
    for (auto& factor : bayesNet) {
      conditionals_.push_back(factor);
    }

    // Calculate the cost-to-go for each conditional
    costToGo_ = computeCostToGo(conditionals_);

    // Create the root node and push it to the expansions queue
    expansions_.push(SearchNode::Root(
        conditionals_.size(), costToGo_.empty() ? 0.0 : costToGo_.back()));
  }

  /**
   * Construct from a DiscreteBayesTree and K.
   */
  DiscreteSearch(const DiscreteBayesTree& bayesTree, size_t K) : solutions_(K) {
    using CliquePtr = DiscreteBayesTree::sharedClique;
    std::function<void(const CliquePtr&)> collectConditionals =
        [&](const CliquePtr& clique) -> void {
      if (!clique) return;

      // Recursive post-order traversal: process children first
      for (const auto& child : clique->children) {
        collectConditionals(child);
      }

      // Then add the current clique's conditional
      conditionals_.push_back(clique->conditional());
    };

    // Start traversal from each root in the tree
    for (const auto& root : bayesTree.roots()) collectConditionals(root);

    // Calculate the cost-to-go for each conditional
    costToGo_ = computeCostToGo(conditionals_);

    // Create the root node and push it to the expansions queue
    expansions_.push(SearchNode::Root(
        conditionals_.size(), costToGo_.empty() ? 0.0 : costToGo_.back()));
  }

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
  std::vector<Solution> run() {
    while (!expansions_.empty()) {
      numExpansions++;
      expandNextNode();
    }

    // Extract solutions from bestSolutions in ascending order of error
    return solutions_.extractSolutions();
  }

 private:
  /**
   * @brief Compute the cost-to-go for each conditional.
   *
   * @param conditionals The conditionals of the DiscreteBayesNet.
   * @return A vector of cost-to-go values.
   */
  static std::vector<double> computeCostToGo(
      const std::vector<DiscreteConditional::shared_ptr>& conditionals) {
    std::vector<double> costToGo;
    double error = 0.0;
    for (const auto& conditional : conditionals) {
      Ordering ordering(conditional->begin(), conditional->end());
      auto maxx = conditional->max(ordering);
      assert(maxx->size() == 1);
      error -= std::log(maxx->evaluate({}));
      costToGo.push_back(error);
    }
    return costToGo;
  }

  /**
   * @brief Expand the next node in the search tree.
   */
  void expandNextNode() {
    // Pop the partial assignment with the smallest bound
    SearchNode current = expansions_.top();
    expansions_.pop();

    // If we already have K solutions, prune if we cannot beat the worst one.
    if (solutions_.prune(current.bound)) {
      return;
    }

    // Check if we have a complete assignment
    if (current.isComplete()) {
      solutions_.maybeAdd(current.error, current.assignment);
      return;
    }

    // Expand on the next factor
    const auto& conditional = conditionals_[current.nextConditional];

    for (auto& fa : conditional->frontalAssignments()) {
      auto childNode = current.expand(*conditional, fa);
      if (childNode.nextConditional >= 0)
        childNode.bound =
            childNode.error + costToGo_[childNode.nextConditional];

      // Again, prune if we cannot beat the worst solution
      if (!solutions_.prune(childNode.bound)) {
        expansions_.push(childNode);
      }
    }
  }

  std::vector<DiscreteConditional::shared_ptr> conditionals_;
  std::vector<double> costToGo_;
  std::priority_queue<SearchNode, std::vector<SearchNode>,
                      SearchNode::CompareByBound>
      expansions_;
  Solutions solutions_;
};
}  // namespace gtsam
