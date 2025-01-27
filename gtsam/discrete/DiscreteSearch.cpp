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

SearchNode SearchNode::Root(size_t numConditionals, double bound) {
  return {.assignment = DiscreteValues(),
          .error = 0.0,
          .bound = bound,
          .nextConditional = static_cast<int>(numConditionals) - 1};
}

SearchNode SearchNode::expand(const DiscreteConditional& conditional,
                              const DiscreteValues& fa) const {
  // Combine the new frontal assignment with the current partial assignment
  DiscreteValues newAssignment = assignment;
  for (auto& [key, value] : fa) {
    newAssignment[key] = value;
  }

  return {.assignment = newAssignment,
          .error = error + conditional.error(newAssignment),
          .bound = 0.0,
          .nextConditional = nextConditional - 1};
}

bool Solutions::maybeAdd(double error, const DiscreteValues& assignment) {
  const bool full = pq_.size() == maxSize_;
  if (full && error >= pq_.top().error) return false;
  if (full) pq_.pop();
  pq_.emplace(error, assignment);
  return true;
}

std::ostream& operator<<(std::ostream& os, const Solutions& sn) {
  os << "Solutions (top " << sn.pq_.size() << "):\n";
  auto pq = sn.pq_;
  while (!pq.empty()) {
    os << pq.top() << "\n";
    pq.pop();
  }
  return os;
}

bool Solutions::prune(double bound) const {
  if (pq_.size() < maxSize_) return false;
  return bound >= pq_.top().error;
}

std::vector<Solution> Solutions::extractSolutions() {
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

DiscreteSearch::DiscreteSearch(const DiscreteBayesNet& bayesNet) {
  std::vector<DiscreteConditional::shared_ptr> conditionals;
  for (auto& factor : bayesNet) conditionals_.push_back(factor);
  costToGo_ = computeCostToGo(conditionals_);
}

DiscreteSearch::DiscreteSearch(const DiscreteBayesTree& bayesTree) {
  std::function<void(const DiscreteBayesTree::sharedClique&)>
      collectConditionals = [&](const auto& clique) {
        if (!clique) return;
        for (const auto& child : clique->children) collectConditionals(child);
        conditionals_.push_back(clique->conditional());
      };
  for (const auto& root : bayesTree.roots()) collectConditionals(root);
  costToGo_ = computeCostToGo(conditionals_);
};

using SearchNodeQueue = std::priority_queue<SearchNode, std::vector<SearchNode>,
                                            SearchNode::Compare>;

std::vector<Solution> DiscreteSearch::run(size_t K) const {
  SearchNodeQueue expansions;
  expansions.push(SearchNode::Root(conditionals_.size(),
                                   costToGo_.empty() ? 0.0 : costToGo_.back()));

  Solutions solutions(K);

  auto expandNextNode = [&] {
    // Pop the partial assignment with the smallest bound
    SearchNode current = expansions.top();
    expansions.pop();

    // If we already have K solutions, prune if we cannot beat the worst
    // one.
    if (solutions.prune(current.bound)) {
      return;
    }

    // Check if we have a complete assignment
    if (current.isComplete()) {
      solutions.maybeAdd(current.error, current.assignment);
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
      if (!solutions.prune(childNode.bound)) {
        expansions.emplace(childNode);
      }
    }
  };

#ifdef DISCRETE_SEARCH_DEBUG
  size_t numExpansions = 0;
#endif

  // Perform the search
  while (!expansions.empty()) {
    expandNextNode();
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

std::vector<double> DiscreteSearch::computeCostToGo(
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

}  // namespace gtsam
