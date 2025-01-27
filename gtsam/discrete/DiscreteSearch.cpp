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
  for (auto& kv : fa) {
    newAssignment[kv.first] = kv.second;
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
  auto pq = sn.pq_;
  while (!pq.empty()) {
    const Solution& best = pq.top();
    os << "Error: " << best.error << ", Values: " << best.assignment
       << std::endl;
    pq.pop();
  }
  return os;
}

bool Solutions::prune(double bound) const {
  if (pq_.size() < maxSize_) return false;
  double worstError = pq_.top().error;
  return (bound >= worstError);
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

DiscreteSearch::DiscreteSearch(const DiscreteBayesNet& bayesNet, size_t K)
    : solutions_(K) {
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

DiscreteSearch::DiscreteSearch(const DiscreteBayesTree& bayesTree, size_t K)
    : solutions_(K) {
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

std::vector<Solution> DiscreteSearch::run() {
  while (!expansions_.empty()) {
    numExpansions++;
    expandNextNode();
  }

  // Extract solutions from bestSolutions in ascending order of error
  return solutions_.extractSolutions();
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

void DiscreteSearch::expandNextNode() {
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
      childNode.bound = childNode.error + costToGo_[childNode.nextConditional];

    // Again, prune if we cannot beat the worst solution
    if (!solutions_.prune(childNode.bound)) {
      expansions_.push(childNode);
    }
  }
}

}  // namespace gtsam
