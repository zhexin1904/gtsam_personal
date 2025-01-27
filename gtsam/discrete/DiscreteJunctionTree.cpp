/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteJunctionTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/inference/JunctionTree-inst.h>

namespace gtsam {

// Instantiate base classes
template class EliminatableClusterTree<DiscreteBayesTree, DiscreteFactorGraph>;
template class JunctionTree<DiscreteBayesTree, DiscreteFactorGraph>;

/* ************************************************************************* */
DiscreteJunctionTree::DiscreteJunctionTree(
    const DiscreteEliminationTree& eliminationTree)
    : Base(eliminationTree) {}
/* ************************************************************************* */
namespace {
struct PrintForestVisitorPre {
  const KeyFormatter& formatter;
  PrintForestVisitorPre(const KeyFormatter& formatter) : formatter(formatter) {}
  std::string operator()(
      const std::shared_ptr<DiscreteJunctionTree::Cluster>& node,
      const std::string& parentString) {
    // Print the current node
    node->print(parentString + "-", formatter);
    node->factors.print(parentString + "-", formatter);
    std::cout << std::endl;
    // Increment the indentation
    return parentString + "| ";
  }
};
}  // namespace

void DiscreteJunctionTree::print(const std::string& s,
                                 const KeyFormatter& keyFormatter) const {
  PrintForestVisitorPre visitor(keyFormatter);
  treeTraversal::DepthFirstForest(*this, s, visitor);
}

}  // namespace gtsam
