/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file TableDistribution.cpp
 *  @date Dec 22, 2024
 *  @author Varun Agrawal
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>
#include <gtsam/discrete/TableDistribution.h>
#include <gtsam/discrete/Ring.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/hybrid/HybridValues.h>

#include <algorithm>
#include <cassert>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std;
using std::pair;
using std::stringstream;
using std::vector;
namespace gtsam {

/* ************************************************************************** */
TableDistribution::TableDistribution(const size_t nrFrontals,
                                                   const TableFactor& f)
    : BaseConditional(nrFrontals, DecisionTreeFactor(f.discreteKeys(), ADT())),
      table_(f / (*f.sum(nrFrontals))) {}

/* ************************************************************************** */
TableDistribution::TableDistribution(
    size_t nrFrontals, const DiscreteKeys& keys,
    const Eigen::SparseVector<double>& potentials)
    : BaseConditional(nrFrontals, keys, DecisionTreeFactor(keys, ADT())),
      table_(TableFactor(keys, potentials)) {}

/* ************************************************************************** */
TableDistribution::TableDistribution(const TableFactor& joint,
                                                   const TableFactor& marginal)
    : BaseConditional(joint.size() - marginal.size(),
                      joint.discreteKeys() & marginal.discreteKeys(), ADT()),
      table_(joint / marginal) {}

/* ************************************************************************** */
TableDistribution::TableDistribution(const TableFactor& joint,
                                                   const TableFactor& marginal,
                                                   const Ordering& orderedKeys)
    : TableDistribution(joint, marginal) {
  keys_.clear();
  keys_.insert(keys_.end(), orderedKeys.begin(), orderedKeys.end());
}

/* ************************************************************************** */
TableDistribution::TableDistribution(const Signature& signature)
    : BaseConditional(1, DecisionTreeFactor(DiscreteKeys{{1, 1}}, ADT(1))),
      table_(TableFactor(signature.discreteKeys(), signature.cpt())) {}

/* ************************************************************************** */
TableDistribution TableDistribution::operator*(
    const TableDistribution& other) const {
  // Take union of frontal keys
  std::set<Key> newFrontals;
  for (auto&& key : this->frontals()) newFrontals.insert(key);
  for (auto&& key : other.frontals()) newFrontals.insert(key);

  // Check if frontals overlapped
  if (nrFrontals() + other.nrFrontals() > newFrontals.size())
    throw std::invalid_argument(
        "TableDistribution::operator* called with overlapping frontal "
        "keys.");

  // Now, add cardinalities.
  DiscreteKeys discreteKeys;
  for (auto&& key : frontals())
    discreteKeys.emplace_back(key, cardinality(key));
  for (auto&& key : other.frontals())
    discreteKeys.emplace_back(key, other.cardinality(key));

  // Sort
  std::sort(discreteKeys.begin(), discreteKeys.end());

  // Add parents to set, to make them unique
  std::set<DiscreteKey> parents;
  for (auto&& key : this->parents())
    if (!newFrontals.count(key)) parents.emplace(key, cardinality(key));
  for (auto&& key : other.parents())
    if (!newFrontals.count(key)) parents.emplace(key, other.cardinality(key));

  // Finally, add parents to keys, in order
  for (auto&& dk : parents) discreteKeys.push_back(dk);

  TableFactor product = this->table_ * other.table();
  return TableDistribution(newFrontals.size(), product);
}

/* ************************************************************************** */
void TableDistribution::print(const string& s,
                                     const KeyFormatter& formatter) const {
  cout << s << " P( ";
  for (const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
    cout << formatter(*it) << " ";
  }
  if (nrParents()) {
    cout << "| ";
    for (const_iterator it = beginParents(); it != endParents(); ++it) {
      cout << formatter(*it) << " ";
    }
  }
  cout << "):\n";
  table_.print("", formatter);
  cout << endl;
}

/* ************************************************************************** */
bool TableDistribution::equals(const DiscreteFactor& other,
                                      double tol) const {
  auto dtc = dynamic_cast<const TableDistribution*>(&other);
  if (!dtc) {
    return false;
  } else {
    const DiscreteConditional& f(
        static_cast<const DiscreteConditional&>(other));
    return table_.equals(dtc->table_, tol) &&
           DiscreteConditional::BaseConditional::equals(f, tol);
  }
}

/* ****************************************************************************/
DiscreteConditional::shared_ptr TableDistribution::max(
    const Ordering& keys) const {
  auto m = *table_.max(keys);

  return std::make_shared<TableDistribution>(m.discreteKeys().size(), m);
}

/* ****************************************************************************/
void TableDistribution::setData(
    const DiscreteConditional::shared_ptr& dc) {
  if (auto dtc = std::dynamic_pointer_cast<TableDistribution>(dc)) {
    this->table_ = dtc->table_;
  } else {
    this->table_ = TableFactor(dc->discreteKeys(), *dc);
  }
}

/* ****************************************************************************/
DiscreteConditional::shared_ptr TableDistribution::prune(
    size_t maxNrAssignments) const {
  TableFactor pruned = table_.prune(maxNrAssignments);

  return std::make_shared<TableDistribution>(
      this->nrFrontals(), this->discreteKeys(), pruned.sparseTable());
}

}  // namespace gtsam
