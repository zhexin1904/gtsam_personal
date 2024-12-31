/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteTableConditional.cpp
 *  @date Dec 22, 2024
 *  @author Varun Agrawal
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>
#include <gtsam/discrete/DiscreteTableConditional.h>
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
DiscreteTableConditional::DiscreteTableConditional(const size_t nrFrontals,
                                                   const TableFactor& f)
    : BaseConditional(nrFrontals, DecisionTreeFactor(f.discreteKeys(), ADT())),
      table_(f / (*f.sum(nrFrontals))) {}

/* ************************************************************************** */
DiscreteTableConditional::DiscreteTableConditional(
    size_t nrFrontals, const DiscreteKeys& keys,
    const Eigen::SparseVector<double>& potentials)
    : BaseConditional(nrFrontals, keys, DecisionTreeFactor(keys, ADT())),
      table_(TableFactor(keys, potentials)) {}

/* ************************************************************************** */
DiscreteTableConditional::DiscreteTableConditional(const TableFactor& joint,
                                                   const TableFactor& marginal)
    : BaseConditional(joint.size() - marginal.size(),
                      joint.discreteKeys() & marginal.discreteKeys(), ADT()),
      table_(joint / marginal) {}

/* ************************************************************************** */
DiscreteTableConditional::DiscreteTableConditional(const TableFactor& joint,
                                                   const TableFactor& marginal,
                                                   const Ordering& orderedKeys)
    : DiscreteTableConditional(joint, marginal) {
  keys_.clear();
  keys_.insert(keys_.end(), orderedKeys.begin(), orderedKeys.end());
}

/* ************************************************************************** */
DiscreteTableConditional::DiscreteTableConditional(const Signature& signature)
    : BaseConditional(1, DecisionTreeFactor()),
      table_(TableFactor(signature.discreteKeys(), signature.cpt())) {}

/* ************************************************************************** */
DiscreteTableConditional DiscreteTableConditional::operator*(
    const DiscreteTableConditional& other) const {
  // Take union of frontal keys
  std::set<Key> newFrontals;
  for (auto&& key : this->frontals()) newFrontals.insert(key);
  for (auto&& key : other.frontals()) newFrontals.insert(key);

  // Check if frontals overlapped
  if (nrFrontals() + other.nrFrontals() > newFrontals.size())
    throw std::invalid_argument(
        "DiscreteTableConditional::operator* called with overlapping frontal "
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
  return DiscreteTableConditional(newFrontals.size(), product);
}

/* ************************************************************************** */
void DiscreteTableConditional::print(const string& s,
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
bool DiscreteTableConditional::equals(const DiscreteFactor& other,
                                      double tol) const {
  if (!dynamic_cast<const DiscreteConditional*>(&other)) {
    return false;
  } else {
    const DiscreteConditional& f(
        static_cast<const DiscreteConditional&>(other));
    return DiscreteConditional::equals(f, tol);
  }
}

/* ************************************************************************** */
TableFactor::shared_ptr DiscreteTableConditional::likelihood(
    const DiscreteValues& frontalValues) const {
  throw std::runtime_error("Likelihood not implemented");
}

/* ****************************************************************************/
TableFactor::shared_ptr DiscreteTableConditional::likelihood(
    size_t frontal) const {
  throw std::runtime_error("Likelihood not implemented");
}

/* ************************************************************************** */
size_t DiscreteTableConditional::argmax(
    const DiscreteValues& parentsValues) const {
  // Initialize
  size_t maxValue = 0;
  double maxP = 0;
  DiscreteValues values = parentsValues;

  assert(nrFrontals() == 1);
  Key j = firstFrontalKey();
  for (size_t value = 0; value < cardinality(j); value++) {
    values[j] = value;
    double pValueS = (*this)(values);
    // Update MPE solution if better
    if (pValueS > maxP) {
      maxP = pValueS;
      maxValue = value;
    }
  }
  return maxValue;
}

}  // namespace gtsam
