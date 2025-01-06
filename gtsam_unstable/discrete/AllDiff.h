/*
 * AllDiff.h
 * @brief General "all-different" constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam_unstable/discrete/BinaryAllDiff.h>

namespace gtsam {

/**
 * General AllDiff constraint.
 * Returns 1 if values for all keys are different, 0 otherwise.
 */
class GTSAM_UNSTABLE_EXPORT AllDiff : public Constraint {
  std::map<Key, size_t> cardinalities_;

  DiscreteKey discreteKey(size_t i) const {
    Key j = keys_[i];
    return DiscreteKey(j, cardinalities_.at(j));
  }

 public:
  /// Construct from keys.
  AllDiff(const DiscreteKeys& dkeys);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const AllDiff*>(&other))
      return false;
    else {
      const AllDiff& f(static_cast<const AllDiff&>(other));
      return cardinalities_.size() == f.cardinalities_.size() &&
             std::equal(cardinalities_.begin(), cardinalities_.end(),
                        f.cardinalities_.begin());
    }
  }

  /// Calculate value = expensive !
  double evaluate(const Assignment<Key>& values) const override;

  /// Convert into a decisiontree, can be *very* expensive !
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  /// Multiply factors, DiscreteFactor::shared_ptr edition
  DiscreteFactor::shared_ptr multiply(
      const DiscreteFactor::shared_ptr& df) const override {
    return std::make_shared<DecisionTreeFactor>(
        this->operator*(df->toDecisionTreeFactor()));
  }

  /// divide by DiscreteFactor::shared_ptr f (safely)
  DiscreteFactor::shared_ptr operator/(
      const DiscreteFactor::shared_ptr& df) const override;

  /// Compute error for each assignment and return as a tree
  AlgebraicDecisionTree<Key> errorTree() const override {
    throw std::runtime_error("AllDiff::error not implemented");
  }

  /*
   * Ensure Arc-consistency by checking every possible value of domain j.
   * @param j domain to be checked
   * @param (in/out) domains all domains, but only domains->at(j) will be checked.
   * @return true if domains->at(j) was changed, false otherwise.
   */
  bool ensureArcConsistency(Key j, Domains* domains) const override;

  /// Partially apply known values
  Constraint::shared_ptr partiallyApply(const DiscreteValues&) const override;

  /// Partially apply known values, domain version
  Constraint::shared_ptr partiallyApply(
      const Domains&) const override;

  /// Get the number of non-zero values contained in this factor.
  uint64_t nrValues() const override { return 1; };

  DiscreteFactor::shared_ptr sum(size_t nrFrontals) const override {
    return toDecisionTreeFactor().sum(nrFrontals);
  }

  DiscreteFactor::shared_ptr sum(const Ordering& keys) const override {
    return toDecisionTreeFactor().sum(keys);
  }

  DiscreteFactor::shared_ptr max(size_t nrFrontals) const override {
    return toDecisionTreeFactor().max(nrFrontals);
  }

  DiscreteFactor::shared_ptr max(const Ordering& keys) const override {
    return toDecisionTreeFactor().max(keys);
  }
};

}  // namespace gtsam
