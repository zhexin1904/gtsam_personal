/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file TableDistribution.h
 *  @date Dec 22, 2024
 *  @author Varun Agrawal
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/inference/Conditional-inst.h>

#include <memory>
#include <string>
#include <vector>

namespace gtsam {

/**
 * Distribution which uses a SparseVector as the internal
 * representation, similar to the TableFactor.
 *
 * This is primarily used in the case when we have a clique in the BayesTree
 * which consists of all the discrete variables, e.g. in hybrid elimination.
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT TableDistribution : public DiscreteConditional {
 private:
  TableFactor table_;

  typedef Eigen::SparseVector<double>::InnerIterator SparseIt;

 public:
  // typedefs needed to play nice with gtsam
  typedef TableDistribution This;            ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef DiscreteConditional
      BaseConditional;  ///< Typedef to our conditional base class

  using Values = DiscreteValues;  ///< backwards compatibility

  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  TableDistribution() {}

  /// Construct from factor, taking the first `nFrontals` keys as frontals.
  TableDistribution(size_t nFrontals, const TableFactor& f);

  /**
   * Construct from DiscreteKeys and SparseVector, taking the first
   * `nFrontals` keys as frontals, in the order given.
   */
  TableDistribution(size_t nFrontals, const DiscreteKeys& keys,
                    const Eigen::SparseVector<double>& potentials);

  /** Construct from signature */
  explicit TableDistribution(const Signature& signature);

  /**
   * Construct from key, parents, and a Signature::Table specifying the
   * conditional probability table (CPT) in 00 01 10 11 order. For
   * three-valued, it would be 00 01 02 10 11 12 20 21 22, etc....
   *
   * Example: TableDistribution P(D, {B,E}, table);
   */
  TableDistribution(const DiscreteKey& key, const DiscreteKeys& parents,
                    const Signature::Table& table)
      : TableDistribution(Signature(key, parents, table)) {}

  /**
   * Construct from key, parents, and a vector<double> specifying the
   * conditional probability table (CPT) in 00 01 10 11 order. For
   * three-valued, it would be 00 01 02 10 11 12 20 21 22, etc....
   *
   * Example: TableDistribution P(D, {B,E}, table);
   */
  TableDistribution(const DiscreteKey& key, const DiscreteKeys& parents,
                    const std::vector<double>& table)
      : TableDistribution(1, TableFactor(DiscreteKeys{key} & parents, table)) {}

  /**
   * Construct from key, parents, and a string specifying the conditional
   * probability table (CPT) in 00 01 10 11 order. For three-valued, it would
   * be 00 01 02 10 11 12 20 21 22, etc....
   *
   * The string is parsed into a Signature::Table.
   *
   * Example: TableDistribution P(D, {B,E}, "9/1 2/8 3/7 1/9");
   */
  TableDistribution(const DiscreteKey& key, const DiscreteKeys& parents,
                    const std::string& spec)
      : TableDistribution(Signature(key, parents, spec)) {}

  /// No-parent specialization; can also use DiscreteDistribution.
  TableDistribution(const DiscreteKey& key, const std::string& spec)
      : TableDistribution(Signature(key, {}, spec)) {}

  /**
   * @brief construct P(X|Y) = f(X,Y)/f(Y) from f(X,Y) and f(Y)
   * Assumes but *does not check* that f(Y)=sum_X f(X,Y).
   */
  TableDistribution(const TableFactor& joint, const TableFactor& marginal);

  /**
   * @brief construct P(X|Y) = f(X,Y)/f(Y) from f(X,Y) and f(Y)
   * Assumes but *does not check* that f(Y)=sum_X f(X,Y).
   * Makes sure the keys are ordered as given. Does not check orderedKeys.
   */
  TableDistribution(const TableFactor& joint, const TableFactor& marginal,
                    const Ordering& orderedKeys);

  /**
   * @brief Combine two conditionals, yielding a new conditional with the union
   * of the frontal keys, ordered by gtsam::Key.
   *
   * The two conditionals must make a valid Bayes net fragment, i.e.,
   * the frontal variables cannot overlap, and must be acyclic:
   * Example of correct use:
   *   P(A,B) = P(A|B) * P(B)
   *   P(A,B|C) = P(A|B) * P(B|C)
   *   P(A,B,C) = P(A,B|C) * P(C)
   * Example of incorrect use:
   *   P(A|B) * P(A|C) = ?
   *   P(A|B) * P(B|A) = ?
   * We check for overlapping frontals, but do *not* check for cyclic.
   */
  TableDistribution operator*(const TableDistribution& other) const;

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Discrete Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// GTSAM-style equals
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /// Return the underlying TableFactor
  TableFactor table() const { return table_; }

  using BaseConditional::evaluate;  // HybridValues version

  /// Evaluate the conditional given the values.
  virtual double evaluate(const Assignment<Key>& values) const override {
    return table_.evaluate(values);
  }

  /**
   * @brief Create new conditional by maximizing over all
   * values with the same separator.
   *
   * @param keys The keys to sum over.
   * @return DiscreteConditional::shared_ptr
   */
  virtual DiscreteConditional::shared_ptr max(
      const Ordering& keys) const override;

  /// @}
  /// @name Advanced Interface
  /// @{

  /// Set the underlying data from the DiscreteConditional
  virtual void setData(const DiscreteConditional::shared_ptr& dc) override;

  /// Prune the conditional
  virtual DiscreteConditional::shared_ptr prune(
      size_t maxNrAssignments) const override;

  /// Get a DecisionTreeFactor representation.
  DecisionTreeFactor toDecisionTreeFactor() const override {
    return table_.toDecisionTreeFactor();
  }

  /// Get the number of non-zero values.
  size_t nrValues() const { return table_.sparseTable().nonZeros(); }

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
    ar& BOOST_SERIALIZATION_NVP(table_);
  }
#endif
};
// TableDistribution

// traits
template <>
struct traits<TableDistribution> : public Testable<TableDistribution> {};

}  // namespace gtsam
