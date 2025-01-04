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

  /// Construct from TableFactor.
  TableDistribution(const TableFactor& f);

  /**
   * Construct from DiscreteKeys and SparseVector.
   */
  TableDistribution(const DiscreteKeys& keys,
                    const Eigen::SparseVector<double>& potentials);

  /**
   * Construct from DiscreteKeys and std::vector.
   */
  TableDistribution(const DiscreteKeys& keys,
                    const std::vector<double>& potentials);

  /**
   * Construct from single DiscreteKey and std::vector.
   */
  TableDistribution(const DiscreteKey& key,
                    const std::vector<double>& potentials)
      : TableDistribution(DiscreteKeys(key), potentials) {}

  /**
   * Construct from DiscreteKey and std::string.
   */
  TableDistribution(const DiscreteKeys& key, const std::string& potentials);

  /**
   * Construct from single DiscreteKey and std::string.
   */
  TableDistribution(const DiscreteKey& key, const std::string& potentials)
      : TableDistribution(DiscreteKeys(key), potentials) {}

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

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Table Distribution: ",
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
