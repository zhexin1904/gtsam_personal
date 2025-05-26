/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DecisionTreeFactor.cpp
 * @brief discrete factor
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/base/FastSet.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/hybrid/HybridValues.h>

#include <utility>

using namespace std;

namespace gtsam {

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor() {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                         const ADT& potentials)
      : DiscreteFactor(keys.indices(), keys.cardinalities()), ADT(potentials) {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteConditional& c)
      : DiscreteFactor(c.keys(), c.cardinalities()),
        AlgebraicDecisionTree<Key>(c) {}

  /* ************************************************************************ */
  bool DecisionTreeFactor::equals(const DiscreteFactor& other,
                                  double tol) const {
    if (!dynamic_cast<const DecisionTreeFactor*>(&other)) {
      return false;
    } else {
      const auto& f(static_cast<const DecisionTreeFactor&>(other));
      return Base::equals(other, tol) && ADT::equals(f, tol);
    }
  }

  /* ************************************************************************ */
  double DecisionTreeFactor::error(const DiscreteValues& values) const {
    return -std::log(evaluate(values));
  }
  
  /* ************************************************************************ */
  double DecisionTreeFactor::error(const HybridValues& values) const {
    return error(values.discrete());
  }

  /* ************************************************************************ */
  DiscreteFactor::shared_ptr DecisionTreeFactor::multiply(
      const DiscreteFactor::shared_ptr& f) const {
    DiscreteFactor::shared_ptr result;
    if (auto tf = std::dynamic_pointer_cast<TableFactor>(f)) {
      // If f is a TableFactor, we convert `this` to a TableFactor since this
      // conversion is cheaper than converting `f` to a DecisionTreeFactor. We
      // then return a TableFactor.
      result = std::make_shared<TableFactor>((*tf) * TableFactor(*this));

    } else if (auto dtf = std::dynamic_pointer_cast<DecisionTreeFactor>(f)) {
      // If `f` is a DecisionTreeFactor, simply call operator*.
      result = std::make_shared<DecisionTreeFactor>(this->operator*(*dtf));

    } else {
      // Simulate double dispatch in C++
      // Useful for other classes which inherit from DiscreteFactor and have
      // only `operator*(DecisionTreeFactor)` defined. Thus, other classes don't
      // need to be updated.
      result = std::make_shared<DecisionTreeFactor>(f->operator*(*this));
    }
    return result;
  }

  /* ************************************************************************ */
  DiscreteFactor::shared_ptr DecisionTreeFactor::operator/(
      const DiscreteFactor::shared_ptr& f) const {
    if (auto tf = std::dynamic_pointer_cast<TableFactor>(f)) {
      // Check if `f` is a TableFactor. If yes, then
      // convert `this` to a TableFactor which is cheaper.
      return std::make_shared<TableFactor>(tf->operator/(TableFactor(*this)));

    } else if (auto dtf = std::dynamic_pointer_cast<DecisionTreeFactor>(f)) {
      // If `f` is a DecisionTreeFactor, divide normally.
      return std::make_shared<DecisionTreeFactor>(this->operator/(*dtf));

    } else {
      // Else, convert `f` to a DecisionTreeFactor so we can divide
      return std::make_shared<DecisionTreeFactor>(
          this->operator/(f->toDecisionTreeFactor()));
    }
  }

  /* ************************************************************************ */
  double DecisionTreeFactor::safe_div(const double& a, const double& b) {
    // The use for safe_div is when we divide the product factor by the sum
    // factor. If the product or sum is zero, we accord zero probability to the
    // event.
    return (a == 0 || b == 0) ? 0 : (a / b);
  }

  /* ************************************************************************ */
  void DecisionTreeFactor::print(const string& s,
                                 const KeyFormatter& formatter) const {
    cout << s;
    cout << " f[";
    for (auto&& key : keys()) {
      cout << " (" << formatter(key) << "," << cardinality(key) << "),";
    }
    cout << " ]" << endl;
    ADT::print("", formatter);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(Unary op) const {
    // apply operand
    ADT result = ADT::apply(op);
    // Make a new factor
    return DecisionTreeFactor(discreteKeys(), result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(UnaryAssignment op) const {
    // apply operand
    ADT result = ADT::apply(op);
    // Make a new factor
    return DecisionTreeFactor(discreteKeys(), result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(const DecisionTreeFactor& f,
                                               Binary op) const {
    map<Key, size_t> cs;  // new cardinalities
    // make unique key-cardinality map
    for (Key j : keys()) cs[j] = cardinality(j);
    for (Key j : f.keys()) cs[j] = f.cardinality(j);
    // Convert map into keys
    DiscreteKeys keys;
    keys.reserve(cs.size());
    for (const auto& key : cs) {
      keys.emplace_back(key);
    }
    // apply operand
    ADT result = ADT::apply(f, op);
    // Make a new factor
    return DecisionTreeFactor(keys, result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(size_t nrFrontals,
                                                             Binary op) const {
    if (nrFrontals > size()) {
      throw invalid_argument(
          "DecisionTreeFactor::combine: invalid number of frontal "
          "keys " +
          std::to_string(nrFrontals) + ", nr.keys=" + std::to_string(size()));
    }

    // sum over nrFrontals keys
    size_t i;
    ADT result(*this);
    for (i = 0; i < nrFrontals; i++) {
      Key j = keys_[i];
      result = result.combine(j, cardinality(j), op);
    }

    // Create new factor, note we start with keys after nrFrontals:
    DiscreteKeys dkeys;
    for (; i < keys_.size(); i++) {
      Key j = keys_[i];
      dkeys.push_back(DiscreteKey(j, cardinality(j)));
    }
    return std::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(
      const Ordering& frontalKeys, Binary op) const {
    if (frontalKeys.size() > size()) {
      throw invalid_argument(
          "DecisionTreeFactor::combine: invalid number of frontal "
          "keys " +
          std::to_string(frontalKeys.size()) + ", nr.keys=" +
          std::to_string(size()));
    }

    // sum over nrFrontals keys
    size_t i;
    ADT result(*this);
    for (i = 0; i < frontalKeys.size(); i++) {
      Key j = frontalKeys[i];
      result = result.combine(j, cardinality(j), op);
    }

    /*
    Create new factor, note we collect keys that are not in frontalKeys.
    
    Due to branch merging, the labels in `result` may be missing some keys.
    E.g. After branch merging, we may get a ADT like:
      Leaf [2] 1.0204082

    Hence, code below traverses the original keys and omits those in
    frontalKeys. We loop over cardinalities, which is O(n) even for a map, and
    then "contains" is a binary search on a small vector.
    */
    DiscreteKeys dkeys;
    for (auto&& [key, cardinality] : cardinalities_) {
      if (!frontalKeys.contains(key)) { 
        dkeys.push_back(DiscreteKey(key, cardinality));
      }
    }
    return std::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  std::vector<std::pair<DiscreteValues, double>> DecisionTreeFactor::enumerate()
      const {
    // Get all possible assignments
    DiscreteKeys pairs = discreteKeys();
    // Reverse to make cartesian product output a more natural ordering.
    DiscreteKeys rpairs(pairs.rbegin(), pairs.rend());
    const auto assignments = DiscreteValues::CartesianProduct(rpairs);

    // Construct unordered_map with values
    std::vector<std::pair<DiscreteValues, double>> result;
    for (const auto& assignment : assignments) {
      result.emplace_back(assignment, evaluate(assignment));
    }
    return result;
  }

  /* ************************************************************************ */
  std::vector<double> DecisionTreeFactor::probabilities() const {
    // Set of all keys
    KeySet allKeys(keys().begin(), keys().end());

    std::vector<double> probs;

    /* An operation that takes each leaf probability, and computes the
     * nrAssignments by checking the difference between the keys in the factor
     * and the keys in the assignment.
     * The nrAssignments is then used to append
     * the correct number of leaf probability values to the `probs` vector
     * defined above.
     */
    auto op = [&](const Assignment<Key>& a, double p) {
      // Get all the keys in the current assignment
      KeySet assignment_keys;
      for (auto&& [k, _] : a) {
        assignment_keys.insert(k);
      }

      // Find the keys missing in the assignment
      std::vector<Key> diff;
      std::set_difference(allKeys.begin(), allKeys.end(),
                          assignment_keys.begin(), assignment_keys.end(),
                          std::back_inserter(diff));

      // Compute the total number of assignments in the (pruned) subtree
      size_t nrAssignments = 1;
      for (auto&& k : diff) {
        nrAssignments *= cardinalities_.at(k);
      }
      // Add p `nrAssignments` times to the probs vector.
      probs.insert(probs.end(), nrAssignments, p);

      return p;
    };

    // Go through the tree
    this->visitWith(op);

    return probs;
  }

  /* ************************************************************************ */
  static std::string valueFormatter(const double& v) {
    std::stringstream ss;
    ss << std::setw(4) << std::setprecision(2) << std::fixed << v;
    return ss.str();
  }

  /** output to graphviz format, stream version */
  void DecisionTreeFactor::dot(std::ostream& os,
                               const KeyFormatter& keyFormatter,
                               bool showZero) const {
    ADT::dot(os, keyFormatter, valueFormatter, showZero);
  }

  /** output to graphviz format, open a file */
  void DecisionTreeFactor::dot(const std::string& name,
                              const KeyFormatter& keyFormatter,
                              bool showZero) const {
    ADT::dot(name, keyFormatter, valueFormatter, showZero);
  }

  /** output to graphviz format string */
  std::string DecisionTreeFactor::dot(const KeyFormatter& keyFormatter,
                                      bool showZero) const {
    return ADT::dot(keyFormatter, valueFormatter, showZero);
  }

  // Print out header.
  /* ************************************************************************ */
  string DecisionTreeFactor::markdown(const KeyFormatter& keyFormatter,
                                      const Names& names) const {
    stringstream ss;

    // Print out header.
    ss << "|";
    for (auto& key : keys()) {
      ss << keyFormatter(key) << "|";
    }
    ss << "value|\n";

    // Print out separator with alignment hints.
    ss << "|";
    for (size_t j = 0; j < size(); j++) ss << ":-:|";
    ss << ":-:|\n";

    // Print out all rows.
    auto rows = enumerate();
    for (const auto& kv : rows) {
      ss << "|";
      auto assignment = kv.first;
      for (auto& key : keys()) {
        size_t index = assignment.at(key);
        ss << DiscreteValues::Translate(names, key, index) << "|";
      }
      ss << kv.second << "|\n";
    }
    return ss.str();
  }

  /* ************************************************************************ */
  string DecisionTreeFactor::html(const KeyFormatter& keyFormatter,
                                  const Names& names) const {
    stringstream ss;

    // Print out preamble.
    ss << "<div>\n<table class='DecisionTreeFactor'>\n  <thead>\n";

    // Print out header row.
    ss << "    <tr>";
    for (auto& key : keys()) {
      ss << "<th>" << keyFormatter(key) << "</th>";
    }
    ss << "<th>value</th></tr>\n";

    // Finish header and start body.
    ss << "  </thead>\n  <tbody>\n";

    // Print out all rows.
    auto rows = enumerate();
    for (const auto& kv : rows) {
      ss << "    <tr>";
      auto assignment = kv.first;
      for (auto& key : keys()) {
        size_t index = assignment.at(key);
        ss << "<th>" << DiscreteValues::Translate(names, key, index) << "</th>";
      }
      ss << "<td>" << kv.second << "</td>";  // value
      ss << "</tr>\n";
    }
    ss << "  </tbody>\n</table>\n</div>";
    return ss.str();
  }

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                         const vector<double>& table)
      : DiscreteFactor(keys.indices(), keys.cardinalities()),
        AlgebraicDecisionTree<Key>(keys, table) {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                         const string& table)
      : DiscreteFactor(keys.indices(), keys.cardinalities()),
        AlgebraicDecisionTree<Key>(keys, table) {}

  /**
   * @brief Min-Heap class to help with pruning.
   * The `top` element is always the smallest value.
   */
  class MinHeap {
    std::vector<double> v_;

   public:
    /// Default constructor
    MinHeap() {}

    /// Push value onto the heap
    void push(double x) {
      v_.push_back(x);
      std::push_heap(v_.begin(), v_.end(), std::greater<double>{});
    }

    /// Push value `x`, `n` number of times.
    void push(double x, size_t n) {
      for (size_t i = 0; i < n; ++i) {
        v_.push_back(x);
        std::push_heap(v_.begin(), v_.end(), std::greater<double>{});
      }
    }

    /// Pop the top value of the heap.
    double pop() {
      std::pop_heap(v_.begin(), v_.end(), std::greater<double>{});
      double x = v_.back();
      v_.pop_back();
      return x;
    }

    /// Return the top value of the heap without popping it.
    double top() { return v_.at(0); }

    /**
     * @brief Print the heap as a sequence.
     *
     * @param s A string to prologue the output.
     */
    void print(const std::string& s = "") {
      std::cout << (s.empty() ? "" : s + " ");
      for (size_t i = 0; i < v_.size(); i++) {
        std::cout << v_.at(i);
        if (v_.size() > 1 && i < v_.size() - 1) std::cout << ", ";
      }
      std::cout << std::endl;
    }

    /// Return true if heap is empty.
    bool empty() const { return v_.empty(); }

    /// Return the size of the heap.
    size_t size() const { return v_.size(); }
  };

  /* ************************************************************************ */
  double DecisionTreeFactor::computeThreshold(const size_t N) const {
    // Set of all keys
    std::set<Key> allKeys = this->labels();
    MinHeap min_heap;

    auto op = [&](const Assignment<Key>& a, double p) {
      // Get all the keys in the current assignment
      KeySet assignment_keys;
      for (auto&& [k, _] : a) {
        assignment_keys.insert(k);
      }

      // Find the keys missing in the assignment
      KeyVector diff;
      std::set_difference(allKeys.begin(), allKeys.end(),
                          assignment_keys.begin(), assignment_keys.end(),
                          std::back_inserter(diff));

      // Compute the total number of assignments in the (pruned) subtree
      size_t nrAssignments = 1;
      for (auto&& k : diff) {
        nrAssignments *= cardinalities_.at(k);
      }

      // If min-heap is empty, fill it initially.
      // This is because there is nothing at the top.
      if (min_heap.empty()) {
        min_heap.push(p, std::min(nrAssignments, N));

      } else {
        for (size_t i = 0; i < std::min(nrAssignments, N); ++i) {
          // If p is larger than the smallest element,
          // then we insert into the min heap.
          // We check against the top each time because the
          // heap maintains the smallest element at the top.
          if (p > min_heap.top()) {
            if (min_heap.size() == N) {
              min_heap.pop();
            }
            min_heap.push(p);
          } else {
            // p is <= min value so move to the next one
            break;
          }
        }
      }
      return p;
    };
    this->visitWith(op);

    // If total number of hypotheses is less than N, return 0.0
    if (min_heap.size() < N) {
      return 0.0;
    }
    return min_heap.top();
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::prune(size_t maxNrAssignments) const {
    const size_t N = maxNrAssignments;

    double threshold = computeThreshold(N);

    // Now threshold the decision tree
    size_t total = 0;
    auto thresholdFunc = [threshold, &total, N](const double& value) {
      // There is a possible case where the `threshold` is equal to 0.0
      // In that case `(value < threshold) == false`
      // which increases the leaf total erroneously.
      // Hence we check for 0.0 explicitly.
      if (fpEqual(value, 0.0, 1e-12)) {
        return 0.0;
      }

      // Check if value is less than the threshold and
      // we haven't exceeded the maximum number of leaves.
      // TODO(Varun): Bug since we can have a case where we need to prune higher
      // probabilities after we have reached N.
      // E.g. N=3 for [0.2, 0.2, 0.1, 0.2, 0.3]
      // will give [0.2, 0.2, 0.0, 0.2, 0.0]
      if (value < threshold || total >= N) {
        return 0.0;
      } else {
        total += 1;
        return value;
      }
    };
    DecisionTree<Key, double> thresholded(*this, thresholdFunc);

    // Create pruned decision tree factor and return.
    return DecisionTreeFactor(this->discreteKeys(), thresholded);
  }

/* ************************************************************************ */
DiscreteFactor::shared_ptr DecisionTreeFactor::restrict(
    const DiscreteValues& assignment) const {
  ADT restricted_tree = ADT::restrict(assignment);
  // Get all the keys that are not restricted by the assignment
  // This ensures that the new restricted factor doesn't have keys
  // for which the information has been removed.
  DiscreteKeys restricted_keys = this->discreteKeys();
  for (auto&& kv : assignment) {
    Key key = kv.first;
    // Remove the key from the keys list
    restricted_keys.erase(
        std::remove_if(restricted_keys.begin(), restricted_keys.end(),
                       [key](const DiscreteKey& k) { return k.first == key; }),
        restricted_keys.end());
  }
  // Create the restricted factor with the appropriate keys and tree.
  return std::make_shared<DecisionTreeFactor>(restricted_keys, restricted_tree);
}

  /* ************************************************************************ */
}  // namespace gtsam
