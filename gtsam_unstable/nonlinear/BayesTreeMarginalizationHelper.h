/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeMarginalizationHelper.h
 * @brief   Helper functions for marginalizing variables from a Bayes Tree.
 *
 * @author  Jeffrey (Zhiwei Wang)
 * @date    Oct 28, 2024
 */

// \callgraph
#pragma once

#include <unordered_map>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/base/debug.h>
#include "gtsam_unstable/dllexport.h"

namespace gtsam {

/**
 * This class provides helper functions for marginalizing variables from a Bayes Tree.
 */
template <typename BayesTree>
class GTSAM_UNSTABLE_EXPORT BayesTreeMarginalizationHelper {

public:
  using Clique = typename BayesTree::Clique;
  using sharedClique = typename BayesTree::sharedClique;

  /**
   * This function identifies variables that need to be re-eliminated before
   * performing marginalization.
   * 
   * Re-elimination is necessary for a clique containing marginalizable
   * variables if:
   * 
   * 1. Some non-marginalizable variables appear before marginalizable ones
   *    in that clique;
   * 2. Or it has a child node depending on a marginalizable variable AND the
   *    subtree rooted at that child contains non-marginalizables.
   * 
   * In addition, the subtrees under the aforementioned cliques that require
   * re-elimination, which contain non-marginalizable variables in their root
   * node, also need to be re-eliminated.
   * 
   * @param[in] bayesTree The Bayes tree
   * @param[in] marginalizableKeys Keys to be marginalized
   * @return Set of additional keys that need to be re-eliminated
   */
  static std::set<Key> gatherAdditionalKeysToReEliminate(
      const BayesTree& bayesTree,
      const KeyVector& marginalizableKeys) {
    const bool debug = ISDEBUG("BayesTreeMarginalizationHelper");

    std::set<Key> additionalKeys;
    std::set<Key> marginalizableKeySet(
        marginalizableKeys.begin(), marginalizableKeys.end());
    std::set<sharedClique> dependentSubtrees;
    CachedSearch cachedSearch;

    // Check each clique that contains a marginalizable key
    for (const sharedClique& clique :
         getCliquesContainingKeys(bayesTree, marginalizableKeySet)) {

      if (needsReelimination(clique, marginalizableKeySet, &cachedSearch)) {
        // Add frontal variables from current clique
        addCliqueToKeySet(clique, &additionalKeys);

        // Then gather dependent subtrees to be added later
        gatherDependentSubtrees(
            clique, marginalizableKeySet, &dependentSubtrees, &cachedSearch);
      }
    }

    // Add the remaining dependent cliques
    for (const sharedClique& subtree : dependentSubtrees) {
      addSubtreeToKeySet(subtree, &additionalKeys);
    }

    if (debug) {
      std::cout << "BayesTreeMarginalizationHelper: Additional keys to re-eliminate: ";
      for (const Key& key : additionalKeys) {
        std::cout << DefaultKeyFormatter(key) << " ";
      }
      std::cout << std::endl;
    }

    return additionalKeys;
  }

 protected:

  /**
   * Gather the cliques containing any of the given keys.
   * 
   * @param[in] bayesTree The Bayes tree
   * @param[in] keysOfInterest Set of keys of interest
   * @return Set of cliques that contain any of the given keys
   */
  static std::set<sharedClique> getCliquesContainingKeys(
      const BayesTree& bayesTree,
      const std::set<Key>& keysOfInterest) {
    std::set<sharedClique> cliques;
    for (const Key& key : keysOfInterest) {
      cliques.insert(bayesTree[key]);
    }
    return cliques;
  }

  /**
   * A struct to cache the results of the below two functions.
   */
  struct CachedSearch {
    std::unordered_map<Clique*, bool> wholeMarginalizableCliques;
    std::unordered_map<Clique*, bool> wholeMarginalizableSubtrees;
  };

  /**
   * Check if all variables in the clique are marginalizable.
   * 
   * Note we use a cache map to avoid repeated searches.
   */
  static bool isWholeCliqueMarginalizable(
      const sharedClique& clique,
      const std::set<Key>& marginalizableKeys,
      CachedSearch* cache) {
    auto it = cache->wholeMarginalizableCliques.find(clique.get());
    if (it != cache->wholeMarginalizableCliques.end()) {
      return it->second;
    } else {
      bool ret = true;
      for (Key key : clique->conditional()->frontals()) {
        if (!marginalizableKeys.count(key)) {
          ret = false;
          break;
        }
      }
      cache->wholeMarginalizableCliques.insert({clique.get(), ret});
      return ret;
    }
  }

  /**
   * Check if all variables in the subtree are marginalizable.
   * 
   * Note we use a cache map to avoid repeated searches.
   */
  static bool isWholeSubtreeMarginalizable(
      const sharedClique& subtree,
      const std::set<Key>& marginalizableKeys,
      CachedSearch* cache) {
    auto it = cache->wholeMarginalizableSubtrees.find(subtree.get());
    if (it != cache->wholeMarginalizableSubtrees.end()) {
      return it->second;
    } else {
      bool ret = true;
      if (isWholeCliqueMarginalizable(subtree, marginalizableKeys, cache)) {
        for (const sharedClique& child : subtree->children) {
          if (!isWholeSubtreeMarginalizable(child, marginalizableKeys, cache)) {
            ret = false;
            break;
          }
        }
      } else {
        ret = false;
      }
      cache->wholeMarginalizableSubtrees.insert({subtree.get(), ret});
      return ret;
    }
  }

  /**
   * Check if a clique contains variables that need reelimination due to
   * elimination ordering conflicts.
   * 
   * @param[in] clique The clique to check
   * @param[in] marginalizableKeys Set of keys to be marginalized
   * @return true if any variables in the clique need re-elimination
   */
  static bool needsReelimination(
      const sharedClique& clique,
      const std::set<Key>& marginalizableKeys,
      CachedSearch* cache) {
    bool hasNonMarginalizableAhead = false;

    // Check each frontal variable in order
    for (Key key : clique->conditional()->frontals()) {
      if (marginalizableKeys.count(key)) {
        // If we've seen non-marginalizable variables before this one,
        // we need to reeliminate
        if (hasNonMarginalizableAhead) {
          return true;
        }

        // Check if any child depends on this marginalizable key and the
        // subtree rooted at that child contains non-marginalizables.
        for (const sharedClique& child : clique->children) {
          if (hasDependency(child, key) &&
              !isWholeSubtreeMarginalizable(child, marginalizableKeys, cache)) {
            return true;
          }
        }
      } else {
        hasNonMarginalizableAhead = true;
      }
    }
    return false;
  }

  /**
   * Gather all subtrees that depend on a marginalizable key and contain
   * non-marginalizable variables in their root.
   *
   * @param[in] rootClique The starting clique
   * @param[in] marginalizableKeys Set of keys to be marginalized
   * @param[out] dependentSubtrees Pointer to set storing dependent cliques
   */
  static void gatherDependentSubtrees(
      const sharedClique& rootClique,
      const std::set<Key>& marginalizableKeys,
      std::set<sharedClique>* dependentSubtrees,
      CachedSearch* cache) {
    for (Key key : rootClique->conditional()->frontals()) {
      if (marginalizableKeys.count(key)) {
        // Find children that depend on this key
        for (const sharedClique& child : rootClique->children) {
          if (!dependentSubtrees->count(child) &&
              hasDependency(child, key)) {
            getSubtreesContainingNonMarginalizables(
                child, marginalizableKeys, cache, dependentSubtrees);
          }
        }
      }
    }
  }

  /**
   * Gather all subtrees that contain non-marginalizable variables in its root.
   */
  static void getSubtreesContainingNonMarginalizables(
      const sharedClique& rootClique,
      const std::set<Key>& marginalizableKeys,
      CachedSearch* cache,
      std::set<sharedClique>* subtreesContainingNonMarginalizables) {
    // If the root clique itself contains non-marginalizable variables, we
    // just add it to subtreesContainingNonMarginalizables;    
    if (!isWholeCliqueMarginalizable(rootClique, marginalizableKeys, cache)) {
      subtreesContainingNonMarginalizables->insert(rootClique);
      return;
    }

    // Otherwise, we need to recursively check the children
    for (const sharedClique& child : rootClique->children) {
      getSubtreesContainingNonMarginalizables(
          child, marginalizableKeys, cache,
          subtreesContainingNonMarginalizables);
    }
  }

  /**
   * Add all frontal variables from a clique to a key set.
   * 
   * @param[in] clique Clique to add keys from
   * @param[out] additionalKeys Pointer to the output key set
   */
  static void addCliqueToKeySet(
      const sharedClique& clique,
      std::set<Key>* additionalKeys) {
    for (Key key : clique->conditional()->frontals()) {
      additionalKeys->insert(key);
    }
  }

  /**
   * Add all frontal variables from a subtree to a key set.
   *
   * @param[in] subRoot Root clique of the subtree
   * @param[out] additionalKeys Pointer to the output key set
   */
  static void addSubtreeToKeySet(
      const sharedClique& subRoot,
      std::set<Key>* additionalKeys) {
    std::set<sharedClique> cliques;
    cliques.insert(subRoot);
    while(!cliques.empty()) {
      auto begin = cliques.begin();
      sharedClique clique = *begin;
      cliques.erase(begin);
      addCliqueToKeySet(clique, additionalKeys);
      for (const sharedClique& child : clique->children) {
        cliques.insert(child);
      }
    }
  }

  /**
   * Check if the clique depends on the given key.
   * 
   * @param[in] clique Clique to check
   * @param[in] key Key to check for dependencies
   * @return true if clique depends on the key
   */
  static bool hasDependency(
      const sharedClique& clique, Key key) {
    auto conditional = clique->conditional();
    if (std::find(conditional->beginParents(),
        conditional->endParents(), key)
        != conditional->endParents()) {
      return true;
    } else {
      return false;
    }
  }
};
// BayesTreeMarginalizationHelper

}/// namespace gtsam
