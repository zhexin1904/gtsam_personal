/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DCSAM.cpp
 * @brief   Discrete-Continuous Smoothing and Mapping for Factored Models
 * @author  Kevin Doherty
 * @author  Varun Agrawal
 * @date    March 2025
 */

#include <gtsam/hybrid/DCSAM.h>
#include <gtsam/hybrid/DiscreteBoundaryFactor.h>

namespace gtsam {

DCSAM::DCSAM() {
  // Setup isam
  ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_params.setOptimizationParams(ISAM2DoglegParams());
  isam_ = ISAM2(isam_params);
}

DCSAM::DCSAM(const ISAM2Params &isam_params) : isam_(ISAM2(isam_params)) {}

void DCSAM::update(const HybridNonlinearFactorGraph &hnfg,
                   const HybridValues &initialGuess) {
  // First things first: combine currContinuous_ estimate with
  // the new continuous values from initialGuess to
  // produce the full continuous variable state.
  currContinuous_.insert_or_assign(initialGuess.nonlinear());

  // Also combine currDiscrete_ estimate with new discrete values from
  // initialGuess to give a full discrete variable state.
  for (const auto &kv : initialGuess.discrete()) {
    // This will update the element with key `kv.first` if one exists, or add a
    // new element with key `kv.first` if not.
    currDiscrete_[kv.first] = initialGuess.discrete().at(kv.first);
  }

  // We'll combine the nonlinear factors with DCContinuous factors before
  // passing to the continuous solver; likewise for the discrete factors and
  // DCDiscreteFactors.
  NonlinearFactorGraph continuousCombined;
  DiscreteFactorGraph discreteCombined;

  // Each hybrid factor will be split into a separate discrete
  // and continuous component
  for (auto &factor : hnfg) {
    if (auto hybrid_factor = std::dynamic_pointer_cast<HybridFactor>(factor)) {
      if (auto nonlinear_mixture =
              std::dynamic_pointer_cast<HybridNonlinearFactor>(hybrid_factor)) {
        hfg_.push_back(nonlinear_mixture);
      }

    } else if (auto cont_factor =
                   std::dynamic_pointer_cast<NonlinearFactor>(factor)) {
      continuousCombined.push_back(cont_factor);

    } else if (auto discrete_factor =
                   std::dynamic_pointer_cast<DiscreteFactor>(factor)) {
      discreteCombined.push_back(discrete_factor);
    }
  }

  // Record the discrete factors.
  updateDiscrete(discreteCombined);

  // Update current discrete state estimate.
  if (!initialGuess.nonlinear().empty() && initialGuess.discrete().empty() &&
      discreteCombined.empty()) {
  } else {
    currDiscrete_ = solveDiscrete();
  }

  // Only the initialGuess needs to be provided for the continuous solver
  // (not the entire continuous state).
  updateContinuousInfo(currDiscrete_, continuousCombined,
                       initialGuess.nonlinear());

  currContinuous_ = isam_.calculateEstimate();
  // Update discrete info from last solve and
  updateDiscreteInfo(currContinuous_, currDiscrete_);
}

void DCSAM::update() { update(HybridNonlinearFactorGraph()); }

void DCSAM::updateDiscrete(const DiscreteFactorGraph &dfg,
                           const DiscreteValues &discreteVals) {
  for (auto &factor : dfg) {
    dfg_.push_back(factor);
  }
  updateDiscreteInfo(currContinuous_, discreteVals);
}

void DCSAM::updateDiscreteInfo(const Values &continuousVals,
                               const DiscreteValues &discreteVals) {
  for (const auto &kv : discreteVals) {
    // This will update the element with key `kv.first` if one exists, or add a
    // new element with key `kv.first` if not.
    currDiscrete_[kv.first] = discreteVals.at(kv.first);
  }
}

void DCSAM::updateContinuous() {
  isam_.update();
  currContinuous_ = isam_.calculateEstimate();
}

void DCSAM::updateContinuousInfo(const DiscreteValues &discreteVals,
                                 const NonlinearFactorGraph &newFactors,
                                 const Values &initialGuess) {
  // Initialize continuous factors
  NonlinearFactorGraph graph(newFactors);

  // Given the best discrete values estimate,
  // we get the corresponding continuous factors.
  for (auto &factor : hfg_) {
    if (auto nonlinear_mixture =
            std::dynamic_pointer_cast<HybridNonlinearFactor>(factor)) {
      auto sharedContinuous = nonlinear_mixture->factors()(currDiscrete_).first;
      graph.push_back(sharedContinuous);
      // Record the continuous factor
      nfg_.push_back(sharedContinuous);
    }
  }

  // Mark all affected variables, which are the continuous variables
  ISAM2UpdateParams updateParams;
  FastMap<FactorIndex, KeySet> newAffectedKeys;
  for (size_t j = 0; j < nfg_.size(); j++) {
    auto continuousFactor = nfg_[j];
    for (const Key &k : continuousFactor->keys()) {
      newAffectedKeys[j].insert(k);
    }
  }
  updateParams.newAffectedKeys = std::move(newAffectedKeys);

  // Solve for continuous values
  isam_.update(newFactors, initialGuess, updateParams);
}

DiscreteValues DCSAM::solveDiscrete() const {
  // First copy the recorded discrete factors
  DiscreteFactorGraph dfg(dfg_);

  // Next we add the hybrid factors as DiscreteBoundary factors
  // This step ensures we use the latest currContinuous_ values
  // to compute the discrete factors
  for (auto &factor : hfg_) {
    auto nonlinear_mixture =
        std::dynamic_pointer_cast<HybridNonlinearFactor>(factor);
    if (nonlinear_mixture) {
      auto discreteFactor = std::make_shared<DiscreteBoundaryFactor>(
          nonlinear_mixture->discreteKeys(), nonlinear_mixture->factors(),
          currContinuous_);
      dfg.push_back(discreteFactor);
    }
  }

  // Finally we optimize for the MPE
  DiscreteValues discreteVals = dfg.optimize();
  return discreteVals;
}

HybridValues DCSAM::calculateEstimate() const {
  // NOTE: if we have these cached from solves, we could presumably just return
  // the cached values.
  Values continuousVals = isam_.calculateEstimate();
  DiscreteValues discreteVals = dfg_.optimize();
  HybridValues values(VectorValues(), discreteVals, continuousVals);
  return values;
}

// TODO(Varun) Create HybridMarginals
//  // NOTE separate dcmarginals class?
//  DCMarginals DCSAM::getMarginals(const NonlinearFactorGraph &graph,
//                                  const Values &continuousEst,
//                                  const DiscreteFactorGraph &dfg) {
//    return DCMarginals{Marginals(graph, continuousEst),
//                       dcsam::DiscreteMarginalsOrdered(dfg)};
//  }

}  // namespace gtsam
