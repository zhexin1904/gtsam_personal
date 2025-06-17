/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Incremental_CFS.cpp
 * @brief Incremental Concurrent Filtering and Smoothing with proper loop closure handling
 */

#include <queue>

#include "examples/City10000.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/BatchFixedLagSmoother.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam_unstable/nonlinear/ConcurrentBatchFilter.h"
#include "gtsam_unstable/nonlinear/ConcurrentBatchSmoother.h"
#include "gtsam_unstable/nonlinear/ConcurrentIncrementalFilter.h"
#include "gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>
#include <vector>


using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

// Structure to store pending loop closures
struct PendingLoopClosure {
    Key keyFrom;
    Key keyTo;
    Pose2 measurement;
    SharedNoiseModel noise;

    PendingLoopClosure(Key from, Key to, const Pose2& meas, const SharedNoiseModel& n)
        : keyFrom(from), keyTo(to), measurement(meas), noise(n) {}
};

int main(int argc, char** argv) {
  // Parameters
  const double lag = 10.0;
  const size_t maxLoopCount = 10000;
  LevenbergMarquardtParams params;
  // Print one line per iteration: iteration number, error, lambda, etc.
  params.setVerbosity("SUMMARY");
  // Also print details of each attempted λ‐adjustment
  params.setVerbosityLM("TRYLAMBDA");

  // (Optional) tweak other settings:
  params.setMaxIterations(50);
  params.setRelativeErrorTol(1e-3);
  params.setAbsoluteErrorTol(1e-3);
  // Create the concurrent filter and smoother
//  ConcurrentBatchFilter concurrentFilter(params);
  ConcurrentIncrementalFilter concurrentFilter;
  ConcurrentBatchSmoother concurrentSmoother;
//  BatchFixedLagSmoother batchSmoother(1000.0);

  // Test on incremental fix lag smoother
  ISAM2Params isamParameters;
//  isamParameters.relinearizeThreshold = 0.1;
//  isamParameters.relinearizeSkip = 1;
  // Important!!!!!! Key parameter to ensure old factors are released after marginalization
  isamParameters.findUnusedFactorSlots = true;
  // Initialize fixed-lag smoother with a 1-second lag window
  IncrementalFixedLagSmoother FixLagSmoother(lag, isamParameters);

  // Load dataset
  City10000Dataset dataset(findExampleDataFile(
      "/home/jason/DPGO/gtsam_personal/examples/Data/T1_city10000_04.txt"));

  // Containers for new data
  NonlinearFactorGraph newFactors;
  Values newValues;
  FixedLagSmoother::KeyTimestampMap newTimestamps;

  // Queue for storing pending loop closures
  queue<PendingLoopClosure> pendingLoopClosures;

  // Initialize with prior
  Pose2 priorPose(0, 0, 0);
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  newFactors.addPrior<Pose2>(X(0), priorPose, priorNoise);
  newValues.insert(X(0), priorPose);
  newTimestamps[X(0)] = 0.0;

  // Main loop
  size_t index = 0;
  std::vector<Pose2> poseArray;
  std::pair<size_t, size_t> keys;

  while (dataset.next(&poseArray, &keys) && index < maxLoopCount) {
    size_t keyS = keys.first;
    size_t keyT = keys.second;
    Pose2 odomPose = poseArray[0];

//    if (keyT % 10 == 0) {
//      concurrentSmoother.update();
//      synchronize(concurrentFilter, concurrentSmoother);
//    }

    if (keyS == keyT - 1) {  // Sequential measurement

      // Periodic synchronization
      if (keyT % 100 == 0) {
        const Values& smootherValues = concurrentSmoother.getLinearizationPoint();
        // Update the exsisting loop closure firstly
        // Process pending queue
        size_t size = pendingLoopClosures.size();
        for (size_t i = 0; i < size; ++i) {
          auto& pendingLoop = pendingLoopClosures.front();
          if (smootherValues.exists(pendingLoop.keyFrom) &&
              smootherValues.exists(pendingLoop.keyTo)) {
            // Add to the smoother if valid
            newFactors.push_back(BetweenFactor<Pose2>(
                pendingLoop.keyFrom, pendingLoop.keyTo,
                pendingLoop.measurement, pendingLoop.noise));
            cout << "Processed queued loop closure between poses "
                 << pendingLoop.keyFrom << " and " << pendingLoop.keyTo << endl;
          } else {
            // If not valid, re-add it to the back of the queue
            pendingLoopClosures.push(pendingLoop);
          }
          // Remove the current element from the front
          pendingLoopClosures.pop();
        }

        if (!newFactors.empty()) {
          concurrentSmoother.update(newFactors, Values());
          synchronize(concurrentFilter, concurrentSmoother);
          newFactors.resize(0);
        }
        else {
          concurrentSmoother.update();
          synchronize(concurrentFilter, concurrentSmoother);
        }

      }

      // Add new values
      newValues.insert(X(keyT), odomPose);
      newTimestamps[X(keyT)] = double(keyT);

      // Add odometry factor
      auto odomNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
      newFactors.push_back(
          BetweenFactor<Pose2>(X(keyS), X(keyT), odomPose, odomNoise));

      // Determine keys to marginalize
      FastList<Key> oldKeys;
      if (keyT > lag) {
        oldKeys.push_back(X(keyT - lag));
      }

      // Update filter and smoothers
      concurrentFilter.update(newFactors, newValues, oldKeys);

      newFactors.resize(0);
      newValues.clear();
      newTimestamps.clear();
    } else {  // Loop closure detected
      const Values& smootherValues = concurrentSmoother.getLinearizationPoint();
      // Create loop closure factor but don't add it immediately
      auto loopNoise = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.25));
      const PendingLoopClosure loop(X(keyS), X(keyT), odomPose, loopNoise);

      // Update if in smoother, else queue
//      const Values& smootherValues = concurrentSmoother.getLinearizationPoint();
      if (smootherValues.exists(loop.keyFrom) &&
          smootherValues.exists(loop.keyTo)) {
        NonlinearFactorGraph loopFactors;
        loopFactors.push_back(BetweenFactor<Pose2>(
            loop.keyFrom, loop.keyTo, loop.measurement, loop.noise));
        concurrentSmoother.update(loopFactors, Values());
        cout << "Loop closure added between poses " << keyS << " and " << keyT
             << endl;
      } else {
        pendingLoopClosures.push(loop);
        cout << "Loop closure queued between poses " << keyS << " and " << keyT
             << " (waiting for variables)" << endl;
      }
      index++;
    }
  }
}