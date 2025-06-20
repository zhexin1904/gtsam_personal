//
// Created by jason on 6/20/25.
//
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
 * @file ConcurrentFilteringAndSmoothingExample.cpp
 * @brief Demonstration of the concurrent filtering and smoothing architecture using
 * a planar robot example and multiple odometry-like sensors
 * @author Stephen Williams
  */

 /**
 * A simple 2D pose slam example with multiple odometry-like measurements
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - The robot moves forward at 2m/s
 *  - We have measurements between each pose from multiple odometry sensors
  */

 // This example demonstrates the use of the Concurrent Filtering and Smoothing architecture in GTSAM unstable
#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
#include <gtsam_unstable/nonlinear/ConcurrentBatchSmoother.h>

 // We will compare the results to a similar Fixed-Lag Smoother
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>

 // In GTSAM, measurement functions are represented as 'factors'. Several common factors
 // have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
 // Here we will use Between factors for the relative motion described by odometry measurements.
 // Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

 // When the factors are created, we will add them to a Factor Graph. As the factors we are using
 // are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

 // The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
 // nonlinear functions around an initial linearization point, then solve the linear system
 // to update the linearization point. This happens repeatedly until the solver converges
 // to a consistent set of variable values. This requires us to specify an initial guess
 // for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

 // We will use simple integer Keys to uniquely identify each robot pose.
#include <gtsam/inference/Key.h>

 // We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>
#include <iomanip>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/ConcurrentIncrementalFilter.h>
using namespace std;
 using namespace gtsam;

 /**
 * @brief Check that two Values objects share no common keys.
 * @param v1 First Values map
 * @param v2 Second Values map
 * @return true if no intersecting keys; false otherwise
  */
 bool noKeyIntersection(const Values& v1, const Values& v2) {
   KeyVector keys1 = v1.keys();
   for (Key key : keys1) {
     if (v2.exists(key)) {
       return false;
     }
   }
   return true;
 }
 int main(int argc, char** argv) {

   // Define the smoother lag (in seconds)
   double lag = 2.0;

   // Create a Concurrent Filter and Smoother
   //  ConcurrentBatchFilter concurrentFilter;
   ConcurrentIncrementalFilter concurrentFilter;

   ConcurrentBatchSmoother concurrentSmoother;

   // And a fixed lag smoother with a short lag
   BatchFixedLagSmoother fixedlagSmoother(lag);

   // And a fixed lag smoother with very long lag (i.e. a full batch smoother)
   BatchFixedLagSmoother batchSmoother(1000.0);


   // Create containers to store the factors and linearization points that
   // will be sent to the smoothers
   NonlinearFactorGraph newFactors;
   Values newValues;
   FixedLagSmoother::KeyTimestampMap newTimestamps;

   // Create a prior on the first pose, placing it at the origin
   Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
   auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
   Key priorKey = 0;
   newFactors.addPrior(priorKey, priorMean, priorNoise);
   newValues.insert(priorKey, priorMean); // Initialize the first pose at the mean of the prior
   newTimestamps[priorKey] = 0.0; // Set the timestamp associated with this key to 0.0 seconds;

   // Now, loop through several time steps, creating factors from different "sensors"
   // and adding them to the fixed-lag smoothers
   double deltaT = 0.25;
   for(double time = 0.0+deltaT; time <= 5.0; time += deltaT) {

     // Define the keys related to this timestamp
     Key previousKey(1000 * (time-deltaT));
     Key currentKey(1000 * (time));

     // Assign the current key to the current timestamp
     newTimestamps[currentKey] = time;

     // Add a guess for this pose to the new values
     // Since the robot moves forward at 2 m/s, then the position is simply: time[s]*2.0[m/s]
     // {This is not a particularly good way to guess, but this is just an example}
     Pose2 currentPose(time * 2.0, 0.0, 0.0);
     newValues.insert(currentKey, currentPose);

     // Add odometry factors from two different sources with different error stats
     Pose2 odometryMeasurement1 = Pose2(0.61, -0.08, 0.02);
     auto odometryNoise1 = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
     newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement1, odometryNoise1));

     Pose2 odometryMeasurement2 = Pose2(0.47, 0.03, 0.01);
     auto odometryNoise2 = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
     newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement2, odometryNoise2));

     // Unlike the fixed-lag versions, the concurrent filter implementation
     // requires the user to supply the specify which keys to move to the smoother
     FastList<Key> oldKeys;
     if(time >= lag+deltaT) {
       oldKeys.push_back(1000 * (time-lag-deltaT));
     }

     // Update the various inference engines
     concurrentFilter.update(newFactors, newValues, oldKeys);
     fixedlagSmoother.update(newFactors, newValues, newTimestamps);
     batchSmoother.update(newFactors, newValues, newTimestamps);

     // Manually synchronize the Concurrent Filter and Smoother every 1.0 s
     if(fmod(time, 1.0) < 0.01) {
       // Synchronize the Filter and Smoother
       concurrentSmoother.update();
       synchronize(concurrentFilter, concurrentSmoother);
     }

     // Clear contains for the next iteration
     newTimestamps.clear();
     newValues.clear();
     newFactors.resize(0);
   }

   // At the moment, all three versions produce the same output.
   // Now lets create a "loop closure" factor between the first pose and the current pose
   Key loopKey1(1000 * (0.0));
   Key loopKey2(1000 * (5.0));
   Pose2 loopMeasurement = Pose2(9.5, 1.00, 0.00);
   auto loopNoise = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.25));
   NonlinearFactor::shared_ptr loopFactor(new BetweenFactor<Pose2>(loopKey1, loopKey2, loopMeasurement, loopNoise));

   // This measurement cannot be added directly to the concurrent filter because it connects a filter state to a smoother state
   // This measurement can never be added to the fixed-lag smoother, as one of the poses has been permanently marginalized out
   // This measurement can be incorporated into the full batch version though
   newFactors.push_back(loopFactor);
   batchSmoother.update(newFactors, Values(), FixedLagSmoother::KeyTimestampMap());
   newFactors.resize(0);



   // Continue adding odometry factors until the loop closure may be incorporated into the concurrent smoother
   for(double time = 5.0+deltaT; time <= 8.0; time += deltaT) {

     // Define the keys related to this timestamp
     Key previousKey(1000 * (time-deltaT));
     Key currentKey(1000 * (time));

     // Assign the current key to the current timestamp
     newTimestamps[currentKey] = time;

     // Add a guess for this pose to the new values
     // Since the robot moves forward at 2 m/s, then the position is simply: time[s]*2.0[m/s]
     // {This is not a particularly good way to guess, but this is just an example}
     Pose2 currentPose(time * 2.0, 0.0, 0.0);
     newValues.insert(currentKey, currentPose);

     // Add odometry factors from two different sources with different error stats
     Pose2 odometryMeasurement1 = Pose2(0.61, -0.08, 0.02);
     auto odometryNoise1 = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
     newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement1, odometryNoise1));

     Pose2 odometryMeasurement2 = Pose2(0.47, 0.03, 0.01);
     auto odometryNoise2 = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
     newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement2, odometryNoise2));

     // Unlike the fixed-lag versions, the concurrent filter implementation
     // requires the user to supply the specify which keys to marginalize
     FastList<Key> oldKeys;
     if(time >= lag+deltaT) {
       oldKeys.push_back(1000 * (time-lag-deltaT));
     }

     // Update the various inference engines
     concurrentFilter.update(newFactors, newValues, oldKeys);
     fixedlagSmoother.update(newFactors, newValues, newTimestamps);
     batchSmoother.update(newFactors, newValues, newTimestamps);

     // Manually synchronize the Concurrent Filter and Smoother every 1.0 s
     if(fmod(time, 1.0) < 0.01) {
       // Synchronize the Filter and Smoother
       concurrentSmoother.update();
       synchronize(concurrentFilter, concurrentSmoother);
     }

     // Print the optimized current pose
     cout << endl;

     // Clear contains for the next iteration
     newTimestamps.clear();
     newValues.clear();
     newFactors.resize(0);
   }

   // The state at 5.0s should have been transferred to the concurrent smoother at this point. Add the loop closure.
   newFactors.push_back(loopFactor);
   concurrentSmoother.update(newFactors, Values());
   newFactors.resize(0);


   // Now run for a few more seconds so the concurrent smoother and filter have to to re-sync
   // Continue adding odometry factors until the loop closure may be incorporated into the concurrent smoother
   for(double time = 8.0+deltaT; time <= 15.0; time += deltaT) {

     // Define the keys related to this timestamp
     Key previousKey(1000 * (time-deltaT));
     Key currentKey(1000 * (time));

     // Assign the current key to the current timestamp
     newTimestamps[currentKey] = time;

     // Add a guess for this pose to the new values
     // Since the robot moves forward at 2 m/s, then the position is simply: time[s]*2.0[m/s]
     // {This is not a particularly good way to guess, but this is just an example}
     Pose2 currentPose(time * 2.0, 0.0, 0.0);
     newValues.insert(currentKey, currentPose);

     // Add odometry factors from two different sources with different error stats
     Pose2 odometryMeasurement1 = Pose2(0.61, -0.08, 0.02);
     auto odometryNoise1 = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
     newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement1, odometryNoise1));

     Pose2 odometryMeasurement2 = Pose2(0.47, 0.03, 0.01);
     auto odometryNoise2 = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
     newFactors.push_back(BetweenFactor<Pose2>(previousKey, currentKey, odometryMeasurement2, odometryNoise2));

     // Unlike the fixed-lag versions, the concurrent filter implementation
     // requires the user to supply the specify which keys to marginalize
     FastList<Key> oldKeys;
     if(time >= lag+deltaT) {
       oldKeys.push_back(1000 * (time-lag-deltaT));
     }

     // Update the various inference engines
     concurrentFilter.update(newFactors, newValues, oldKeys);
     fixedlagSmoother.update(newFactors, newValues, newTimestamps);
     batchSmoother.update(newFactors, newValues, newTimestamps);

     // Manually synchronize the Concurrent Filter and Smoother every 1.0 s
     if(fmod(time, 1.0) < 0.01) {
       // Synchronize the Filter and Smoother
       concurrentSmoother.update();
       synchronize(concurrentFilter, concurrentSmoother);
     }

     // Clear contains for the next iteration
     newTimestamps.clear();
     newValues.clear();
     newFactors.resize(0);
   }


   // And to demonstrate the fixed-lag aspect, print the keys contained in each smoother after 3.0 seconds
//   cout << "After 15.0 seconds, each version contains to the following keys:" << endl;
//   cout << "  Concurrent Filter Keys: " << endl;
//   for(const auto key: concurrentFilter.getLinearizationPoint().keys()) {
//     cout << setprecision(5) << "    Key: " << key << endl;
//   }
//   cout << "  Concurrent Smoother Keys: " << endl;
//   for(const auto key: concurrentSmoother.getLinearizationPoint().keys()) {
//     cout << setprecision(5) << "    Key: " << key << endl;
//   }
//   cout << "  Fixed-Lag Smoother Keys: " << endl;
//   for(const auto& key_timestamp: fixedlagSmoother.timestamps()) {
//     cout << setprecision(5) << "    Key: " << key_timestamp.first << endl;
//   }
//   cout << "  Batch Smoother Keys: " << endl;
//   for(const auto& key_timestamp: batchSmoother.timestamps()) {
//     cout << setprecision(5) << "    Key: " << key_timestamp.first << endl;
//   }

   // 4. Extract filter summary (old states condensed)
   NonlinearFactorGraph filterSummaryFactors_final;
   Values              filterSummaryValues_final;

   NonlinearFactorGraph sepFactors;
   Values               sepValues;
   // 1) Get the separator‐only summary exactly as before
   concurrentFilter.getSummarizedFactors(sepFactors, sepValues);
   // 2) Pull the full ISAM2 linearization point (all current variables)
   Values allValues = concurrentFilter.getLinearizationPoint();
   // 3) Compute the set of “other” keys = allValues.keys \ sepValues.keys
   KeySet otherKeys;
   for (Key k : allValues.keys()) {
     if (!sepValues.exists(k))
       otherKeys.insert(k);
   }
   // 4) Extract those remaining values
   Values otherValues;
   for (Key k : otherKeys)
     otherValues.insert(k, allValues.at(k));
   // 5) Now grab the raw ISAM2 factor list and keep only factors on otherKeys
   NonlinearFactorGraph otherFactors;
//   auto allFactors = concurrentFilter.getFactors();
   NonlinearFactorGraph allFactors = concurrentFilter.getFactors();

   // 2) Keep only the alive (non-null) pointers
   NonlinearFactorGraph aliveFactors;
   aliveFactors.reserve(allFactors.size());
   for (const auto& fptr : allFactors) {
     if (fptr)            // skip nullptr entries
       aliveFactors.push_back(fptr);
   }
   for (const auto& factor : aliveFactors) {
     // check if *all* keys in this factor lie in otherKeys
     bool onOther = true;
     for (Key k : factor->keys()) {
       if (!otherKeys.count(k)) {
         onOther = false;
         break;
       }
     }
     if (onOther)
       otherFactors.push_back(factor);
   }

   filterSummaryValues_final = otherValues;
   filterSummaryFactors_final = otherFactors;

   // 5. Extract smoother factors and values (current window)
   NonlinearFactorGraph smootherFactors_final;
   Values              smootherValues_final;
   smootherValues_final = concurrentSmoother.getLinearizationPoint();
   smootherFactors_final = concurrentSmoother.getFactors();
   // 6. Build combined graph
   NonlinearFactorGraph combinedGraph = filterSummaryFactors_final;
   combinedGraph.push_back(smootherFactors_final);;

   if (!noKeyIntersection(filterSummaryValues_final, smootherValues_final)) {
     std::cout << "ERROR: Filter and Smoother values have overlapping keys" << std::endl;
   }

   // 7. Build initial estimate
   Values initialEstimate = smootherValues_final;
   for (const auto& kv : filterSummaryValues_final) {
     Key key = kv.key;               // not kv.first
     const Value& value = kv.value;  // not kv.second
     if (!initialEstimate.exists(key)) {
       initialEstimate.insert(key, value);
     }
   }

   // Then run batch optimization as usual:
   LevenbergMarquardtParams params;
   Values finalEstimate = LevenbergMarquardtOptimizer(
                              combinedGraph, initialEstimate, params).optimize();

   cout << "  Combined Keys: " << endl;
   cout << "Size: " << finalEstimate.keys().size() << endl;
   for(const auto key: finalEstimate.keys()) {
     cout << setprecision(5) << "    Key: " << key << endl;
   }
   cout << "  Concurrent Filter Keys: " << endl;
   cout << "Size: " << concurrentFilter.getLinearizationPoint().keys().size() << endl;
   for(const auto key: concurrentFilter.getLinearizationPoint().keys()) {
     cout << setprecision(5) << "    Key: " << key << endl;
   }
   cout << "  Concurrent Smoother Keys: " << endl;
   cout << "Size: " << concurrentSmoother.getLinearizationPoint().keys().size() << endl;
   for(const auto key: concurrentSmoother.getLinearizationPoint().keys()) {
     cout << setprecision(5) << "    Key: " << key << endl;
   }



   return 0;
 }
