/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ISAM2_City10000.cpp
 * @brief  Example of using ISAM2 estimation
 *         with multiple odometry measurements.
 * @author Varun Agrawal
 * @date   January 22, 2025
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace boost::algorithm;

using symbol_shorthand::X;

// Experiment Class
class Experiment {
  /// The City10000 dataset
  City10000Dataset dataset_;

 public:
  // Parameters with default values
  size_t maxLoopCount = 2000;  // 200 //2000 //8000

  // false: run original iSAM2 without ambiguities
  // true: run original iSAM2 with ambiguities
  const bool isWithAmbiguity = false;

 private:
  ISAM2Params parameters;
  parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0);
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;

  ISAM2 isam2(parameters);
  NonlinearFactorGraph graph;
  Values initial_;
  Values results;

 public:
  /// Construct with filename of experiment to run
  explicit Experiment(const std::string& filename) : dataset_(filename) {}

  /// @brief Run the main experiment with a given maxLoopCount.
  void run() {
    // Initialize local variables
    size_t poseCount = 0, index = 0;

    std::list<double> timeList;

    // Set up initial prior
    Pose2 priorPose(0, 0, 0);
    initial_.insert(X(0), priorPose);
    graph.addPrior<Pose2>(X(0), priorPose, kPriorNoiseModel);
    poseCount++;

    // Initial update
    isam2.update(graph, initial_);
    graph.resize(0);
    initial_.clear();
    results = isam2.calculateBestEstimate();

    // Start main loop
    size_t keyS = 0;
    size_t keyT = 0;
    clock_t startTime = clock();

    std::vector<Pose2> poseArray;
    std::pair<size_t, size_t> keys;

    while (dataset_.next(&poseArray, &keys) && index < maxLoopCount) {
      keyS = keys.first;
      keyT = keys.second;
      size_t numMeasurements = poseArray.size();

      Pose2 odomPose;
      if (isWithAmbiguity) {
        // Get wrong intentionally
        int id = index % numMeasurements;
        odomPose = Pose2(poseArray[id]);
      } else {
        odomPose = poseArray[0];
      }

      if (keyS == keyT - 1) {  // new X(key)
        initial_.insert(X(keyT), results.at<Pose2>(X(keyS)) * odomPose);
        graph.add(
            BetweenFactor<Pose2>(X(keyS), X(keyT), odomPose, kPoseNoiseModel));
        poseCount++;

      } else {  // loop
        int id = index % numMeasurements;
        if (isWithAmbiguity && id % 2 == 0) {
          graph.add(BetweenFactor<Pose2>(X(keyS), X(keyT), odomPose,
                                         kPoseNoiseModel));
        } else {
          graph.add(BetweenFactor<Pose2>(
              X(keyS), X(keyT), odomPose,
              noiseModel::Diagonal::Sigmas(Vector3::Ones() * 10.0)));
        }
        index++;
      }

      isam2.update(graph, initial_);
      graph.resize(0);
      initial_.clear();
      results = isam2.calculateBestEstimate();

      // Print loop index and time taken in processor clock ticks
      if (index % 50 == 0 && keyS != keyT - 1) {
        std::cout << "index: " << index << std::endl;
        std::cout << "accTime:  " << timeList.back() / CLOCKS_PER_SEC
                  << std::endl;
      }

      if (keyS == keyT - 1) {
        clock_t curTime = clock();
        timeList.push_back(curTime - startTime);
      }

      if (timeList.size() % 100 == 0 && (keyS == keyT - 1)) {
        string stepFileIdx = std::to_string(100000 + timeList.size());

        ofstream stepOutfile;
        string stepFileName = "step_files/ISAM2_City10000_S" + stepFileIdx;
        stepOutfile.open(stepFileName + ".txt");
        for (size_t i = 0; i < (keyT + 1); ++i) {
          Pose2 outPose = results.at<Pose2>(X(i));
          stepOutfile << outPose.x() << " " << outPose.y() << " "
                      << outPose.theta() << endl;
        }
        stepOutfile.close();
      }
    }

    clock_t endTime = clock();
    clock_t totalTime = endTime - startTime;
    cout << "totalTime: " << totalTime / CLOCKS_PER_SEC << endl;

    /// Write results to file
    writeResult(results, (keyT + 1), "ISAM2_City10000.txt");

    ofstream outfileTime;
    std::string timeFileName = "ISAM2_City10000_time.txt";
    outfileTime.open(timeFileName);
    for (auto accTime : timeList) {
      outfileTime << accTime << std::endl;
    }
    outfileTime.close();
    cout << "Written cumulative time to: " << timeFileName << " file."
         << endl;
  }
};

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  Experiment experiment(findExampleDataFile("T1_City10000_04.txt"));
  // Experiment experiment("../data/mh_T1_City10000_04.txt"); //Type #1 only
  // Experiment experiment("../data/mh_T3b_City10000_10.txt"); //Type #3 only
  // Experiment experiment("../data/mh_T1_T3_City10000_04.txt"); //Type #1 +
  // Type #3

  // Run the experiment
  experiment.run();

  return 0;
}
