//
// Created by jason on 6/23/25.
//

// Concurrent filtering and smoothing, simulating reading incremental PGO data
// Modify optimize(), to use original LM solver

#include <queue>

#include "examples/City10000.h"
#include "gtsam/geometry/Pose2.h"
#include <gtsam/geometry/Pose3.h>

#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/BatchFixedLagSmoother.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam_unstable/nonlinear/ConcurrentCertifiableBatchSmoother.h"
#include "gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h"
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/ConcurrentIncrementalFilter.h>
#include <gtsam/nonlinear/FixedLagSmoother.h>
using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using gtsam::symbol_shorthand::X;
using Scalar = double;

// Export Pose2 values: .g2o if g2o=true, else TUM-style (.tum)
void ExportPose2(const Values& poses, const std::string& path, bool g2o) {
  if (g2o) {
    // Write g2o: keys must be integers starting at 0
    //        gtsam::writeG2o(poses, path + ".g2o");
  } else {
    std::ofstream file(path + ".tum");
    if (!file) {
      std::cerr << "Error opening " << path << ".tum" << std::endl;
      return;
    }
    // TUM format: timestamp tx ty tz qx qy qz qw
    // Use integer key as timestamp
    size_t id = 0;
    for (auto key : poses.keys()) {
      Pose2 p = poses.at<Pose2>(key);
      double tx = p.x();
      double ty = p.y();
      // embed into 3D: z = 0, quaternion from yaw
      double theta = p.theta();
      // Construct quaternion about Z-axis
      Eigen::AngleAxisd aa(theta, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q(aa);
      file << id << " "
           << tx << " " << ty << " " << 0.0 << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
      id++;
    }
    file.close();
  }
}

// Export Pose3 values: .g2o if g2o=true, else TUM-style (.tum)
void ExportPose3(const Values& poses, const std::string& path, bool g2o) {
  if (g2o) {
    //        gtsam::writeG2o(poses, path + ".g2o");
  } else {
    std::ofstream file(path + ".tum");
    if (!file) {
      std::cerr << "Error opening " << path << ".tum" << std::endl;
      return;
    }
    // TUM format: timestamp tx ty tz qx qy qz qw
    size_t id = 0;
    for (auto key : poses.keys()) {
      Pose3 p = poses.at<Pose3>(key);
      auto t = p.translation();
      Eigen::Quaterniond q(p.rotation().toQuaternion());
      file << id << " "
           << t.x() << " " << t.y() << " " << t.z() << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
      id++;
    }
    file.close();
  }
}

// Convenience wrappers
// Export 2D poses
void ExportValues2D(const Values& poses, const std::string& path, bool g2o) {
  ExportPose2(poses, path, g2o);
}

// Export 3D poses
void ExportValues3D(const Values& poses, const std::string& path, bool g2o) {
  ExportPose3(poses, path, g2o);
}

/**
 * @brief Check that two Values objects share no common keys.
 * @param v1 First Values map
 * @param v2 Second Values map
 * @return true if no intersecting keys; false otherwise
 */
bool noKeyIntersection(const Values& v1, const Values& v2) {
  for (Key key : v1.keys()) {
    if (v2.exists(key)) return false;
  }
  return true;
}

/**
 * @brief Merge filter, smoother, and loop closures into a batch optimizer.
 * @param concurrentFilter The concurrent filter instance
 * @param concurrentSmoother The concurrent smoother instance
 * @param loopClosures Graph of additional loop closure factors
 * @return Refined Values after batch optimization
 */
Values refineWithFilterSmoother(
    ConcurrentIncrementalFilter& concurrentFilter,
    ConcurrentCertifiableBatchSmoother& concurrentSmoother,
    const NonlinearFactorGraph& loopClosures) {
  // 1) Separator-only summary
  NonlinearFactorGraph sepFactors;
  Values sepValues;
  concurrentFilter.getSummarizedFactors(sepFactors, sepValues);

  // 2) Full filter values
  Values allValues = concurrentFilter.getLinearizationPoint();

  // 3) Other keys = allValues \ sepValues
  KeySet otherKeys;
  for (Key k : allValues.keys()) {
    if (!sepValues.exists(k)) otherKeys.insert(k);
  }

  // 4) Extract other values
  Values otherValues;
  for (Key k : otherKeys) otherValues.insert(k, allValues.at(k));

  // 5) Extract alive filter factors on otherKeys
  NonlinearFactorGraph allFactors = concurrentFilter.getFactors();
  NonlinearFactorGraph otherFactors;
  for (const auto& fptr : allFactors) {
    if (!fptr) continue;
    bool onOther = true;
    for (Key k : fptr->keys()) {
      if (!otherKeys.count(k)) { onOther = false; break; }
    }
    if (onOther) otherFactors.push_back(fptr);
  }

  // Use otherFactors as filter summary
  NonlinearFactorGraph filterSummaryFactors = otherFactors;
  Values filterSummaryValues = otherValues;

  // 6) Extract smoother factors and values
  NonlinearFactorGraph smootherFactors = concurrentSmoother.getFactors();
  Values smootherValues = concurrentSmoother.getLinearizationPoint();

  // 7) Combine graphs: filter summary, smoother, then loop closures
  NonlinearFactorGraph combinedGraph = filterSummaryFactors;
  combinedGraph.push_back(smootherFactors);
  combinedGraph.push_back(loopClosures);

  // 8) Check for key intersections
  if (!noKeyIntersection(filterSummaryValues, smootherValues)) {
    throw std::runtime_error("Overlapping keys between filter and smoother values");
  }

  // 9) Build initial estimate
  Values initialEstimate = smootherValues;
  for (const auto& kv : filterSummaryValues) {
    initialEstimate.insert(kv.key, kv.value);
  }

  // 10) Batch optimization
  LevenbergMarquardtParams params;
  LevenbergMarquardtOptimizer optimizer(combinedGraph, initialEstimate, params);
  return optimizer.optimize();
}
// Structure to store pending loop closures
struct PendingLoopClosure {
  Key keyFrom;
  Key keyTo;
  Pose2 measurement;
  SharedNoiseModel noise;
  PendingLoopClosure(Key from, Key to, const Pose2& meas, const SharedNoiseModel& n)
      : keyFrom(from), keyTo(to), measurement(meas), noise(n) {}
};

NonlinearFactorGraph extractAllLoopClosures(
    std::queue<PendingLoopClosure>& pendingLoopClosures) {
  NonlinearFactorGraph loopGraph;
  // Consume all closures in the queue
  while (!pendingLoopClosures.empty()) {
    PendingLoopClosure loop = pendingLoopClosures.front();
    pendingLoopClosures.pop();
    loopGraph.add(
        BetweenFactor<Pose2>(
            loop.keyFrom, loop.keyTo,
            loop.measurement, loop.noise));
  }
  return loopGraph;
}

int main(int argc, char** argv) {
  // Parameters
  const double lag = 20.0;

  // Create the concurrent filter and smoother
  ISAM2Params isamParams;
  isamParams.relinearizeThreshold = 0.01;
  isamParams.relinearizeSkip = 1;
  //    isamParams.factorization = ISAM2Params::Factorization::QR;
  ConcurrentIncrementalFilter concurrentFilter(isamParams);

  LevenbergMarquardtParams lmParams;
//  lmParams.absoluteErrorTol = 1e-10;
//  lmParams.relativeErrorTol = 1e-10;
//  lmParams.maxIterations = 100;
  ConcurrentCertifiableBatchSmoother concurrentSmoother(lmParams);

  // Containers for new data
  NonlinearFactorGraph newFactors;
  Values newValues;
  gtsam::Pose2 prev;
  // Queue for storing pending loop closures
  queue<PendingLoopClosure> pendingLoopClosures;

  // Initialize with prior
  Pose2 priorPose(0, 0, 0);
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  newFactors.addPrior<Pose2>(X(0), priorPose, priorNoise);
  newValues.insert(X(0), priorPose);
  prev = priorPose;
  // Process initial update
  concurrentFilter.update(newFactors, newValues, FastList<Key>());
  synchronize(concurrentFilter, concurrentSmoother);

  // Clear containers
  newFactors.resize(0);
  newValues.clear();
  // Main loop
  size_t index = 0;
  std::vector<Pose2> poseArray;
  std::pair<size_t, size_t> keys;

  // Map original g2o IDs to contiguous indices
  std::unordered_map<size_t, size_t> poseIndex;
  size_t nextPoseIdx = 1;
  poseIndex[0] = 0;

  std::ifstream infile(argv[1]);
  std::string line, token;
  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    ss >> token;
    if (token != "EDGE_SE2") continue;

    // Parse g2o: id1 id2 dx dy dtheta sigmas... (we use information matrix entries)
    size_t id1, id2;
    double dx, dy, dtheta, I11, I12, I13, I22, I23, I33;
    ss >> id1 >> id2 >> dx >> dy >> dtheta
        >> I11 >> I12 >> I13 >> I22 >> I23 >> I33;

    if (!poseIndex.count(id1)) poseIndex[id1] = nextPoseIdx++;
    if (!poseIndex.count(id2)) poseIndex[id2] = nextPoseIdx++;
    size_t i = poseIndex[id1], j = poseIndex[id2];

    // Build information matrix and noise model
    Eigen::Matrix3d info;
    info << I11, I12, I13,
        I12, I22, I23,
        I13, I23, I33;
    auto noise = gtsam::noiseModel::Gaussian::Information(info);
    if (id1 == id2 - 1) {  // Odometry measurement
      gtsam::Pose2 guess = prev.compose(gtsam::Pose2(dx,dy,dtheta));
      prev = guess;
      newValues.insert(X(j), guess);
      newFactors.add(gtsam::BetweenFactor<gtsam::Pose2>(
          X(i), X(j), gtsam::Pose2(dx,dy,dtheta), noise));

      // Update smoother
      // Determine keys to marginalize
      FastList<Key> oldKeys;
      if (id2 > lag) {
        oldKeys.push_back(X(id2 - lag - 1));
      }

      // Update filter and smoothers
      try {
        concurrentFilter.update(newFactors, newValues, oldKeys);
      } catch (const std::exception& e) {
        // Handle update failure and print the factors and keys in the filter
        cerr << "Error during filter update: " << e.what() << endl;
        cerr << "Current factors in the filter:" << endl;

        const NonlinearFactorGraph& factors = concurrentFilter.getFactors();
        factors.print("Factors:");

        cerr << "Current keys in the filter:" << endl;
        const Values& values = concurrentFilter.getLinearizationPoint();
        values.print("Keys:");
        cerr << "Exiting due to filter update failure." << endl;

        ExportValues2D(values, "/home/jason/DPGO/StiefelManifold/results/debug_poses_filter", false);

        auto smoother_values = concurrentSmoother.getLinearizationPoint();
        ExportValues2D(smoother_values, "/home/jason/DPGO/StiefelManifold/results/debug_poses_smoother", false);

        exit(EXIT_FAILURE);

      }
      // Update filter and smoothers
      newFactors.resize(0);
      newValues.clear();

      // Periodic synchronization and loop closure processing
      if (id2 % 100 == 0) {
        // First, check if we can process any pending loop closures
        size_t n = pendingLoopClosures.size();
        for (size_t k = 0; k < n; ++k) {
          PendingLoopClosure loop = pendingLoopClosures.front();
          pendingLoopClosures.pop();

          const Values& smootherValues = concurrentSmoother.getLinearizationPoint();
          if (smootherValues.exists(loop.keyFrom) && smootherValues.exists(loop.keyTo)) {
            // both endpoints present → add to new factors
            newFactors.push_back(
                BetweenFactor<Pose2>(
                    loop.keyFrom, loop.keyTo, loop.measurement, loop.noise));
          } else {
            // one or both endpoints missing → defer for later
            pendingLoopClosures.push(loop);
          }
        }

        // Update smoother with accumulated loop closures if any
        if (newFactors.size() > 0) {
          concurrentSmoother.update<Pose2>(newFactors, Values());
        }

        // Synchronize filter and smoother
        concurrentSmoother.update<Pose2>();
        synchronize(concurrentFilter, concurrentSmoother);

        // Print progress
        cout << "Processed pose " << id2 << ", Pending loops: "
             << pendingLoopClosures.size() << endl;
      }
      // Clear containers for next iteration
      newFactors.resize(0);
      newValues.clear();
      index++;

    } else {  // Loop closure
      // Create loop closure factor but store it for later
      pendingLoopClosures.push(PendingLoopClosure(X(i), X(j), gtsam::Pose2(dx,dy,dtheta), noise));
    }
  }

  // Need a final sync after reading all the data of filter
  concurrentSmoother.update<Pose2>();
  synchronize(concurrentFilter, concurrentSmoother);

  for (size_t i = 0; i < 5; i++){
    concurrentSmoother.update<Pose2>();
  }

  auto result_CFS = concurrentSmoother.calculateEstimate();
  ExportValues2D(result_CFS, "/home/jason/DPGO/StiefelManifold/results/incremental_result_Smoother", false);


  NonlinearFactorGraph loopClosureFactors = extractAllLoopClosures(pendingLoopClosures);
  Values finalEstimate = refineWithFilterSmoother(concurrentFilter, concurrentSmoother, loopClosureFactors);
  ExportValues2D(finalEstimate, "/home/jason/DPGO/StiefelManifold/results/incremental_result_Final", false);

  // Print final statistics
  cout << "\nFinal Statistics:" << endl;
  cout << "Processed poses: " << index << endl;
  cout << "Remaining loop closures: " << pendingLoopClosures.size() << endl;
  cout << "Filter keys: " << concurrentFilter.getLinearizationPoint().size() << endl;
  cout << "Smoother keys: " << concurrentSmoother.getLinearizationPoint().size() << endl;
  cout << "Final keys:" << finalEstimate.size() << endl;

  return 0;
}