/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   City10000.h
 * @brief  Class for City10000 dataset
 * @author Varun Agrawal
 * @date   February 3, 2025
 */

#include <gtsam/geometry/Pose2.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>

using namespace gtsam;
using namespace boost::algorithm;

class City10000Dataset {
  std::ifstream in_;

 public:
  City10000Dataset(const std::string& filename) : in_(filename) {
    if (!in_.is_open()) {
      std::cerr << "Failed to open file: " << filename << std::endl;
    }
  }

  /// Parse line from file
  std::pair<std::vector<Pose2>, std::pair<size_t, size_t>> parseLine(
      const std::string& line) const {
    std::vector<std::string> parts;
    split(parts, line, is_any_of(" "));

    size_t keyS = stoi(parts[1]);
    size_t keyT = stoi(parts[3]);

    int numMeasurements = stoi(parts[5]);
    std::vector<Pose2> poseArray(numMeasurements);
    for (int i = 0; i < numMeasurements; ++i) {
      double x = stod(parts[6 + 3 * i]);
      double y = stod(parts[7 + 3 * i]);
      double rad = stod(parts[8 + 3 * i]);
      poseArray[i] = Pose2(x, y, rad);
    }
    return {poseArray, {keyS, keyT}};
  }

  /// Read and parse the next line.
  bool next(std::vector<Pose2>* poseArray, std::pair<size_t, size_t>* keys) {
    std::string line;
    if (getline(in_, line)) {
      std::tie(*poseArray, *keys) = parseLine(line);
      return true;
    } else
      return false;
  }
};