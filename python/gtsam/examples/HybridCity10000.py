"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Script for running hybrid estimator on the City10000 dataset.

Author: Varun Agrawal
"""

import argparse
import time

import numpy as np
from gtsam.symbol_shorthand import L, M, X

import gtsam
from gtsam import (BetweenFactorPose2, HybridNonlinearFactor,
                   HybridNonlinearFactorGraph, HybridSmoother, HybridValues,
                   Pose2, PriorFactorPose2, Values)


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser()
    parser.add_argument("data_file",
                        help="The path to the City10000 data file",
                        default="T1_city10000_04.txt")
    return parser.parse_args()


# Noise models
open_loop_model = gtsam.noiseModel.Diagonal.Sigmas(np.ones(3) * 10)
open_loop_constant = open_loop_model.negLogConstant()

prior_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
    np.asarray([0.0001, 0.0001, 0.0001]))

pose_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
    np.asarray([1.0 / 30.0, 1.0 / 30.0, 1.0 / 100.0]))
pose_noise_constant = pose_noise_model.negLogConstant()


class City10000Dataset:
    """Class representing the City10000 dataset."""

    def __init__(self, filename):
        self.filename_ = filename
        try:
            f = open(self.filename_, 'r')
            f.close()
        except OSError:
            print(f"Failed to open file: {self.filename_}")

    def read_line(self, line: str, delimiter: str = " "):
        """Read a `line` from the dataset, separated by the `delimiter`."""
        return line.split(delimiter)

    def parse_line(self, line: str) -> tuple[list[Pose2], tuple[int, int]]:
        """Parse line from file"""
        parts = self.read_line(line)

        key_s = int(parts[1])
        key_t = int(parts[3])

        num_measurements = int(parts[5])
        pose_array = [Pose2()] * num_measurements

        for i in range(num_measurements):
            x = float(parts[6 + 3 * i])
            y = float(parts[7 + 3 * i])
            rad = float(parts[8 + 3 * i])
            pose_array[i] = Pose2(x, y, rad)

        return pose_array, (key_s, key_t)

    def next(self):
        """Read and parse the next line."""
        with open(self.filename_, 'w') as f:
            line = f.readline()
            if line:
                yield self.parse_line(line)
            else:
                yield None, None


def main():
    """Main runner"""
    args = parse_arguments()


if __name__ == "__main__":
    main()
