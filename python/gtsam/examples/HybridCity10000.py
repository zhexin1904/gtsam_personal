"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Script for running hybrid estimator on the City10000 dataset.

Author: Varun Agrawal
"""

import argparse

import numpy as np

import gtsam


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


def main():
    """Main runner"""
    args = parse_arguments()


if __name__ == "__main__":
    main()
