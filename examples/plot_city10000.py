"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Script to plot City10000 results.
Can be used to plot results from both C++ and python scripts.

python plot_city10000.py

Author: Varun Agrawal
"""

import numpy as np
from matplotlib import pyplot as plt


def plot_estimates(gt,
                   estimates,
                   fignum: int,
                   estimate_color=(0.1, 0.1, 0.9, 0.4),
                   estimate_label="Hybrid Factor Graphs"):
    fig = plt.figure(fignum)
    ax = fig.gca()
    ax.axis('equal')
    ax.axis((-65.0, 65.0, -75.0, 60.0))
    ax.plot(gt[:, 0],
            gt[:, 1],
            '--',
            linewidth=1,
            color=(0.1, 0.7, 0.1, 0.5),
            label="Ground Truth")
    ax.plot(estimates[:, 0],
            estimates[:, 1],
            '-',
            linewidth=1,
            color=estimate_color,
            label=estimate_label)
    ax.legend()


def main():
    gt = np.loadtxt('Data/ISAM2_GT_city10000.txt', delimiter=" ")

    # Generate by running `make ISAM2_City10000.run`
    eh_poses = np.loadtxt('../build/examples/ISAM2_city10000.txt',
                          delimiter=" ")

    # Generate by running `make Hybrid_City10000.run`
    h_poses = np.loadtxt('../build/examples/Hybrid_City10000.txt',
                         delimiter=" ")

    # Plot the same number of GT poses as estimated ones
    gt = gt[:h_poses.shape[0], :]
    eh_poses = eh_poses[:h_poses.shape[0], :]

    plot_estimates(gt,
                   h_poses,
                   1,
                   estimate_color=(0.1, 0.1, 0.9, 0.4),
                   estimate_label="Hybrid Factor Graphs")
    plot_estimates(gt,
                   eh_poses,
                   2,
                   estimate_color=(0.9, 0.1, 0.1, 0.4),
                   estimate_label="ISAM2")

    plt.show()


if __name__ == "__main__":
    main()
