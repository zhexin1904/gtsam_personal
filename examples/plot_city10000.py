"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Script to plot City10000 results.
Can be used to plot results from both C++ and python scripts.

Usage:
```
python plot_city10000.py Data/ISAM2_GT_city10000.txt --estimates ../build/examples/ISAM2_city10000.txt ../build/examples/Hybrid_City10000.txt
```

You can generate estimates by running
- `make ISAM2_City10000.run` for the ISAM2 version
- `make Hybrid_City10000.run` for the Hybrid Smoother version

Author: Varun Agrawal
"""

import argparse

import numpy as np
from matplotlib import pyplot as plt


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser()
    parser.add_argument("ground_truth", help="The ground truth data file.")
    parser.add_argument(
        "--estimates",
        nargs='+',
        help="File(s) with estimates (as .txt), can be more than one.")
    return parser.parse_args()


def plot_estimates(gt,
                   estimates,
                   fignum: int,
                   estimate_color=(0.1, 0.1, 0.9, 0.4),
                   estimate_label="Hybrid Factor Graphs"):
    """Plot the City10000 estimates against the ground truth.

    Args:
        gt (np.ndarray): The ground truth trajectory as xy values.
        estimates (np.ndarray): The estimates trajectory as xy values.
        fignum (int): The figure number for multiple plots.
        estimate_color (tuple, optional): The color to use for the graph of estimates.
            Defaults to (0.1, 0.1, 0.9, 0.4).
        estimate_label (str, optional): Label for the estimates, used in the legend.
            Defaults to "Hybrid Factor Graphs".
    """
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
    """Main runner"""
    args = parse_args()
    gt = np.loadtxt(args.ground_truth, delimiter=" ")

    # Default colors and labels, assuming we have only 2 estimates
    colors = ((0.9, 0.1, 0.1, 0.4), (0.1, 0.1, 0.9, 0.4))
    labels = ("ISAM2", "Hybrid Factor Graphs")

    for i in range(len(args.estimates)):
        h_poses = np.loadtxt(args.estimates[i], delimiter=" ")
        gt = gt[:h_poses.shape[0]]

        plot_estimates(gt,
                       h_poses,
                       i + 1,
                       estimate_color=colors[i],
                       estimate_label=labels[i])

    plt.show()


if __name__ == "__main__":
    main()
