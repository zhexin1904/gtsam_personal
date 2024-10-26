"""
  GTSAM Copyright 2010, Georgia Tech Research Corporation,
  Atlanta, Georgia 30332-0415
  All Rights Reserved
  Authors: Frank Dellaert, et al. (see THANKS for the full author list)

  See LICENSE for the license information

  Solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file
  Author: Frank Dellaert (Python: Akshay Krishnan, John Lambert, Varun Agrawal)
"""

"""
Python version of EssentialViewGraphExample.cpp
View-graph calibration with essential matrices.
Author: Frank Dellaert
Date: October 2024
"""

import numpy as np
from gtsam.examples import SFMdata

import gtsam
from gtsam import Cal3_S2, EdgeKey, EssentialMatrix
from gtsam import EssentialTransferFactorCal3_S2 as Factor
from gtsam import (LevenbergMarquardtOptimizer, LevenbergMarquardtParams,
                   NonlinearFactorGraph, PinholeCameraCal3_S2, Point2, Point3,
                   Pose3, Values, symbol_shorthand)

# For symbol shorthand (e.g., X(0), L(1))
K = symbol_shorthand.K


# Formatter function for printing keys
def formatter(key):
    sym = gtsam.Symbol(key)
    if sym.chr() == ord("K"):
        return str(sym)
    elif sym.chr() == ord("E"):
        idx = sym.index()
        a = idx // 10
        b = idx % 10
        return f"E({a},{b})"
    else:
        return str(sym)


def main():
    # Define the camera calibration parameters
    K_initial = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)

    # Create the set of 8 ground-truth landmarks
    points = SFMdata.createPoints()

    # Create the set of 4 ground-truth poses
    poses = SFMdata.posesOnCircle(4, 30)

    # Calculate ground truth essential matrices, 1 and 2 poses apart
    E1 = EssentialMatrix.FromPose3(poses[0].between(poses[1]))
    E2 = EssentialMatrix.FromPose3(poses[0].between(poses[2]))

    # Simulate measurements from each camera pose
    p = [[None for _ in range(8)] for _ in range(4)]
    for i in range(4):
        camera = PinholeCameraCal3_S2(poses[i], K_initial)
        for j in range(8):
            p[i][j] = camera.project(points[j])

    # Create the factor graph
    graph = NonlinearFactorGraph()

    for a in range(4):
        b = (a + 1) % 4  # Next camera
        c = (a + 2) % 4  # Camera after next

        # Collect data for the three factors
        tuples1 = []
        tuples2 = []
        tuples3 = []

        for j in range(8):
            tuples1.append((p[a][j], p[b][j], p[c][j]))
            tuples2.append((p[a][j], p[c][j], p[b][j]))
            tuples3.append((p[c][j], p[b][j], p[a][j]))

        # Add transfer factors between views a, b, and c.
        graph.add(Factor(EdgeKey(a, c), EdgeKey(b, c), tuples1))
        graph.add(Factor(EdgeKey(a, b), EdgeKey(b, c), tuples2))
        graph.add(Factor(EdgeKey(a, c), EdgeKey(a, b), tuples3))

    # Create a delta vector to perturb the ground truth (small perturbation)
    delta = np.ones(5) * 1e-2

    # Create the initial estimate for essential matrices
    initialEstimate = Values()
    for a in range(4):
        b = (a + 1) % 4  # Next camera
        c = (a + 2) % 4  # Camera after next
        initialEstimate.insert(EdgeKey(a, b).key(), E1.retract(delta))
        initialEstimate.insert(EdgeKey(a, c).key(), E2.retract(delta))

    # Insert initial calibrations
    for i in range(4):
        initialEstimate.insert(K(i), K_initial)

    # Optimize the graph and print results
    params = LevenbergMarquardtParams()
    params.setlambdaInitial(1000.0)  # Initialize lambda to a high value
    params.setVerbosityLM("SUMMARY")
    optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, params)
    result = optimizer.optimize()

    print("Initial error = ", graph.error(initialEstimate))
    print("Final error = ", graph.error(result))

    # Print final results
    print("Final Results:")
    result.print("", formatter)

    # Print ground truth essential matrices
    print("Ground Truth E1:\n", E1)
    print("Ground Truth E2:\n", E2)


if __name__ == "__main__":
    main()
