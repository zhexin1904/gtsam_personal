"""
  Compare the Fundamental Matrix and Essential Matrix methods for optimizing the view-graph.
  It measures the distance from the ground truth matrices in terms of the norm of local coordinates (geodesic distance) 
  on the F-manifold. It also plots the final error of the optimization.
  Author: Frank Dellaert (with heavy assist from ChatGPT)
  Date: October 2024
"""

import matplotlib.pyplot as plt
import numpy as np
from gtsam.examples import SFMdata

import gtsam
import argparse
from gtsam import (
    Cal3_S2,
    EdgeKey,
    EssentialMatrix,
    FundamentalMatrix,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    NonlinearFactorGraph,
    PinholeCameraCal3_S2,
    Values,
)

# For symbol shorthand (e.g., K(0), K(1))
K_sym = gtsam.symbol_shorthand.K

# Methods to compare
methods = ["FundamentalMatrix", "EssentialMatrix"]


# Formatter function for printing keys
def formatter(key):
    sym = gtsam.Symbol(key)
    if sym.chr() == ord("k"):
        return f"k{sym.index()}"
    else:
        edge = EdgeKey(key)
        return f"({edge.i()},{edge.j()})"


# Function to simulate data
def simulate_data(num_cameras):
    # Define the camera calibration parameters
    K = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)

    # Create the set of 8 ground-truth landmarks
    points = SFMdata.createPoints()

    # Create the set of ground-truth poses
    poses = SFMdata.posesOnCircle(num_cameras, 30)

    # Simulate measurements from each camera pose
    measurements = [[None for _ in range(len(points))] for _ in range(num_cameras)]
    for i in range(num_cameras):
        camera = PinholeCameraCal3_S2(poses[i], K)
        for j in range(len(points)):
            measurements[i][j] = camera.project(points[j])

    return points, poses, measurements, K


# Function to compute ground truth matrices
def compute_ground_truth_matrices(method, poses, K):
    if method == "FundamentalMatrix":
        F1 = FundamentalMatrix(K, poses[0].between(poses[1]), K)
        F2 = FundamentalMatrix(K, poses[0].between(poses[2]), K)
        return F1, F2
    elif method == "EssentialMatrix":
        E1 = EssentialMatrix.FromPose3(poses[0].between(poses[1]))
        E2 = EssentialMatrix.FromPose3(poses[0].between(poses[2]))
        return E1, E2
    else:
        raise ValueError(f"Unknown method {method}")


# Function to build the factor graph
def build_factor_graph(method, num_cameras, measurements):
    graph = NonlinearFactorGraph()

    if method == "FundamentalMatrix":
        # Use TransferFactorFundamentalMatrix
        FactorClass = gtsam.TransferFactorFundamentalMatrix
    elif method == "EssentialMatrix":
        # Use EssentialTransferFactorCal3_S2
        FactorClass = gtsam.EssentialTransferFactorCal3_S2
    else:
        raise ValueError(f"Unknown method {method}")

    for a in range(num_cameras):
        b = (a + 1) % num_cameras  # Next camera
        c = (a + 2) % num_cameras  # Camera after next

        # Vectors to collect tuples for each factor
        tuples1 = []
        tuples2 = []
        tuples3 = []

        # Collect data for the three factors
        for j in range(len(measurements[0])):
            tuples1.append((measurements[a][j], measurements[b][j], measurements[c][j]))
            tuples2.append((measurements[a][j], measurements[c][j], measurements[b][j]))
            tuples3.append((measurements[c][j], measurements[b][j], measurements[a][j]))

        # Add transfer factors between views a, b, and c.
        graph.add(FactorClass(EdgeKey(a, c), EdgeKey(b, c), tuples1))
        graph.add(FactorClass(EdgeKey(a, b), EdgeKey(b, c), tuples2))
        graph.add(FactorClass(EdgeKey(a, c), EdgeKey(a, b), tuples3))

    return graph


# Function to get initial estimates
def get_initial_estimate(method, num_cameras, ground_truth, K):
    initialEstimate = Values()

    if method == "FundamentalMatrix":
        F1, F2 = ground_truth
        for a in range(num_cameras):
            b = (a + 1) % num_cameras  # Next camera
            c = (a + 2) % num_cameras  # Camera after next
            initialEstimate.insert(EdgeKey(a, b).key(), F1)
            initialEstimate.insert(EdgeKey(a, c).key(), F2)
    elif method == "EssentialMatrix":
        E1, E2 = ground_truth
        for a in range(num_cameras):
            b = (a + 1) % num_cameras  # Next camera
            c = (a + 2) % num_cameras  # Camera after next
            initialEstimate.insert(EdgeKey(a, b).key(), E1)
            initialEstimate.insert(EdgeKey(a, c).key(), E2)
        # Insert initial calibrations
        for i in range(num_cameras):
            initialEstimate.insert(K_sym(i), K)
    else:
        raise ValueError(f"Unknown method {method}")

    return initialEstimate


# Function to optimize the graph
def optimize(graph, initialEstimate):
    params = LevenbergMarquardtParams()
    params.setlambdaInitial(1000.0)  # Initialize lambda to a high value
    params.setVerbosityLM("SUMMARY")
    optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, params)
    result = optimizer.optimize()
    return result


# Function to compute distances from ground truth
def compute_distances(method, result, ground_truth, num_cameras, K):
    distances = []

    if method == "FundamentalMatrix":
        F1, F2 = ground_truth
    elif method == "EssentialMatrix":
        E1, E2 = ground_truth
        # Convert ground truth EssentialMatrices to FundamentalMatrices using GTSAM method
        F1 = gtsam.FundamentalMatrix(K, E1, K)
        F2 = gtsam.FundamentalMatrix(K, E2, K)
    else:
        raise ValueError(f"Unknown method {method}")

    for a in range(num_cameras):
        b = (a + 1) % num_cameras
        c = (a + 2) % num_cameras
        key_ab = EdgeKey(a, b).key()
        key_ac = EdgeKey(a, c).key()

        if method == "FundamentalMatrix":
            F_est_ab = result.atFundamentalMatrix(key_ab)
            F_est_ac = result.atFundamentalMatrix(key_ac)
        elif method == "EssentialMatrix":
            E_est_ab = result.atEssentialMatrix(key_ab)
            E_est_ac = result.atEssentialMatrix(key_ac)
            # Convert estimated EssentialMatrices to FundamentalMatrices using GTSAM method
            F_est_ab = gtsam.FundamentalMatrix(K, E_est_ab, K)
            F_est_ac = gtsam.FundamentalMatrix(K, E_est_ac, K)
        else:
            raise ValueError(f"Unknown method {method}")

        # Compute local coordinates (geodesic distance on the F-manifold)
        dist_ab = np.linalg.norm(F1.localCoordinates(F_est_ab))
        dist_ac = np.linalg.norm(F2.localCoordinates(F_est_ac))
        distances.append(dist_ab)
        distances.append(dist_ac)

    return distances


# Function to plot results
def plot_results(results):
    methods = list(results.keys())
    final_errors = [results[method]["final_error"] for method in methods]
    distances = [np.mean(results[method]["distances"]) for method in methods]

    fig, ax1 = plt.subplots()

    color = "tab:red"
    ax1.set_xlabel("Method")
    ax1.set_ylabel("Final Error", color=color)
    ax1.bar(methods, final_errors, color=color, alpha=0.6)
    ax1.tick_params(axis="y", labelcolor=color)

    ax2 = ax1.twinx()
    color = "tab:blue"
    ax2.set_ylabel("Mean Geodesic Distance", color=color)
    ax2.plot(methods, distances, color=color, marker="o")
    ax2.tick_params(axis="y", labelcolor=color)

    plt.title("Comparison of Methods")
    fig.tight_layout()
    plt.show()


# Main function
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Compare Fundamental and Essential Matrix Methods")
    parser.add_argument("--num_cameras", type=int, default=4, help="Number of cameras (default: 4)")
    args = parser.parse_args()

    # Initialize results dictionary
    results = {}

    for method in methods:
        print(f"Running method: {method}")

        # Simulate data
        points, poses, measurements, K = simulate_data(args.num_cameras)

        # Compute ground truth matrices
        ground_truth = compute_ground_truth_matrices(method, poses, K)

        # Build the factor graph
        graph = build_factor_graph(method, args.num_cameras, measurements)

        # Get initial estimates
        initialEstimate = get_initial_estimate(method, args.num_cameras, ground_truth, K)

        # Optimize the graph
        result = optimize(graph, initialEstimate)

        # Compute distances from ground truth
        distances = compute_distances(method, result, ground_truth, args.num_cameras, K)

        # Compute final error
        final_error = graph.error(result)

        # Store results
        results[method] = {"distances": distances, "final_error": final_error}

        print(f"Method: {method}")
        print(f"Final Error: {final_error}")
        print(f"Mean Geodesic Distance: {np.mean(distances)}\n")

    # Plot results
    plot_results(results)


if __name__ == "__main__":
    main()
