# Nonlinear

The `nonlinear` module in GTSAM includes a comprehensive set of tools for nonlinear optimization using factor graphs. Here's an overview of key components organized by category:

## Core Classes

- **[NonlinearFactorGraph](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearFactorGraph.h)**: Represents the optimization problem as a graph of factors.
- **[NonlinearFactor](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearFactor.h)**: Base class for all nonlinear factors.
- **[NoiseModelFactor](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NoiseModelFactor.h)**: Base class for factors with noise models.
- **[Values](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/Values.h)**: Container for variable assignments used in optimization.

## Batch Optimizers

- **[NonlinearOptimizer](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearOptimizer.h)**: Base class for all batch optimizers.
    - **[NonlinearOptimizerParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearOptimizerParams.h)**: Base parameters class for all optimizers.

- **[GaussNewtonOptimizer](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GaussNewtonOptimizer.h)**: Implements Gauss-Newton optimization.
    - **[GaussNewtonParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GaussNewtonParams.h)**: Parameters for Gauss-Newton optimization.

- **[LevenbergMarquardtOptimizer](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/LevenbergMarquardtOptimizer.h)**: Implements Levenberg-Marquardt optimization.
    - **[LevenbergMarquardtParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/LevenbergMarquardtParams.h)**: Parameters for Levenberg-Marquardt optimization.

- **[DoglegOptimizer](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/DoglegOptimizer.h)**: Implements Powell's Dogleg optimization.
    - **[DoglegParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/DoglegParams.h)**: Parameters for Dogleg optimization.

- **[GncOptimizer](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GncOptimizer.h)**: Implements robust optimization using Graduated Non-Convexity.
    - **[GncParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GncParams.h)**: Parameters for Graduated Non-Convexity optimization.

## Incremental Optimizers

- **[ISAM2](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/ISAM2.h)**: Incremental Smoothing and Mapping 2, with fluid relinearization.
    - **[ISAM2Params](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/ISAM2Params.h)**: Parameters controlling the ISAM2 algorithm.
    - **[ISAM2Result](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/ISAM2Result.h)**: Results from ISAM2 update operations.
- **[NonlinearISAM](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearISAM.h)**: Original iSAM implementation (mostly superseded by ISAM2).

## Specialized Factors

- **[PriorFactor](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/PriorFactor.h)**: Imposes a prior constraint on a variable.
- **[NonlinearEquality](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearEquality.h)**: Enforces equality constraints between variables.
- **[LinearContainerFactor](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/LinearContainerFactor.h)**: Wraps linear factors for inclusion in nonlinear factor graphs.

## Filtering and Smoothing

- **[ExtendedKalmanFilter](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/ExtendedKalmanFilter.h)**: Nonlinear Kalman filter implementation.
- **[FixedLagSmoother](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/FixedLagSmoother.h)**: Base class for fixed-lag smoothers.
- **[BatchFixedLagSmoother](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/BatchFixedLagSmoother.h)**: Implementation of a fixed-lag smoother using batch optimization.

## Analysis and Visualization

- **[Marginals](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/Marginals.h)**: Computes marginal covariances from optimization results.
- **[GraphvizFormatting](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GraphvizFormatting.h)**: Provides customization for factor graph visualization.