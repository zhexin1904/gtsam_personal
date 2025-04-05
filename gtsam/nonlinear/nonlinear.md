# Nonlinear

The `nonlinear` module in GTSAM focuses on solving nonlinear optimization problems using factor graphs and incremental solvers. 

Key Concepts:
- **Nonlinear Factors**: Represent constraints or measurements in a nonlinear optimization problem.
- **Factor Graphs**: A graphical representation of the optimization problem.
- **Nonlinear Solvers**: Various optimization methods, typically calling linear solves in `linear`.

## Basics
- **`NonlinearFactor.h`**: Defines the base classes for nonlinear factors, `NonlinearFactor` and `NoiseModelFactor`.
- **`NonlinearFactorGraph.h`**: Implements a factor graph consisting of nonlinear factors.
- **`Values.h`**: Stores variable assignments for optimization.

## Optimizers:
- **`GaussNewtonOptimizer.h`**: Implements the Gauss-Newton optimization algorithm.
- **`LevenbergMarquardtOptimizer.h`**: Provides the Levenberg-Marquardt optimization algorithm.
- **`DoglegOptimizer.h`**: Implements the Dogleg optimization algorithm.

## Incremental Optimizers:
- **`ISAM2.h`**: Implements the iSAM2 incremental solver.
