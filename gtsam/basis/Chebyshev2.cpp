/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Chebyshev2.cpp
 * @brief Chebyshev parameterizations on Chebyshev points of second kind
 * @author Varun Agrawal, Jing Dong, Frank Dellaert
 * @date July 4, 2020
 */

#include <gtsam/basis/Chebyshev2.h>
#include <cassert>

namespace gtsam {

double Chebyshev2::Point(size_t N, int j) {
  if (N == 1) return 0.0;
  assert(j >= 0 && size_t(j) < N);
  const double dTheta = M_PI / (N - 1);
  return -cos(dTheta * j);
}

double Chebyshev2::Point(size_t N, int j, double a, double b) {
  if (N == 1) return (a + b) / 2;
  return a + (b - a) * (Point(N, j) + 1.0) / 2.0;
}

Vector Chebyshev2::Points(size_t N) {
  Vector points(N);
  if (N == 1) {
    points(0) = 0.0;
    return points;
  }
  size_t half = N / 2;
  const double dTheta = M_PI / (N - 1);
  for (size_t j = 0; j < half; ++j) {
    double c = cos(j * dTheta);
    points(j) = -c;
    points(N - 1 - j) = c;
  }
  if (N % 2 == 1) {
    points(half) = 0.0;
  }
  return points;
}

Vector Chebyshev2::Points(size_t N, double a, double b) {
  Vector points = Points(N);
  const double T1 = (a + b) / 2, T2 = (b - a) / 2;
  points = T1 + (T2 * points).array();
  return points;
}

namespace {
  // Find the index of the Chebyshev point that coincides with x
  // within the interval [a, b]. If no such point exists, return nullopt.
  static std::optional<size_t> coincidentPoint(size_t N, double x, double a, double b, double tol = 1e-12) {
    if (N == 0) return std::nullopt;
    if (N == 1) {
      double mid = (a + b) / 2;
      if (std::abs(x - mid) < tol) return 0;
    }
    else {
      // Compute normalized value y such that cos(j*dTheta) = y.
      double y = 1.0 - 2.0 * (x - a) / (b - a);
      if (y < -1.0 || y > 1.0) return std::nullopt;
      double dTheta = M_PI / (N - 1);
      double jCandidate = std::acos(y) / dTheta;
      size_t jRounded = static_cast<size_t>(std::round(jCandidate));
      if (std::abs(jCandidate - jRounded) < tol) return jRounded;
    }
    return std::nullopt;
  }

  // Get signed distances from x to all Chebyshev points
  static Vector signedDistances(size_t N, double x, double a, double b) {
    Vector result(N);
    const Vector points = Chebyshev2::Points(N, a, b); // only thing that depends on [a,b]
    for (size_t j = 0; j < N; j++) {
      const double dj = x - points[j];
      result(j) = dj;
    }
    return result;
  }
}

Weights Chebyshev2::CalculateWeights(size_t N, double x, double a, double b) {
  // We start by getting distances from x to all Chebyshev points
  const Vector distances = signedDistances(N, x, a, b);

  Weights weights(N);
  if (auto j = coincidentPoint(N, x, a, b)) {
    // exceptional case: x coincides with a Chebyshev point
    weights.setZero();
    weights(*j) = 1;
    return weights;
  }

  // Beginning of interval, j = 0, x(0) = a
  weights(0) = 0.5 / distances(0);

  // All intermediate points j=1:N-2
  double d = weights(0), s = -1;  // changes sign s at every iteration
  for (size_t j = 1; j < N - 1; j++, s = -s) {
    weights(j) = s / distances(j);
    d += weights(j);
  }

  // End of interval, j = N-1, x(N-1) = b
  weights(N - 1) = 0.5 * s / distances(N - 1);
  d += weights(N - 1);

  // normalize
  return weights / d;
}

Weights Chebyshev2::DerivativeWeights(size_t N, double x, double a, double b) {
  Weights weightDerivatives(N);
  if (auto j = coincidentPoint(N, x, a, b)) {
    // exceptional case: x coincides with a Chebyshev point
    weightDerivatives.setZero();
    // compute the jth row of the differentiation matrix for this point
    double cj = (*j == 0 || *j == N - 1) ? 2. : 1.;
    for (size_t k = 0; k < N; k++) {
      if (*j == 0 && k == 0) {
        // we reverse the sign since we order the cheb points from -1 to 1
        weightDerivatives(k) = -(cj * (N - 1) * (N - 1) + 1) / 6.0;
      } else if (*j == N - 1 && k == N - 1) {
        // we reverse the sign since we order the cheb points from -1 to 1
        weightDerivatives(k) = (cj * (N - 1) * (N - 1) + 1) / 6.0;
      } else if (k == *j) {
        double xj = Point(N, *j);
        double xj2 = xj * xj;
        weightDerivatives(k) = -0.5 * xj / (1 - xj2);
      } else {
        double xj = Point(N, *j);
        double xk = Point(N, k);
        double ck = (k == 0 || k == N - 1) ? 2. : 1.;
        double t = ((*j + k) % 2) == 0 ? 1 : -1;
        weightDerivatives(k) = (cj / ck) * t / (xj - xk);
      }
    }
    return 2 * weightDerivatives / (b - a);
  }

  // This section of code computes the derivative of
  // the Barycentric Interpolation weights formula by applying
  // the chain rule on the original formula.

  // g and k are multiplier terms which represent the derivatives of
  // the numerator and denominator
  double g = 0, k = 0;
  double w = 1;

  const Vector distances = signedDistances(N, x, a, b);
  for (size_t j = 0; j < N; j++) {
    if (j == 0 || j == N - 1) {
      w = 0.5;
    } else {
      w = 1.0;
    }

    double t = (j % 2 == 0) ? 1 : -1;

    double c = t / distances(j);
    g += w * c;
    k += (w * c / distances(j));
  }

  double s = 1;  // changes sign s at every iteration
  double g2 = g * g;

  for (size_t j = 0; j < N; j++, s = -s) {
    // Beginning of interval, j = 0, x0 = -1.0 and end of interval, j = N-1,
    // x0 = 1.0
    if (j == 0 || j == N - 1) {
      w = 0.5;
    } else {
      // All intermediate points j=1:N-2
      w = 1.0;
    }
    weightDerivatives(j) = (w * -s / (g * distances(j) * distances(j))) -
                           (w * -s * k / (g2 * distances(j)));
  }

  return weightDerivatives;
}

Chebyshev2::DiffMatrix Chebyshev2::DifferentiationMatrix(size_t N, double a, double b) {
  DiffMatrix D(N, N);
  if (N == 1) {
    D(0, 0) = 1;
    return D;
  }

  const Vector points = Points(N); // a,b dependence is done at return
  for (size_t i = 0; i < N; i++) {
    double xi = points(i);
    double ci = (i == 0 || i == N - 1) ? 2. : 1.;
    for (size_t j = 0; j < N; j++) {
      if (i == 0 && j == 0) {
        // we reverse the sign since we order the cheb points from -1 to 1
        D(i, j) = -(ci * (N - 1) * (N - 1) + 1) / 6.0;
      } else if (i == N - 1 && j == N - 1) {
        // we reverse the sign since we order the cheb points from -1 to 1
        D(i, j) = (ci * (N - 1) * (N - 1) + 1) / 6.0;
      } else if (i == j) {
        double xi2 = xi * xi;
        D(i, j) = -xi / (2 * (1 - xi2));
      } else {
        double xj = points(j);
        double cj = (j == 0 || j == N - 1) ? 2. : 1.;
        double t = ((i + j) % 2) == 0 ? 1 : -1;
        D(i, j) = (ci / cj) * t / (xi - xj);
      }
    }
  }
  // scale the matrix to the range
  return D / ((b - a) / 2.0);
}

Weights Chebyshev2::IntegrationWeights(size_t N) {
  Weights weights(N);
  const size_t K = N - 1,  // number of intervals between N points
    K2 = K * K;

  // Compute endpoint weight.
  weights(0) = 1.0 / (K2 + K % 2 - 1);
  weights(N - 1) = weights(0);

  // Compute up to the middle; mirror symmetry holds.
  const size_t mid = (N - 1) / 2;
  double dTheta = M_PI / K;
  for (size_t i = 1; i <= mid; ++i) {
    double w = (K % 2 == 0) ? 1 - cos(i * M_PI) / (K2 - 1) : 1;
    const size_t last_k = K / 2 + K % 2 - 1;
    const double theta = i * dTheta;
    for (size_t k = 1; k <= last_k; ++k)
      w -= 2.0 * cos(2 * k * theta) / (4 * k * k - 1);
    w *= 2.0 / K;
    weights(i) = w;
    weights(N - 1 - i) = w;
  }
  return weights;
}

Weights Chebyshev2::IntegrationWeights(size_t N, double a, double b) {
  return IntegrationWeights(N) * (b - a) / 2.0;
}
}  // namespace gtsam
