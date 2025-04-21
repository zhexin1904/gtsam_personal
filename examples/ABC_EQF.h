/**
 * @file ABC_EQF.h
 * @brief Header file for the Attitude-Bias-Calibration Equivariant Filter
 *
 * This file contains declarations for the Equivariant Filter (EqF) for attitude estimation
 * with both gyroscope bias and sensor extrinsic calibration, based on the paper:
 * "Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation
 * with Online Calibration" by Fornasier et al.
 * Authors: Darshan Rajasekaran & Jennifer Oum
 */


#ifndef ABC_EQF_H
#define ABC_EQF_H
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/navigation/ImuBias.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <functional>
#include <chrono>
#include <numeric>  // For std::accumulate

// All implementations are wrapped in this namespace to avoid conflicts
namespace abc_eqf_lib {

using namespace std;
using namespace gtsam;

// Global configuration
// Define coordinate type: "EXPONENTIAL" or "NORMAL"
extern const std::string COORDINATE;

//========================================================================
// Utility Functions
//========================================================================


/// Check if a vector is a unit vector

bool checkNorm(const Vector3& x, double tol = 1e-3);

/// Check if vector contains NaN values

bool hasNaN(const Vector3& vec);

/// Create a block diagonal matrix from two matrices

Matrix blockDiag(const Matrix& A, const Matrix& B);

/// Repeat a block matrix n times along the diagonal

Matrix repBlock(const Matrix& A, int n);

/// Calculate numerical differential

Matrix numericalDifferential(std::function<Vector(const Vector&)> f, const Vector& x);

//========================================================================
// Core Data Types
//========================================================================

/// Direction class as a S2 element

class Direction {
public:
    Unit3 d;  // Direction (unit vector on S2)

    /**
     * Initialize direction
     * @param d_vec Direction vector (must be unit norm)
     */
    Direction(const Vector3& d_vec);

    // Accessor methods for vector components
    double x() const;
    double y() const;
    double z() const;

    // Check if the direction contains NaN values
    bool hasNaN() const;
};

/// Input class for the Biased Attitude System

struct Input {
    Vector3 w;               /// Angular velocity (3-vector)
    Matrix Sigma;            /// Noise covariance (6x6 matrix)
    static Input random();   /// Random input
    Matrix3 W() const {      /// Return w as a skew symmetric matrix
        return Rot3::Hat(w);
    }
    static Input create(const Vector3& w, const Matrix& Sigma) { /// Initialize w and Sigma
       if (Sigma.rows() != 6 || Sigma.cols() != 6) {
           throw std::invalid_argument("Input measurement noise covariance must be 6x6");
       }
       /// Check positive semi-definite
       Eigen::SelfAdjointEigenSolver<Matrix> eigensolver(Sigma);
       if (eigensolver.eigenvalues().minCoeff() < -1e-10) {
           throw std::invalid_argument("Covariance matrix must be semi-positive definite");
       }

       return Input{w, Sigma};  // use brace initialization
    }
};

/// Measurement class

struct Measurement {
    Direction y;        /// Measurement direction in sensor frame
    Direction d;        /// Known direction in global frame
    Matrix3 Sigma;      /// Covariance matrix of the measurement
    int cal_idx = -1;   /// Calibration index (-1 for calibrated sensor)
    Measurement(const Vector3& y_vec, const Vector3& d_vec,
                const Matrix3& Sigma, int i = -1);
};

/// State class representing the state of the Biased Attitude System

class State {
public:
    Rot3 R;                  // Attitude rotation matrix R
    Vector3 b;               // Gyroscope bias b
    std::vector<Rot3> S;     // Sensor calibrations S

    State(const Rot3& R = Rot3::Identity(),
          const Vector3& b = Vector3::Zero(),
          const std::vector<Rot3>& S = std::vector<Rot3>());

    static State identity(int n);
};

/// Data structure for ground-truth, input and output data

struct Data {
    State xi;                         // Ground-truth state
    int n_cal;                        // Number of calibration states
    Input u;                          // Input measurements
    std::vector<Measurement> y;       // Output measurements
    int n_meas;                       // Number of measurements
    double t;                         // Time
    double dt;                        // Time step

    /**
     * Initialize Data
     * @param xi Ground-truth state
     * @param n_cal Number of calibration states
     * @param u Input measurements
     * @param y Output measurements
     * @param n_meas Number of measurements
     * @param t Time
     * @param dt Time step
     */
    Data(const State& xi, int n_cal, const Input& u,
         const std::vector<Measurement>& y, int n_meas,
         double t, double dt);
};

//========================================================================
// Symmetry Group
//========================================================================

/**
 * Symmetry group (SO(3) |x so(3)) x SO(3) x ... x SO(3)
 * Each element of the B list is associated with a calibration state
 */
class G {
public:
    Rot3 A;                 // First SO(3) element
    Matrix3 a;              // so(3) element (skew-symmetric matrix)
    std::vector<Rot3> B;    // List of SO(3) elements for calibration

    /**
     * Initialize the symmetry group G
     * @param A SO3 element
     * @param a so(3) element (skew symmetric matrix)
     * @param B list of SO3 elements
     */
    G(const Rot3& A = Rot3::Identity(),
       const Matrix3& a = Matrix3::Zero(),
       const std::vector<Rot3>& B = std::vector<Rot3>());

    /**
     * Define the group operation (multiplication)
     * @param other Another group element
     * @return The product of this and other
     */
    G operator*(const G& other) const;

    /**
     * Return the inverse element of the symmetry group
     * @return The inverse of this group element
     */
    G inv() const;

    /**
     * Return the identity of the symmetry group
     * @param n Number of calibration elements
     * @return The identity element with n calibration components
     */
    static G identity(int n);

    /**
     * Return a group element X given by X = exp(x)
     * @param x Vector representation of Lie algebra element
     * @return Group element given by the exponential of x
     */
    static G exp(const Vector& x);
};

//========================================================================
// Helper Functions for EqF
//========================================================================

/**
 * Compute the lift of the system (Theorem 3.8, Equation 7)
 * @param xi State
 * @param u Input
 * @return Lift vector
 */
Vector lift(const State& xi, const Input& u);

/**
 * Action of the symmetry group on the state space (Equation 4)
 * @param X Group element
 * @param xi State
 * @return New state after group action
 */
State stateAction(const G& X, const State& xi);

/**
 * Action of the symmetry group on the input space (Equation 5)
 * @param X Group element
 * @param u Input
 * @return New input after group action
 */
Input velocityAction(const G& X, const Input& u);

/**
 * Action of the symmetry group on the output space (Equation 6)
 * @param X Group element
 * @param y Direction measurement
 * @param idx Calibration index
 * @return New direction after group action
 */
Vector3 outputAction(const G& X, const Direction& y, int idx);

/**
 * Local coordinates assuming xi_0 = identity (Equation 9)
 * @param e State representing equivariant error
 * @return Local coordinates
 */
Vector local_coords(const State& e);

/**
 * Local coordinates inverse assuming xi_0 = identity
 * @param eps Local coordinates
 * @return Corresponding state
 */
State local_coords_inv(const Vector& eps);

/**
 * Differential of the phi action at E = Id in local coordinates
 * @param xi State
 * @return Differential matrix
 */
Matrix stateActionDiff(const State& xi);

//========================================================================
// Equivariant Filter (EqF)
//========================================================================

/// Equivariant Filter (EqF) implementation

class EqF {
private:
    int dof;                 // Degrees of freedom
    int n_cal;               // Number of calibration states
    G X_hat;                 // Filter state
    Matrix Sigma;            // Error covariance
    State xi_0;              // Origin state
    Matrix Dphi0;            // Differential of phi at origin
    Matrix InnovationLift;   // Innovation lift matrix

    /**
     * Return the state matrix A0t (Equation 14a)
     * @param u Input
     * @return State matrix A0t
     */
    Matrix stateMatrixA(const Input& u) const;

    /**
     * Return the state transition matrix Phi (Equation 17)
     * @param u Input
     * @param dt Time step
     * @return State transition matrix Phi
     */
    Matrix stateTransitionMatrix(const Input& u, double dt) const;

    /**
     * Return the Input matrix Bt
     * @return Input matrix Bt
     */
    Matrix inputMatrixBt() const;

    /**
     * Return the measurement matrix C0 (Equation 14b)
     * @param d Known direction
     * @param idx Calibration index
     * @return Measurement matrix C0
     */
    Matrix measurementMatrixC(const Direction& d, int idx) const;

    /**
     * Return the measurement output matrix Dt
     * @param idx Calibration index
     * @return Measurement output matrix Dt
     */
    Matrix outputMatrixDt(int idx) const;

public:
    /**
     * Initialize EqF
     * @param Sigma Initial covariance
     * @param n Number of calibration states
     * @param m Number of sensors
     */
    EqF(const Matrix& Sigma, int n, int m);

    /**
     * Return estimated state
     * @return Current state estimate
     */
    State stateEstimate() const;

    /**
     * Propagate the filter state
     * @param u Angular velocity measurement
     * @param dt Time step
     */
    void propagation(const Input& u, double dt);

    /**
     * Update the filter state with a measurement
     * @param y Direction measurement
     */
    void update(const Measurement& y);
};

// Global configuration
const std::string COORDINATE = "EXPONENTIAL";  // Denotes how the states are mapped to the vector representations

//========================================================================
// Utility Functions
//========================================================================
/**
 * @brief Verifies if a vector has unit norm within tolerance
 * @param x 3d vector
 * @param tol optional tolerance
 * @return Bool indicating that the vector norm is approximately 1
 * Uses Vector3 norm() method to calculate vector magnitude
 */
bool checkNorm(const Vector3& x, double tol) {
    return abs(x.norm() - 1) < tol || std::isnan(x.norm());
}

/**
 * @brief Checks if the input vector has any NaNs
 * @param vec A 3-D vector
 * @return true if present, false otherwise
 */
bool hasNaN(const Vector3& vec) {
    return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
}

/**
 * @brief Creates a block diagonal matrix from input matrices
 * @param A Matrix A
 * @param B Matrix B
 * @return A single consolidated matrix with A in the top left and B in the
 * bottom right
 * Uses Matrix's rows(), cols(), setZero(), and block() methods
 */

Matrix blockDiag(const Matrix& A, const Matrix& B) {
    if (A.size() == 0) {
        return B;
    } else if (B.size() == 0) {
        return A;
    } else {
        Matrix result(A.rows() + B.rows(), A.cols() + B.cols());
        result.setZero();
        result.block(0, 0, A.rows(), A.cols()) = A;
        result.block(A.rows(), A.cols(), B.rows(), B.cols()) = B;
        return result;
    }
}
/**
 * @brief Creates a block diagonal matrix by repeating a matrix 'n' times
 * @param A A  matrix
 * @param n Number of times to be repeated
 * @return Block diag matrix with A repeated 'n' times
 * Recursively uses blockDiag() function
 */
Matrix repBlock(const Matrix& A, int n) {
    if (n <= 0) return Matrix();

    Matrix result = A;
    for (int i = 1; i < n; i++) {
        result = blockDiag(result, A);
    }
    return result;
}

/**
 * @brief Calculates the Jacobian matrix using central difference approximation
 * @param f Vector function f
 * @param x The point at which Jacobian is evaluated
 * @return Matrix containing numerical partial derivatives of f at x
 * Uses Vector's size() and Zero(), Matrix's Zero() and col() methods
 */
Matrix numericalDifferential(std::function<Vector(const Vector&)> f, const Vector& x) {
    double h = 1e-6;
    Vector fx = f(x);
    int n = fx.size();
    int m = x.size();
    Matrix Df = Matrix::Zero(n, m);

    for (int j = 0; j < m; j++) {
        Vector ej = Vector::Zero(m);
        ej(j) = 1.0;

        Vector fplus = f(x + h * ej);
        Vector fminus = f(x - h * ej);

        Df.col(j) = (fplus - fminus) / (2*h);
    }

    return Df;
}

//========================================================================
// Direction Class Implementation
//========================================================================

/**
 * @brief Initializes a direction object vector from a provided 3D vector input
 * @param d_vec A 3-D vector that should have a unit norm(This represents a
 * direction in 3D space) Uses Unit3's constructor which normalizes vectors
 */


Direction::Direction(const Vector3& d_vec) : d(d_vec) {
    if (!checkNorm(d_vec)) {
        throw std::invalid_argument("Direction must be a unit vector");
    }
}
 /** Access the individual components of the direction vector defined above using this methods below
  * Uses the Unit3 object from GTSAM to compute the components
  */

double Direction::x() const {
    return d.unitVector()[0];
}

double Direction::y() const {
    return d.unitVector()[1];
}

double Direction::z() const {
    return d.unitVector()[2];
}

bool Direction::hasNaN() const {
    Vector3 vec = d.unitVector();
    return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
}

//========================================================================
// Input Class Implementation
//========================================================================
/**
 * @brief Constructs an input object using the Angular velocity vector and the
 * covariance matrix
 * @param w Angular vector
 * @param Sigma 6X6 covariance matrix
 * Uses Matrix's rows(), cols() and Eigen's SelfAdjointEigenSolver
 */
// Input::Input(const Vector3& w, const Matrix& Sigma) : w(w), Sigma(Sigma) {
//     if (Sigma.rows() != 6 || Sigma.cols() != 6) {
//         throw std::invalid_argument("Input measurement noise covariance must be 6x6");
//     }
//
//     // Check positive semi-definite
//     Eigen::SelfAdjointEigenSolver<Matrix> eigensolver(Sigma);
//     if (eigensolver.eigenvalues().minCoeff() < -1e-10) {
//         throw std::invalid_argument("Covariance matrix must be semi-positive definite");
//     }
// }
/**
 *
 * @return 3X3 skey symmetric matrix when called
 * Uses Rot3's Hat() to create skew-symmetric matrix
 */
// Matrix3 Input::W() const {
//     return Rot3::Hat(w);
// }


//========================================================================
// Measurement Class Implementation
//========================================================================
/**
 * @brief Constructs measurement with directions and covariance.
 * @param y_vec A 3D vector representing the measured direction in the sensor frame
 * @param d_vec A 3D vector representing the known reference direction in the global frame aka ground truth direction
 * @param Sigma 3x3 positive definite covariance vector representing the uncertainty in the measurements
 * @param i Calibration index - A non-negative integer specifies the element in the calibration vector
 * that corresponds to the sensor of interest. A value of -1 indicates that all the sensors have been calibrated
 *
 * Creates a measurement object that stores the measured direction(y), reference direction(d), measurement noise covariance(Sigma)
 * and Calibration Index cal_idx
 *
 * Uses Eigen's SelfAdjointEigenSolver
 *
 */

Measurement::Measurement(const Vector3& y_vec, const Vector3& d_vec,
                         const Matrix3& Sigma, int i)
    : y(y_vec), d(d_vec), Sigma(Sigma), cal_idx(i) {

    // Check positive semi-definite
    Eigen::SelfAdjointEigenSolver<Matrix3> eigensolver(Sigma);
    if (eigensolver.eigenvalues().minCoeff() < -1e-10) {
        throw std::invalid_argument("Covariance matrix must be semi-positive definite");
    }
}

//========================================================================
// State Class Implementation
//========================================================================
/**
 *
 * @param R Rot3 (Attitude)
 * @param b Vector (Bias)
 * @param S Vector (Rot 3 calibration states)
 * Combines the navigation and the calibration states together and provides a
 * mechanism to represent the complete system
 *
 */
State::State(const Rot3& R, const Vector3& b, const std::vector<Rot3>& S)
    : R(R), b(b), S(S) {}

/**
 *
 * @param n Number of Calibration states
 * @return State object intitialized to identity
 * Creates a default/ initial state
 * Uses GTSAM's Rot3::identity and Vector3 zero function
 */
State State::identity(int n) {
    std::vector<Rot3> calibrations(n, Rot3::Identity());
    return State(Rot3::Identity(), Vector3::Zero(), calibrations);
}

//========================================================================
// Data Struct Implementation
//========================================================================

/**
 *
 * @param xi Ground Truth state
 * @param n_cal Number of calibration states
 * @param u Input measurements
 * @param y Vector of y measurements
 * @param n_meas number of measurements
 * @param t timestamp
 * @param dt time step
 * Used to organize the state, input and measurement data with timestamps for
 * testing Uses Rot3, Vector 3 and Unit3 classes
 */
Data::Data(const State& xi, int n_cal, const Input& u,
           const std::vector<Measurement>& y, int n_meas,
           double t, double dt)
    : xi(xi), n_cal(n_cal), u(u), y(y), n_meas(n_meas), t(t), dt(dt) {}

//========================================================================
// Symmetry Group Implementation - Group Elements and actions
//========================================================================
/**
 *
 * @param A Attitude element of Rot3 type
 * @param a Matrix3 bias element
 * @param B Rot3 vector containing calibration elements
 * Ouptuts a G object using Rot3 for rotation representation
 */
G::G(const Rot3& A, const Matrix3& a, const std::vector<Rot3>& B)
    : A(A), a(a), B(B) {}

/**
 * Defines the group operation (multiplication)
 * @param other Another Group element
 * @return G a product of two group elements
 * Uses Rot3 Hat, Rot3 Vee for multiplication
 *
 */
G G::operator*(const G& other) const {
    if (B.size() != other.B.size()) {
        throw std::invalid_argument("Group elements must have the same number of calibration elements");
    }

    std::vector<Rot3> new_B;
    for (size_t i = 0; i < B.size(); i++) {
        new_B.push_back(B[i] * other.B[i]);
    }

    return G(A * other.A,
            a + Rot3::Hat(A.matrix() * Rot3::Vee(other.a)),
            new_B);
}

/**
 * Used to compute the Group inverse
 * @return The inverse of group element
 * Uses Rot3 inverse, Rot3 matrix, hat and vee functions
 * The left invariant  property of the semi-direct product group structure is implemented here by using the -ve sign
 */
G G::inv() const {
    Matrix3 A_inv = A.inverse().matrix();

    std::vector<Rot3> B_inv;
    for (const auto& b : B) {
        B_inv.push_back(b.inverse());
    }

    return G(A.inverse(),
            -Rot3::Hat(A_inv * Rot3::Vee(a)),
            B_inv);
}

/**
 * Creates the identity element of the group
 * @param n Number of calibration elements
 * @return the identity element
 * Uses Rot3 Identity and Matrix zero
 */
G G::identity(int n) {
    std::vector<Rot3> B(n, Rot3::Identity());
    return G(Rot3::Identity(), Matrix3::Zero(), B);
}
/**
 * Maps the tangent space elements to the group
 * @param x Vector in lie algebra
 * @return the group element G
 * Uses Rot3 expmap and Expmapderivative function
 */
G G::exp(const Vector& x) {
    if (x.size() < 6 || x.size() % 3 != 0) {
        throw std::invalid_argument("Wrong size, a vector with size multiple of 3 and at least 6 must be provided");
    }

    int n = (x.size() - 6) / 3;
    Rot3 A = Rot3::Expmap(x.head<3>());

    Vector3 a_vee = Rot3::ExpmapDerivative(-x.head<3>()) * x.segment<3>(3);
    Matrix3 a = Rot3::Hat(a_vee);

    std::vector<Rot3> B;
    for (int i = 0; i < n; i++) {
        B.push_back(Rot3::Expmap(x.segment<3>(6 + 3*i)));
    }

    return G(A, a, B);
}

//========================================================================
// Helper Functions Implementation
//========================================================================

/**
 * Maps system dynamics to the symmetry group
 * @param xi State
 * @param u Input
 * @return Lifted input in Lie Algebra
 * Uses Vector zero & Rot3 inverse, matrix functions
 */
Vector lift(const State& xi, const Input& u) {
    int n = xi.S.size();
    Vector L = Vector::Zero(6 + 3 * n);

    // First 3 elements
    L.head<3>() = u.w - xi.b;

    // Next 3 elements
    L.segment<3>(3) = -u.W() * xi.b;

    // Remaining elements
    for (int i = 0; i < n; i++) {
        L.segment<3>(6 + 3*i) = xi.S[i].inverse().matrix() * L.head<3>();
    }

    return L;
}
/**
 * Implements group actions on the states
 * @param X A symmetry group element G consisting of the attitude, bias and the
 * calibration components X.a -> Rotation matrix containing the attitude X.b ->
 * A skew-symmetric matrix representing bias X.B -> A vector of Rotation
 * matrices for the calibration components
 * @param xi State object
 * xi.R -> Attitude (Rot3)
 * xi.b -> Gyroscope Bias(Vector 3)
 * xi.S -> Vector of calibration matrices(Rot3)
 * @return Transformed state
 * Uses the Rot3 inverse and Vee functions
 */
State stateAction(const G& X, const State& xi) {
    if (xi.S.size() != X.B.size()) {
        throw std::invalid_argument("Number of calibration states and B elements must match");
    }

    std::vector<Rot3> new_S;
    for (size_t i = 0; i < X.B.size(); i++) {
        new_S.push_back(X.A.inverse() * xi.S[i] * X.B[i]);
    }

    return State(xi.R * X.A,
                X.A.inverse().matrix() * (xi.b - Rot3::Vee(X.a)),
                new_S);
}
/**
 * Transforms the angular velocity measurements b/w frames
 * @param X A symmetry group element X with the components
 * @param u Inputs
 * @return Transformed inputs
 * Uses Rot3 Inverse, matrix and Vee functions and is critical for maintaining
 * the input equivariance
 */
Input velocityAction(const G& X, const Input& u) {
    return Input{X.A.inverse().matrix() * (u.w - Rot3::Vee(X.a)), u.Sigma};
}
/**
 * Transforms the Direction measurements based on the calibration type ( Eqn 6)
 * @param X Group element X
 * @param y Direction measurement y
 * @param idx Calibration index
 * @return Transformed direction
 * Uses Rot3 inverse, matric and Unit3 unitvector functions
 */
Vector3 outputAction(const G& X, const Direction& y, int idx) {
    if (idx == -1) {
        return X.A.inverse().matrix() * y.d.unitVector();
    } else {
        if (idx >= static_cast<int>(X.B.size())) {
            throw std::out_of_range("Calibration index out of range");
        }
        return X.B[idx].inverse().matrix() * y.d.unitVector();
    }
}

/**
 * Maps the error states to vector representations through exponential
 * coordinates
 * @param e error state
 * @return Vector with local coordinates
 * Uses Rot3 logamo for mapping rotations to the tangent space
 */
Vector local_coords(const State& e) {
    if (COORDINATE == "EXPONENTIAL") {
        Vector eps(6 + 3 * e.S.size());

        // First 3 elements
        eps.head<3>() = Rot3::Logmap(e.R);

        // Next 3 elements
        eps.segment<3>(3) = e.b;

        // Remaining elements
        for (size_t i = 0; i < e.S.size(); i++) {
            eps.segment<3>(6 + 3*i) = Rot3::Logmap(e.S[i]);
        }

        return eps;
    } else if (COORDINATE == "NORMAL") {
        throw std::runtime_error("Normal coordinate representation is not implemented yet");
    } else {
        throw std::invalid_argument("Invalid coordinate representation");
    }
}
/**
 * Used to convert the vectorized errors back to state space
 * @param eps Local coordinates in the exponential parameterization
 * @return State object corresponding to these coordinates
 * Uses Rot3 expmap through the G::exp() function
 */
State local_coords_inv(const Vector& eps) {
    G X = G::exp(eps);

    if (COORDINATE == "EXPONENTIAL") {
        std::vector<Rot3> S = X.B;
        return State(X.A, eps.segment<3>(3), S);
    } else if (COORDINATE == "NORMAL") {
        throw std::runtime_error("Normal coordinate representation is not implemented yet");
    } else {
        throw std::invalid_argument("Invalid coordinate representation");
    }
}
/**
 * Computes the differential of a state action at the identity of the symmetry
 * group
 * @param xi State object Xi representing the point at which to evaluate the
 * differential
 * @return A matrix representing the jacobian of the state action
 * Uses numericalDifferential, and Rot3 expmap, logmap
 */
Matrix stateActionDiff(const State& xi) {
    std::function<Vector(const Vector&)> coordsAction =
        [&xi](const Vector& U) {
            return local_coords(stateAction(G::exp(U), xi));
        };

    Vector zeros = Vector::Zero(6 + 3 * xi.S.size());
    Matrix differential = numericalDifferential(coordsAction, zeros);
    return differential;
}

//========================================================================
// Equivariant Filter (EqF) Implementation
//========================================================================
/**
 * Initializes the EqF with state dimension validation  and computes lifted
 * innovation mapping
 * @param Sigma Initial covariance
 * @param n Number of calibration states
 * @param m Number of sensors
 * Uses SelfAdjointSolver, completeOrthoganalDecomposition().pseudoInverse()
 */
EqF::EqF(const Matrix& Sigma, int n, int m)
    : dof(6 + 3 * n), n_cal(n),  X_hat(G::identity(n)),
      Sigma(Sigma), xi_0(State::identity(n)) {

    if (Sigma.rows() != dof || Sigma.cols() != dof) {
        throw std::invalid_argument("Initial covariance dimensions must match the degrees of freedom");
    }

    // Check positive semi-definite
    Eigen::SelfAdjointEigenSolver<Matrix> eigensolver(Sigma);
    if (eigensolver.eigenvalues().minCoeff() < -1e-10) {
        throw std::invalid_argument("Covariance matrix must be semi-positive definite");
    }

    if (n < 0) {
        throw std::invalid_argument("Number of calibration states must be non-negative");
    }

    if (m <= 1) {
        throw std::invalid_argument("Number of direction sensors must be at least 2");
    }

    // Compute differential of phi
    Dphi0 = stateActionDiff(xi_0);
    InnovationLift = Dphi0.completeOrthogonalDecomposition().pseudoInverse();
}
/**
 * Computes the internal group state to a physical state estimate
 * @return Current state estimate
 */
State EqF::stateEstimate() const {
    return stateAction(X_hat, xi_0);
}
/**
 * Implements the prediction step of the EqF using system dynamics and
 * covariance propagation and advances the filter state by symmtery-preserving
 * dynamics.Uses a Lie group integrator scheme for discrete time propagation
 * @param u Angular velocity measurements
 * @param dt time steps
 * Updated internal state and covariance
 */
void EqF::propagation(const Input& u, double dt) {
    State state_est = stateEstimate();
    Vector L = lift(state_est, u);

    Matrix Phi_DT = stateTransitionMatrix(u, dt);
    Matrix Bt = inputMatrixBt();

    Matrix tempSigma = blockDiag(u.Sigma,
                                repBlock(1e-9 * Matrix3::Identity(), n_cal));
    Matrix M_DT = (Bt * tempSigma * Bt.transpose()) * dt;

    X_hat = X_hat * G::exp(L * dt);
    Sigma = Phi_DT * Sigma * Phi_DT.transpose() + M_DT;
}
/**
 * Implements the correction step of the filter using discrete measurements
 * Computes the measurement residual, Kalman gain and the updates both the state
 * and covariance
 *
 * @param y Measurements
 */
void EqF::update(const Measurement& y) {
  if (y.cal_idx > n_cal) {
    throw std::invalid_argument("Calibration index out of range");
  }

  // Get vector representations for checking
  Vector3 y_vec = y.y.d.unitVector();
  Vector3 d_vec = y.d.d.unitVector();

  // Skip update if any NaN values are present
  if (std::isnan(y_vec[0]) || std::isnan(y_vec[1]) || std::isnan(y_vec[2]) ||
      std::isnan(d_vec[0]) || std::isnan(d_vec[1]) || std::isnan(d_vec[2])) {
    return;  // Skip this measurement
  }

  Matrix Ct = measurementMatrixC(y.d, y.cal_idx);
  Vector3 action_result = outputAction(X_hat.inv(), y.y, y.cal_idx);
  Vector3 delta_vec = Rot3::Hat(y.d.d.unitVector()) * action_result;
  Matrix Dt = outputMatrixDt(y.cal_idx);
  Matrix S = Ct * Sigma * Ct.transpose() + Dt * y.Sigma * Dt.transpose();
  Matrix K = Sigma * Ct.transpose() * S.inverse();
  Vector Delta = InnovationLift * K * delta_vec;
  X_hat = G::exp(Delta) * X_hat;
  Sigma = (Matrix::Identity(dof, dof) - K * Ct) * Sigma;
}
/**
 * Computes linearized continuous time state matrix
 * @param u Angular velocity
 * @return Linearized state matrix
 * Uses Matrix zero and Identity functions
 */
Matrix EqF::stateMatrixA(const Input& u) const {
    Matrix3 W0 = velocityAction(X_hat.inv(), u).W();
    Matrix A1 = Matrix::Zero(6, 6);

    if (COORDINATE == "EXPONENTIAL") {
        A1.block<3, 3>(0, 3) = -Matrix3::Identity();
        A1.block<3, 3>(3, 3) = W0;
        Matrix A2 = repBlock(W0, n_cal);
        return blockDiag(A1, A2);
    } else if (COORDINATE == "NORMAL") {
        throw std::runtime_error("Normal coordinate representation is not implemented yet");
    } else {
        throw std::invalid_argument("Invalid coordinate representation");
    }
}

/**
 * Computes the discrete time state transition matrix
 * @param u Angular velocity
 * @param dt time step
 * @return State transition matrix in discrete time
 */
Matrix EqF::stateTransitionMatrix(const Input& u, double dt) const {
    Matrix3 W0 = velocityAction(X_hat.inv(), u).W();
    Matrix Phi1 = Matrix::Zero(6, 6);

    Matrix3 Phi12 = -dt * (Matrix3::Identity() + (dt / 2) * W0 + ((dt*dt) / 6) * W0 * W0);
    Matrix3 Phi22 = Matrix3::Identity() + dt * W0 + ((dt*dt) / 2) * W0 * W0;

    if (COORDINATE == "EXPONENTIAL") {
        Phi1.block<3, 3>(0, 0) = Matrix3::Identity();
        Phi1.block<3, 3>(0, 3) = Phi12;
        Phi1.block<3, 3>(3, 3) = Phi22;
        Matrix Phi2 = repBlock(Phi22, n_cal);
        return blockDiag(Phi1, Phi2);
    } else if (COORDINATE == "NORMAL") {
        throw std::runtime_error("Normal coordinate representation is not implemented yet");
    } else {
        throw std::invalid_argument("Invalid coordinate representation");
    }
}
/**
 * Computes the input uncertainty propagation matrix
 * @return
 * Uses the blockdiag matrix
 */
Matrix EqF::inputMatrixBt() const {
    if (COORDINATE == "EXPONENTIAL") {
        Matrix B1 = blockDiag(X_hat.A.matrix(), X_hat.A.matrix());
        Matrix B2;

        for (const auto& B : X_hat.B) {
            if (B2.size() == 0) {
                B2 = B.matrix();
            } else {
                B2 = blockDiag(B2, B.matrix());
            }
        }

        return blockDiag(B1, B2);
    } else if (COORDINATE == "NORMAL") {
        throw std::runtime_error("Normal coordinate representation is not implemented yet");
    } else {
        throw std::invalid_argument("Invalid coordinate representation");
    }
}
/**
 * Computes the linearized measurement matrix. The structure depends on whether
 * the sensor has a calibration state
 * @param d reference direction
 * @param idx Calibration index
 * @return Measurement matrix
 * Uses the matrix zero, Rot3 hat and the Unitvector functions
 */
Matrix EqF::measurementMatrixC(const Direction& d, int idx) const {
    Matrix Cc = Matrix::Zero(3, 3 * n_cal);

    // If the measurement is related to a sensor that has a calibration state
    if (idx >= 0) {
        // Set the correct 3x3 block in Cc
        Cc.block<3, 3>(0, 3 * idx) = Rot3::Hat(d.d.unitVector());
    }

    Matrix3 wedge_d = Rot3::Hat(d.d.unitVector());

    // Create the combined matrix
    Matrix temp(3, 6 + 3 * n_cal);
    temp.block<3, 3>(0, 0) = wedge_d;
    temp.block<3, 3>(0, 3) = Matrix3::Zero();
    temp.block(0, 6, 3, 3 * n_cal) = Cc;

    return wedge_d * temp;
}
/**
 * Computes the measurement uncertainty propagation matrix
 * @param idx Calibration index
 * @return Returns B[idx] for calibrated sensors, A for uncalibrated
 */
Matrix EqF::outputMatrixDt(int idx) const {
    // If the measurement is related to a sensor that has a calibration state
    if (idx >= 0) {
        if (idx >= static_cast<int>(X_hat.B.size())) {
            throw std::out_of_range("Calibration index out of range");
        }
        return X_hat.B[idx].matrix();
    } else {
        return X_hat.A.matrix();
    }
}




} // namespace abc_eqf_lib

#endif // ABC_EQF_H