/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    Gal3.cpp
 * @brief   Implementation of 3D Galilean Group SGal(3) state
 * @author  Based on Python implementation by User
 * @date    April 29, 2025
 */

#include <gtsam/navigation/Gal3.h> // Note: Adjust path if needed
#include <gtsam/geometry/SO3.h> // For so3::DexpFunctor if needed, and Rot3 Log/Exp
#include <gtsam/base/numericalDerivative.h> // For derivative checking if desired
#include <gtsam/base/Matrix.h> // For Z_3x3 etc.
#include <gtsam/nonlinear/expressions.h> // For constructing expressions

#include <iostream>
#include <cmath>
#include <functional> // For std::function

namespace gtsam {

//------------------------------------------------------------------------------
// Static Helper Functions Implementation
//------------------------------------------------------------------------------

Matrix3 Gal3::SO3_LeftJacobian(const Vector3& theta) {
    using std::cos;
    using std::sin;
    using std::sqrt;

    const double angle_sq = theta.squaredNorm();

    // Use Taylor series expansion for small angles
    if (angle_sq < kSmallAngleThreshold * kSmallAngleThreshold) {
        // Jl(w) approx = I + 0.5*[w]x + (1/6)*[w]x^2
        const Matrix3 W = skewSymmetric(theta);
        return Matrix3::Identity() + 0.5 * W + (1.0/6.0) * W * W;
    }

    const double angle = sqrt(angle_sq);
    const Matrix3 W = skewSymmetric(theta);
    const Matrix3 W2 = W * W;

    const double inv_angle_sq = 1.0 / angle_sq;
    const double sin_angle = sin(angle);
    const double cos_angle = cos(angle);

    // coeff1 = (1 - cos(theta)) / theta^2
    const double coeff1 = (1.0 - cos_angle) * inv_angle_sq;
    // coeff2 = (theta - sin(theta)) / theta^3
    const double coeff2 = (angle - sin_angle) / (angle_sq * angle);

    return Matrix3::Identity() + coeff1 * W + coeff2 * W2;
}

//------------------------------------------------------------------------------
Matrix3 Gal3::SO3_LeftJacobianInverse(const Vector3& theta) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    using std::tan;

    const double angle_sq = theta.squaredNorm();
    const Matrix3 W = skewSymmetric(theta);

    // Use Taylor series expansion for small angles
    if (angle_sq < kSmallAngleThreshold * kSmallAngleThreshold) {
        // Jl(w)^-1 approx = I - 0.5*W + (1/12)*W^2
        return Matrix3::Identity() - 0.5 * W + (1.0/12.0) * W * W;
    }

    const double angle = sqrt(angle_sq);
    const double half_angle = 0.5 * angle;

    // Formula: I - 0.5*W + (1/theta^2 - cot(theta/2)/(2*theta)) * W^2
    // which is I - 0.5*W + (1 - 0.5*theta*cot(0.5*theta))/theta^2 * W^2
    double cot_half_angle;
    // Avoid division by zero/very small numbers for tan(half_angle)
    if (std::abs(sin(half_angle)) < kSmallAngleThreshold) {
        // If sin(half_angle) is near zero, theta is near multiples of 2*pi.
        // cot(x) ~ 1/x for small x. Let x = half_angle.
        // If half_angle is very small, use Taylor expansion of the coefficient:
        // (1 - 0.5*theta*cot(0.5*theta))/theta^2 -> 1/12 + O(theta^2)
        if (std::abs(half_angle) < kSmallAngleThreshold) {
           // Use Taylor approx for the coefficient directly
           const double coeff = 1.0 / 12.0; // + angle_sq / 720.0 + ...; // Higher order if needed
           return Matrix3::Identity() - 0.5 * W + coeff * (W * W);
        } else {
            // theta is near non-zero multiples of 2*pi. tan(half_angle) is near zero.
            // cot becomes large. Use limit or high-precision calculation if needed.
            // For now, use standard tan. May need robustness improvements.
             cot_half_angle = cos(half_angle) / sin(half_angle); // Avoid 1.0/tan()
        }
    } else {
        cot_half_angle = cos(half_angle) / sin(half_angle); // Avoid 1.0/tan()
    }

    const double coeff = (1.0 - 0.5 * angle * cot_half_angle) / angle_sq;
    return Matrix3::Identity() - 0.5 * W + coeff * (W * W);

    // Alternative using GTSAM SO3 properties (verify this matches J_l^-1):
    // J_l(theta)^-1 = J_r(-theta).
    // return gtsam::so3::DexpFunctor(-theta).rightJacobian();
}


//------------------------------------------------------------------------------
Matrix3 Gal3::Calculate_E(const Vector3& theta) {
    using std::cos;
    using std::sin;
    using std::sqrt;

    const double angle_sq = theta.squaredNorm();
    const Matrix3 W = skewSymmetric(theta);
    const Matrix3 W2 = W * W;

    Matrix3 E = 0.5 * Matrix3::Identity();

    // Small angle approximation (from manif SGal3Tangent_base.h / Expmap formula)
    // E approx = 0.5*I + (1/6)*W + (1/24)*W^2
    if (angle_sq < kSmallAngleThreshold * kSmallAngleThreshold) {
        E += (1.0 / 6.0) * W + (1.0 / 24.0) * W2;
        return E;
    }

    const double angle = sqrt(angle_sq);
    const double sin_angle = sin(angle);
    const double cos_angle = cos(angle);

    // Coefficients A and B from manif::SGal3TangentBase::exp()
    // A = (theta - sin(theta)) / theta^3
    const double A = (angle - sin_angle) / (angle_sq * angle);
    // B = (theta^2 + 2*cos(theta) - 2) / (2 * theta^4)
    const double B = (angle_sq + 2.0 * cos_angle - 2.0) / (2.0 * angle_sq * angle_sq);

    E += A * W + B * W2;
    return E;
}

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
Gal3::Gal3(const Matrix5& M) {
    // CORRECTED: Use head(N) with correct sizes for checks
    if (std::abs(M(3, 3) - 1.0) > 1e-9 || std::abs(M(4, 4) - 1.0) > 1e-9 ||
        M.row(4).head(4).norm() > 1e-9 || M.row(3).head(3).norm() > 1e-9) { // Row 4 first 4 = 0; Row 3 first 3 = 0
        throw std::invalid_argument("Invalid Gal3 matrix structure: Check zero blocks and diagonal ones.");
    }
    R_ = Rot3(M.block<3, 3>(0, 0));
    v_ = M.block<3, 1>(0, 3); // Manif: v is in column 3 (0-indexed)
    r_ = Point3(M.block<3, 1>(0, 4)); // Manif: r is in column 4
    t_ = M(3, 4);             // Manif: t is at (3, 4)
}

//------------------------------------------------------------------------------
// Component Access
//------------------------------------------------------------------------------
const Rot3& Gal3::rotation(OptionalJacobian<3, 10> H) const {
    if (H) {
        *H = Matrix::Zero(3, 10);
        H->block<3, 3>(0, 6) = Matrix3::Identity(); // Derivative w.r.t theta
    }
    return R_;
}

//------------------------------------------------------------------------------
const Point3& Gal3::translation(OptionalJacobian<3, 10> H) const {
     if (H) {
        *H = Matrix::Zero(3, 10);
        H->block<3, 3>(0, 0) = R_.matrix(); // Derivative w.r.t rho (local tangent space)
    }
    return r_;
}

//------------------------------------------------------------------------------
const Velocity3& Gal3::velocity(OptionalJacobian<3, 10> H) const {
     if (H) {
        *H = Matrix::Zero(3, 10);
        H->block<3, 3>(0, 3) = R_.matrix(); // d(v)/d(nu)
     }
    return v_;
}

//------------------------------------------------------------------------------
const double& Gal3::time(OptionalJacobian<1, 10> H) const {
    if (H) {
        *H = Matrix::Zero(1, 10);
        (*H)(0, 9) = 1.0; // Derivative w.r.t t_tan
    }
    return t_;
}

//------------------------------------------------------------------------------
// Matrix Representation
//------------------------------------------------------------------------------
Matrix5 Gal3::matrix() const {
    Matrix5 M = Matrix5::Identity();
    M.block<3, 3>(0, 0) = R_.matrix();
    M.block<3, 1>(0, 3) = v_; // Manif: v in col 3
    M.block<3, 1>(0, 4) = Vector3(r_); // Manif: r in col 4 // CORRECTED: Assign Vector3 from Point3
    M(3, 4) = t_;             // Manif: t at (3, 4)
    M.block<1,3>(3,0).setZero(); // Zero out row 3, cols 0-2
    M.block<1,4>(4,0).setZero(); // Zero out row 4, cols 0-3
    return M;
}

//------------------------------------------------------------------------------
// Testable Requirements
//------------------------------------------------------------------------------
void Gal3::print(const std::string& s) const {
    std::cout << (s.empty() ? "" : s + " ");
    std::cout << *this << std::endl;
}

//------------------------------------------------------------------------------
bool Gal3::equals(const Gal3& other, double tol) const {
    return R_.equals(other.R_, tol) &&
           traits<Point3>::Equals(r_, other.r_, tol) &&
           traits<Velocity3>::Equals(v_, other.v_, tol) &&
           std::abs(t_ - other.t_) < tol;
}

//------------------------------------------------------------------------------
// Group Operations
//------------------------------------------------------------------------------
Gal3 Gal3::inverse() const {
    // Formula: R_inv = R', v_inv = -R'*v, r_inv = -R'*(r - t*v), t_inv = -t
    const Rot3 Rinv = R_.inverse();
    const Velocity3 v_inv = -(Rinv.rotate(v_));
    // CORRECTED: Cast r_ to Vector3 for subtraction
    const Point3 r_inv = -(Rinv.rotate(Vector3(r_) - t_ * v_));
    const double t_inv = -t_;
    return Gal3(Rinv, r_inv, v_inv, t_inv);
}

//------------------------------------------------------------------------------
Gal3 Gal3::compose(const Gal3& other, OptionalJacobian<10, 10> H1, OptionalJacobian<10, 10> H2) const {
    // Formula: R = R1*R2, v = R1*v2 + v1, r = R1*r2 + t2*v1 + r1, t = t1+t2
    // where this = g1, other = g2
    const Gal3& g1 = *this;
    const Gal3& g2 = other;

    const Rot3 R_comp = g1.R_.compose(g2.R_);
    // CORRECTED: Cast g1.r_ to Vector3 for addition
    const Vector3 r_comp_vec = g1.R_.rotate(g2.r_) + g2.t_ * g1.v_ + Vector3(g1.r_);
    const Velocity3 v_comp = g1.R_.rotate(g2.v_) + g1.v_;
    const double t_comp = g1.t_ + g2.t_;

    Gal3 result(R_comp, Point3(r_comp_vec), v_comp, t_comp);

    // Jacobians require correct AdjointMap implementation
    if (H1) {
         *H1 = g2.inverse().AdjointMap();
    }
    if (H2) {
        *H2 = Matrix10::Identity();
    }

    return result;
}

//------------------------------------------------------------------------------
Gal3 Gal3::between(const Gal3& other, OptionalJacobian<10, 10> H1, OptionalJacobian<10, 10> H2) const {
    const Gal3& g1 = *this;
    const Gal3& g2 = other;
    Gal3 g1_inv = g1.inverse();
    Gal3 result = g1_inv.compose(g2); // g1_inv * g2

    // Jacobians require correct AdjointMap implementation
    if (H1) {
        *H1 = -result.AdjointMap();
    }
    if (H2) {
         *H2 = g1_inv.AdjointMap();
    }

    return result;
}


//------------------------------------------------------------------------------
// Manifold Operations (Retract/Local)
//------------------------------------------------------------------------------
Gal3 Gal3::retract(const Vector10& xi, OptionalJacobian<10, 10> H1, OptionalJacobian<10, 10> H2) const {
    Gal3 g_exp = Gal3::Expmap(xi);
    Matrix10 H_exp_xi = Matrix10::Zero(); // Placeholder
    if (H2) {
       H_exp_xi = Gal3::ExpmapDerivative(xi); // Needs ExpmapDerivative implemented
    }

    Matrix10 H_comp_this, H_comp_gexp;
    Gal3 result = this->compose(g_exp, H_comp_this, H_comp_gexp);

    if (H1) {
        *H1 = H_comp_this; // Requires AdjointMap
    }
    if (H2) {
        // Needs ExpmapDerivative and potentially AdjointMap if H_comp_gexp is not Identity
        *H2 = H_comp_gexp * H_exp_xi; // H_comp_gexp is Identity here
    }
    return result;
}

//------------------------------------------------------------------------------
Vector10 Gal3::localCoordinates(const Gal3& other, OptionalJacobian<10, 10> H1, OptionalJacobian<10, 10> H2) const {
    Matrix10 H_between_this, H_between_other;
    Gal3 result_g = this->between(other, H_between_this, H_between_other);

    Matrix10 H_log_g = Matrix10::Zero(); // Placeholder
    if (H1 || H2) {
        H_log_g = Gal3::LogmapDerivative(result_g); // Needs LogmapDerivative implemented
    }
    Vector10 result_xi = Gal3::Logmap(result_g);

    // Jacobians require LogmapDerivative and AdjointMap (via between Jacobians)
    if (H1) {
        *H1 = H_log_g * H_between_this;
    }
    if (H2) {
        *H2 = H_log_g * H_between_other;
    }
    return result_xi;
}

//------------------------------------------------------------------------------
// Lie Group Static Functions
//------------------------------------------------------------------------------

Matrix5 Gal3::Hat(const Vector10& xi) {
    const Vector3 rho_tan = rho(xi);
    const Vector3 nu_tan = nu(xi);
    const Vector3 theta_tan = theta(xi);
    const double t_tan_val = t_tan(xi)(0);

    Matrix5 X = Matrix5::Zero();
    X.block<3, 3>(0, 0) = skewSymmetric(theta_tan);
    X.block<3, 1>(0, 3) = nu_tan;  // nu in column 3
    X.block<3, 1>(0, 4) = rho_tan; // rho in column 4
    X(3, 4) = t_tan_val;           // t in element (3, 4)
    return X;
}

//------------------------------------------------------------------------------
Vector10 Gal3::Vee(const Matrix5& X) {
    // CORRECTED: Check X.row(3).head(N) for Lie algebra structure
    // Lie algebra row 3 is [0 0 0 0 t], so first 3 elements should be 0.
    // Lie algebra row 4 is [0 0 0 0 0], so all 5 elements should be 0.
    if (X.row(4).norm() > 1e-9 || X.row(3).head(3).norm() > 1e-9) { // Row 4 all=0; Row 3 first 3=0
     throw std::invalid_argument("Matrix is not in sgal(3)");
    }

    Vector10 xi;
    rho(xi) = X.block<3, 1>(0, 4); // rho from column 4
    nu(xi) = X.block<3, 1>(0, 3);  // nu from column 3
    const Matrix3& S = X.block<3, 3>(0, 0);
    theta(xi) << S(2, 1), S(0, 2), S(1, 0);
    t_tan(xi)(0) = X(3, 4); // t from element (3, 4)
    return xi;
}

//------------------------------------------------------------------------------
Gal3 Gal3::Expmap(const Vector10& xi, OptionalJacobian<10, 10> Hxi) {
    const Vector3 rho_tan = rho(xi);
    const Vector3 nu_tan = nu(xi);
    const Vector3 theta_tan = theta(xi);
    const double t_tan_val = t_tan(xi)(0);

    const Rot3 R = Rot3::Expmap(theta_tan);
    const Matrix3 Jl_theta = SO3_LeftJacobian(theta_tan);
    const Matrix3 E = Calculate_E(theta_tan);

    const Point3 r_final = Point3(Jl_theta * rho_tan + E * (t_tan_val * nu_tan));
    const Velocity3 v_final = Jl_theta * nu_tan;
    const double t_final = t_tan_val;

     if (Hxi) {
         // Jacobian placeholder - requires derivation
         *Hxi = ExpmapDerivative(xi);
     }

    return Gal3(R, r_final, v_final, t_final);
}

//------------------------------------------------------------------------------
Vector10 Gal3::Logmap(const Gal3& g, OptionalJacobian<10, 10> Hg) {
    const Vector3 theta_vec = Rot3::Logmap(g.R_);
    const Matrix3 Jl_theta_inv = SO3_LeftJacobianInverse(theta_vec);
    const Matrix3 E = Calculate_E(theta_vec);

    // CORRECTED: Cast r_ to Vector3 for subtraction
    const Vector3 r_vec = Vector3(g.r_);
    const Velocity3& v_vec = g.v_;
    const double& t_val = g.t_;

    const Vector3 nu_tan = Jl_theta_inv * v_vec;
    const Vector3 rho_tan = Jl_theta_inv * (r_vec - E * (t_val * nu_tan));
    const double t_tan_val = t_val;

    // Jacobian placeholder - requires derivation
    if (Hg) {
        *Hg = LogmapDerivative(g);
    }

    Vector10 xi;
    rho(xi) = rho_tan;
    nu(xi) = nu_tan;
    theta(xi) = theta_vec;
    t_tan(xi)(0) = t_tan_val;
    return xi;
}

//------------------------------------------------------------------------------
Matrix10 Gal3::AdjointMap() const {
    const Matrix3 Rmat = R_.matrix();
    const Matrix3 Vhat = skewSymmetric(v_);
    // CORRECTED: Cast r_ to Vector3 for subtraction
    const Vector3 r_minus_tv = Vector3(r_) - t_ * v_;
    const Matrix3 S_r_minus_tv_hat = skewSymmetric(r_minus_tv);

    Matrix10 Ad = Matrix10::Zero();

    // Row block 1: drho' output (maps input drho, dnu, dtheta, dt)
    Ad.block<3,3>(0,0) = Rmat;
    Ad.block<3,3>(0,3) = t_ * Rmat;
    Ad.block<3,3>(0,6) = -Rmat * S_r_minus_tv_hat;
    Ad.block<3,1>(0,9) = Rmat * v_;

    // Row block 2: dnu' output
    Ad.block<3,3>(3,3) = Rmat;
    Ad.block<3,3>(3,6) = -Rmat * Vhat;

    // Row block 3: dtheta' output
    Ad.block<3,3>(6,6) = Rmat;

    // Row block 4: dt' output
    Ad(9,9) = 1.0;

    return Ad;
}


//------------------------------------------------------------------------------
Vector10 Gal3::Adjoint(const Vector10& xi, OptionalJacobian<10, 10> H_g, OptionalJacobian<10, 10> H_xi) const {
    Matrix10 Ad = AdjointMap();
    if (H_xi) {
        *H_xi = Ad;
    }
    // Jacobian H_g requires adjointMap
    if (H_g) {
         *H_g = -adjointMap(Ad * xi);
    }
    return Ad * xi;
}

//------------------------------------------------------------------------------
Matrix10 Gal3::adjointMap(const Vector10& xi) {
    // Based on Manif SGal3Tangent::adj() / Lie bracket [xi, y]
    const Matrix3 Theta_hat = skewSymmetric(theta(xi));
    const Matrix3 Nu_hat = skewSymmetric(nu(xi));
    const Matrix3 Rho_hat = skewSymmetric(rho(xi));
    const double t_val = t_tan(xi)(0);

    Matrix10 ad = Matrix10::Zero();

    // Row block 1: rho_out = Theta_hat*m + Rho_hat*eta + t_val*p
    ad.block<3,3>(0,0) = Theta_hat;  // d/dm
    ad.block<3,3>(0,3) = t_val * Matrix3::Identity(); // d/dp
    ad.block<3,3>(0,6) = Rho_hat;    // d/deta

    // Row block 2: nu_out = Theta_hat*p + Nu_hat*eta
    ad.block<3,3>(3,3) = Theta_hat; // d/dp
    ad.block<3,3>(3,6) = Nu_hat;    // d/deta

    // Row block 3: theta_out = Theta_hat*eta
    ad.block<3,3>(6,6) = Theta_hat; // d/deta

    // Row block 4: t_out = 0 (already zero)

    return ad;
}

//------------------------------------------------------------------------------
Vector10 Gal3::adjoint(const Vector10& xi, const Vector10& y, OptionalJacobian<10, 10> Hxi, OptionalJacobian<10, 10> Hy) {
    Matrix10 ad_xi = adjointMap(xi);
    if (Hy) *Hy = ad_xi;
    // Jacobian Hxi requires adjointMap
    if (Hxi) {
         *Hxi = -adjointMap(y);
    }
    return ad_xi * y;
}


//------------------------------------------------------------------------------
Matrix10 Gal3::ExpmapDerivative(const Vector10& xi) {
    if (xi.norm() < kSmallAngleThreshold) return Matrix10::Identity();
    // CORRECTED: Explicitly create std::function to resolve ambiguity
    std::function<Gal3(const Vector10&)> fn =
        [](const Vector10& v) { return Gal3::Expmap(v); };
    // Note: Default delta for numericalDerivative might be too large or small
    return numericalDerivative11<Gal3, Vector10>(fn, xi, 1e-5); // Pass function object
}

//------------------------------------------------------------------------------
Matrix10 Gal3::LogmapDerivative(const Gal3& g) {
    Vector10 xi = Gal3::Logmap(g);
    if (xi.norm() < kSmallAngleThreshold) return Matrix10::Identity();
    // CORRECTED: Explicitly create std::function to resolve ambiguity
    std::function<Vector10(const Gal3&)> fn =
        [](const Gal3& g_in) { return Gal3::Logmap(g_in); };
    // Note: Default delta for numericalDerivative might be too large or small
    return numericalDerivative11<Vector10, Gal3>(fn, g, 1e-5); // Pass function object
}


//------------------------------------------------------------------------------
// ChartAtOrigin
//------------------------------------------------------------------------------
Gal3 Gal3::ChartAtOrigin::Retract(const Vector10& xi, ChartJacobian Hxi) {
  return Gal3::Expmap(xi, Hxi);
}

//------------------------------------------------------------------------------
Vector10 Gal3::ChartAtOrigin::Local(const Gal3& g, ChartJacobian Hg) {
  return Gal3::Logmap(g, Hg);
}

//------------------------------------------------------------------------------
// Stream operator
//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const Gal3& state) {
    os << "R: " << state.R_ << "\n";
    os << "r: " << state.r_.transpose() << "\n";
    os << "v: " << state.v_.transpose() << "\n";
    os << "t: " << state.t_;
    return os;
}


} // namespace gtsam
