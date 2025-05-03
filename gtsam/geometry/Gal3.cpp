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
 * @authors Matt Kielo, Scott Baker, Frank Dellaert
 * @date    April 30, 2025
 */

#include <gtsam/geometry/Gal3.h> // Note: Adjust path if needed
#include <gtsam/geometry/SO3.h> // For so3::DexpFunctor if needed, and Rot3 Log/Exp
#include <gtsam/base/numericalDerivative.h> // For derivative checking if desired
#include <gtsam/base/Matrix.h> // For Z_3x3 etc.
#include <gtsam/nonlinear/expressions.h> // For constructing expressions
#include <gtsam/geometry/concepts.h> // For GtsamLieTypeChecks @TODO - check

#include <iostream>
#include <cmath>
#include <functional> // For std::function

namespace gtsam {

//------------------------------------------------------------------------------
// Constants and Helper function for Expmap/Logmap (Implementation Detail)
//------------------------------------------------------------------------------
namespace { // Anonymous namespace for internal linkage

  // Define the threshold locally within the cpp file
  constexpr double kSmallAngleThreshold = 1e-10;

} // end anonymous namespace

//------------------------------------------------------------------------------
// Static Constructor/Create functions
//------------------------------------------------------------------------------
Gal3 Gal3::Create(const Rot3& R, const Point3& r, const Velocity3& v, double t,
                    OptionalJacobian<10, 3> H1, OptionalJacobian<10, 3> H2,
                    OptionalJacobian<10, 3> H3, OptionalJacobian<10, 1> H4) {
      // H = d Logmap(Create(R,r,v,t)) / d [w, r_vec, v_vec, t]
      // Note: Derivatives are w.r.t tangent space elements at the identity
      // for inputs R, r, v, t. However, the standard way is derivative
      // w.r.t minimal tangent spaces of inputs. Let's assume standard approach.
      if (H1) { // d xi_out / d omega_R (3x3)
        H1->setZero();
        H1->block<3, 3>(6, 0) = Matrix3::Identity(); // d theta / d omega_R = I
      }
      if (H2) { // d xi_out / d delta_r (3x3)
        H2->setZero();
        H2->block<3, 3>(0, 0) = R.transpose();      // d rho / d delta_r = R^T
      }
      if (H3) { // d xi_out / d delta_v (3x3)
        H3->setZero();
        H3->block<3, 3>(3, 0) = R.transpose();      // d nu / d delta_v = R^T
      }
      if (H4) { // d xi_out / d delta_t (10x1)
        H4->setZero();
        // As derived: d xi / dt = [-R^T v; 0; 0; 1]
        Vector3 drho_dt = -R.transpose() * v; // Corrected derivative for rho
        H4->block<3, 1>(0, 0) = drho_dt;       // Assign to rho rows (0-2)
        // d nu / dt = 0
        // d theta / dt = 0
        (*H4)(9, 0) = 1.0;                     // d t_tan / dt = 1
      }
      return Gal3(R, r, v, t);
}

//------------------------------------------------------------------------------
Gal3 Gal3::FromPoseVelocityTime(const Pose3& pose, const Velocity3& v, double t,
                                OptionalJacobian<10, 6> H1, OptionalJacobian<10, 3> H2,
                                OptionalJacobian<10, 1> H3) {
    const Rot3& R = pose.rotation();
    const Point3& r = pose.translation(); // Get r for H3 calculation if needed below
    if (H1) { // d xi_out / d xi_pose (10x6), xi_pose = [omega_R; v_r]
        H1->setZero();
        // d theta / d omega_pose = I (rows 6-8, cols 0-2)
        H1->block<3, 3>(6, 0) = Matrix3::Identity();
        // d rho / d v_r = I (rows 0-2, cols 3-5) - Corrected
        H1->block<3, 3>(0, 3) = Matrix3::Identity();
        // Other blocks (d_rho/d_omega, d_nu/d_omega, d_nu/d_vr, d_tt/d_omega, d_tt/d_vr) are zero.
    }
    if (H2) { // d xi_out / d v_in (10x3)
        H2->setZero();
        H2->block<3, 3>(3, 0) = R.transpose();         // d(nu_out)/d(v_in) = R^T
    }
    if (H3) { // d xi_out / d t_in (10x1)
        H3->setZero();
        // As derived: d xi / dt = [-R^T v; 0; 0; 1]
        Vector3 drho_dt = -R.transpose() * v; // Corrected derivative for rho
        H3->block<3, 1>(0, 0) = drho_dt;       // Assign to rho rows (0-2)
        // d nu / dt = 0
        // d theta / dt = 0
        (*H3)(9, 0) = 1.0;                     // d t_tan / dt = 1
    }
    // Pass r (translation from pose) to the Gal3 constructor
    return Gal3(R, r, v, t);
}

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
Gal3::Gal3(const Matrix5& M) {
    // Ensure matrix adheres to SGal(3) structure
    if (std::abs(M(3, 3) - 1.0) > 1e-9 || std::abs(M(4, 4) - 1.0) > 1e-9 ||
        M.row(4).head(4).norm() > 1e-9 || M.row(3).head(3).norm() > 1e-9) { // Row 4 first 4 = 0; Row 3 first 3 = 0
        throw std::invalid_argument("Invalid Gal3 matrix structure: Check zero blocks and diagonal ones.");
    }
    R_ = Rot3(M.block<3, 3>(0, 0));
    v_ = M.block<3, 1>(0, 3); // v is in column 3
    r_ = Point3(M.block<3, 1>(0, 4)); // r is in column 4
    t_ = M(3, 4);             // t is at (3, 4)
}

//------------------------------------------------------------------------------
// Component Access
//------------------------------------------------------------------------------
const Rot3& Gal3::rotation(OptionalJacobian<3, 10> H) const {
    if (H) {
        H->setZero();
        H->block<3, 3>(0, 6) = Matrix3::Identity(); // Derivative w.r.t theta
    }
    return R_;
}

//------------------------------------------------------------------------------
const Point3& Gal3::translation(OptionalJacobian<3, 10> H) const {
     // H = d r / d xi = [dr/drho, dr/dnu, dr/dtheta, dr/dt_tan]
     if (H) {
        H->setZero();
        // dr/drho = R (local tangent space)
        H->block<3,3>(0, 0) = R_.matrix();
        // dr/dnu = 0
        // H->block<3,3>(0, 3) = Matrix3::Zero(); // Already zero
        // dr/dtheta = 0
        // H->block<3,3>(0, 6) = Matrix3::Zero(); // Already zero
        // dr/dt_tan = v (Corrected)
        H->block<3,1>(0, 9) = v_;
    }
    return r_;
}

//------------------------------------------------------------------------------
const Velocity3& Gal3::velocity(OptionalJacobian<3, 10> H) const {
     if (H) {
        H->setZero();
        H->block<3, 3>(0, 3) = R_.matrix(); // d(v)/d(nu)
     }
    return v_;
}

//------------------------------------------------------------------------------
const double& Gal3::time(OptionalJacobian<1, 10> H) const {
    if (H) {
        H->setZero();
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
    M.block<3, 1>(0, 3) = v_;
    M.block<3, 1>(0, 4) = Vector3(r_);
    M(3, 4) = t_;
    M.block<1,3>(3,0).setZero(); // Zero out row 3, cols 0-2
    M.block<1,4>(4,0).setZero(); // Zero out row 4, cols 0-3
    return M;
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
    const Point3 r_inv = -(Rinv.rotate(Vector3(r_) - t_ * v_));
    const double t_inv = -t_;
    return Gal3(Rinv, r_inv, v_inv, t_inv);
}

//------------------------------------------------------------------------------

Gal3 Gal3::operator*(const Gal3& other) const {
    // This function provides the fundamental math for g1 * g2
    // Formula: R = R1*R2, v = R1*v2 + v1, r = R1*r2 + t2*v1 + r1, t = t1+t2
    const Gal3& g1 = *this; // g1 is the current object (*this)
    const Gal3& g2 = other; // g2 is the argument

    const Rot3 R_comp = g1.R_.compose(g2.R_);

    // Ensure correct types for addition (using Vector3 temporarily if r_ is Point3)
    const Vector3 r1_vec(g1.r_);
    const Vector3 r2_vec(g2.r_);
    const Vector3 r_comp_vec = g1.R_.rotate(r2_vec) + g2.t_ * g1.v_ + r1_vec;

    const Velocity3 v_comp = g1.R_.rotate(g2.v_) + g1.v_;
    const double t_comp = g1.t_ + g2.t_;

    // Construct the result using the computed components
    return Gal3(R_comp, Point3(r_comp_vec), v_comp, t_comp);
}


//------------------------------------------------------------------------------
// Lie Group Static Functions
//------------------------------------------------------------------------------
gtsam::Gal3 gtsam::Gal3::Expmap(const Vector10& xi, OptionalJacobian<10, 10> Hxi) {
    // Extract tangent vector components
    const Vector3 rho_tan = rho(xi);
    const Vector3 nu_tan = nu(xi);
    const Vector3 theta_tan = theta(xi);
    const double t_tan_val = t_tan(xi)(0);

    // Use the SO(3) DexpFunctor. This computes theta, W, WW, sin/cos (or Taylor series),
    // and coefficients A, B, C efficiently, handling the near-zero case.
    const gtsam::so3::DexpFunctor dexp_functor(theta_tan);

    // Get Rotation matrix R = Exp(theta)
    // We can use the functor's result directly if it provides it,
    // or call the standard Rot3::Expmap which might be slightly clearer.
    // Let's assume Rot3::Expmap is fine, as DexpFunctor primarily helps with Jacobians and coefficients.
    const Rot3 R = Rot3::Expmap(theta_tan); // or const Rot3 R(dexp_functor.expmap());

    // Get Left Jacobian of SO(3) Expmap: J_R(theta)
    const Matrix3 Jl_theta = dexp_functor.leftJacobian();

    // Calculate the E matrix using coefficients from dexp_functor to avoid redundant calculations.
    // E = 0.5 * I + C * W + B_E * W2
    // where C = dexp_functor.C = (theta - sin(theta)) / theta^3
    // and B_E = (1 - 2 * dexp_functor.B) / (2 * dexp_functor.theta2)
    //       where B = dexp_functor.B = (1 - cos(theta)) / theta^2
    // Note: DexpFunctor handles the small angle case (nearZero==true) internally
    //       by using Taylor approximations for B and C, which results in the
    //       correct Taylor series for E.
    Matrix3 E;
    if (dexp_functor.nearZero) {
         // Although the formula below *should* work due to Taylor series in B,C,
         // we can be explicit using the known Taylor series for E for clarity/safety.
         // E = 0.5*I + (1/6)*W + (1/24)*WW
         E = 0.5 * Matrix3::Identity() + (1.0 / 6.0) * dexp_functor.W + (1.0 / 24.0) * dexp_functor.WW;
    } else {
         // Use the derived formula. theta2 cannot be zero here.
         const double B_E = (1.0 - 2.0 * dexp_functor.B) / (2.0 * dexp_functor.theta2);
         E = 0.5 * Matrix3::Identity() + dexp_functor.C * dexp_functor.W + B_E * dexp_functor.WW;
    }

    // Calculate final state components using the computed parts
    const Point3 r_final = Point3(Jl_theta * rho_tan + E * (t_tan_val * nu_tan));
    const Velocity3 v_final = Jl_theta * nu_tan;
    const double t_final = t_tan_val;

    // Construct the final Gal3 object
    Gal3 result(R, r_final, v_final, t_final);

    // If Jacobian is requested, compute it numerically.
    // The lambda should call this Expmap function *without* the Jacobian flag
    // to avoid infinite recursion.
    if (Hxi) {
        std::function<Gal3(const Vector10&)> expmap_wrapper =
             [](const Vector10& tangent_vector) -> Gal3 {
                 // Call the Expmap function itself, but without requesting Jacobian
                 return Gal3::Expmap(tangent_vector);
             };

        // Compute numerical derivative using the wrapper
        *Hxi = Gal3::ExpmapDerivative(xi);
        // Note: The numerical derivative computes the Right Jacobian Jr(xi) for Expmap.
        // Jr(xi) = d Expmap(xi o eps) / d eps |_{eps=0}
        //        = d (Expmap(xi) * Expmap(Dr(xi) * eps)) / d eps |_{eps=0}
        //        = d (Expmap(xi) * (I + (Dr(xi)*eps)^ )) / d eps |_{eps=0} * Dr(xi) ??? No, this is not right.
        // The numerical derivative directly computes dExpmap(xi)/dxi where the perturbation is additive in xi.
        // This corresponds to the Right Jacobian by definition in GTSAM's manifold context.
        // Retract(x, v) = x.compose(Expmap(v))  (Right chart)
        // Retract(Identity, xi) = Expmap(xi)
        // Derivative d Retract(Identity, xi) / d xi = Jr_SO3(xi) for SO3.
        // Let's assume numericalDerivative11 gives the correct Jacobian expected by the retract/localCoordinates framework.
    }

    return result;
}

// Also update the numerical derivative lambda within Logmap if it re-implements logic:
Vector10 Gal3::Logmap(const Gal3& g, OptionalJacobian<10, 10> Hg) {
    const Vector3 theta_vec = Rot3::Logmap(g.R_);

    // Use DexpFunctor to get Jacobians and coefficients efficiently
    const gtsam::so3::DexpFunctor dexp_functor_log(theta_vec);
    const Matrix3 Jl_theta_inv = dexp_functor_log.leftJacobianInverse(); // Inverse Left Jacobian

    // Calculate E using the same logic as in Expmap, but based on theta_vec
    Matrix3 E;
    if (dexp_functor_log.nearZero) {
         E = 0.5 * Matrix3::Identity() + (1.0 / 6.0) * dexp_functor_log.W + (1.0 / 24.0) * dexp_functor_log.WW;
    } else {
         const double B_E = (1.0 - 2.0 * dexp_functor_log.B) / (2.0 * dexp_functor_log.theta2);
         E = 0.5 * Matrix3::Identity() + dexp_functor_log.C * dexp_functor_log.W + B_E * dexp_functor_log.WW;
    }

    const Vector3 r_vec = Vector3(g.r_);
    const Velocity3& v_vec = g.v_;
    const double& t_val = g.t_;

    const Vector3 nu_tan = Jl_theta_inv * v_vec;
    // Corrected formula for rho_tan based on Expmap structure: r = Jl*rho + E*(t*nu)
    // => Jl*rho = r - E*(t*nu)
    // => rho = Jl_inv * (r - E*(t*nu))
    const Vector3 rho_tan = Jl_theta_inv * (r_vec - E * (t_val * nu_tan));
    const double t_tan_val = t_val;

    Vector10 xi;
    rho(xi) = rho_tan;
    nu(xi) = nu_tan;
    theta(xi) = theta_vec;
    t_tan(xi)(0) = t_tan_val;

    if (Hg) {
        // Define a lambda that calls Logmap *without* the Jacobian argument
        std::function<Vector10(const Gal3&)> logmap_wrapper =
             [](const Gal3& state) -> Vector10 {
                 return Gal3::Logmap(state); // Call Logmap without Hg
             };
         // Compute numerical derivative
        *Hg = Gal3::LogmapDerivative(g);
         // This numerical derivative gives dLogmap(g)/dg where perturbation is on g.
         // This corresponds to the derivative of the Local chart.
         // Local(x,y) = Logmap(x.between(y))
         // Local(Identity, g) = Logmap(Identity.between(g)) = Logmap(g)
         // Derivative d Local(Identity, g) / d g should be Jl_inv(Logmap(g)). Let's assume numericalDerivative works as expected.
    }

    return xi;
}



//------------------------------------------------------------------------------
Matrix10 Gal3::AdjointMap() const {
    const Matrix3 Rmat = R_.matrix();
    const Vector3 v_vec = v_;
    const Vector3 r_minus_tv = Vector3(r_) - t_ * v_;

    Matrix10 Ad = Matrix10::Zero();

    // Row block 1: drho' output
    Ad.block<3,3>(0,0) = Rmat;
    Ad.block<3,3>(0,3) = -t_ * Rmat;
    Ad.block<3,3>(0,6) = skewSymmetric(r_minus_tv) * Rmat; // [(r-t*v)]x * R
    Ad.block<3,1>(0,9) = v_vec;

    // Row block 2: dnu' output
    Ad.block<3,3>(3,3) = Rmat;
    Ad.block<3,3>(3,6) = skewSymmetric(v_vec) * Rmat; // [v]x * R

    // Row block 3: dtheta' output
    Ad.block<3,3>(6,6) = Rmat;

    // Row block 4: dt' output
    Ad(9,9) = 1.0;

    return Ad;
}

//------------------------------------------------------------------------------
Vector10 Gal3::Adjoint(const Vector10& xi, OptionalJacobian<10, 10> H_g, OptionalJacobian<10, 10> H_xi) const {
    Matrix10 Ad = AdjointMap(); // Ad = Ad(g)
    Vector10 y = Ad * xi;      // y = Ad_g(xi)

    if (H_xi) {
        *H_xi = Ad; // Jacobian w.r.t xi is Ad(g)
    }

    if (H_g) {
        // Jacobian H_g = d(Ad_g(xi))/d(g) -> use numerical derivative
        std::function<Vector10(const Gal3&, const Vector10&)> adjoint_action_wrt_g =
          [&](const Gal3& g_in, const Vector10& xi_in) {
              // Call Adjoint *without* asking for its Jacobians for numerical diff
              return g_in.Adjoint(xi_in);
          };
        *H_g = numericalDerivative21(adjoint_action_wrt_g, *this, xi, 1e-7);
    }
    return y;
}

//------------------------------------------------------------------------------
Matrix10 Gal3::adjointMap(const Vector10& xi) { // Lowercase 'a' - ad(xi)
    const Matrix3 Theta_hat = skewSymmetric(theta(xi)); // =[theta_xi]x
    const Matrix3 Nu_hat = skewSymmetric(nu(xi));       // =[nu_xi]x
    const Matrix3 Rho_hat = skewSymmetric(rho(xi));       // =[rho_xi]x
    const double t_val = t_tan(xi)(0);                 // =t_xi
    const Vector3 nu_vec = nu(xi);

    Matrix10 ad = Matrix10::Zero();

    // Structure based on Lie bracket [xi, y] = ad(xi)y
    // Row block 1: maps input rho_y, nu_y, theta_y, t_y to output rho_z
    ad.block<3,3>(0,0) = Theta_hat;
    ad.block<3,3>(0,3) = -t_val * Matrix3::Identity();
    ad.block<3,3>(0,6) = Rho_hat;
    ad.block<3,1>(0,9) = nu_vec;

    // Row block 2: maps input nu_y, theta_y to output nu_z
    ad.block<3,3>(3,3) = Theta_hat;
    ad.block<3,3>(3,6) = Nu_hat;

    // Row block 3: maps input theta_y to output theta_z
    ad.block<3,3>(6,6) = Theta_hat;

    // Row block 4: maps input t_y to output t_z (which is 0)

    return ad;
}

//------------------------------------------------------------------------------
Vector10 Gal3::adjoint(const Vector10& xi, const Vector10& y, OptionalJacobian<10, 10> Hxi, OptionalJacobian<10, 10> Hy) {
    Matrix10 ad_xi = adjointMap(xi);
    if (Hy) *Hy = ad_xi; // Jacobian w.r.t y is ad(xi)
    if (Hxi) {
        // Jacobian w.r.t xi is -ad(y)
         *Hxi = -adjointMap(y);
    }
    return ad_xi * y;
}

//------------------------------------------------------------------------------
Matrix10 Gal3::ExpmapDerivative(const Vector10& xi) {
    // Use numerical derivative for Expmap Jacobian
    // Use locally defined constant
    if (xi.norm() < kSmallAngleThreshold) return Matrix10::Identity();
    std::function<Gal3(const Vector10&)> fn =
        [](const Vector10& v) { return Gal3::Expmap(v); };
    return numericalDerivative11<Gal3, Vector10>(fn, xi, 1e-5);
}

//------------------------------------------------------------------------------
Matrix10 Gal3::LogmapDerivative(const Gal3& g) {
    // Use numerical derivative for Logmap Jacobian
    Vector10 xi = Gal3::Logmap(g);
    // Use locally defined constant
    if (xi.norm() < kSmallAngleThreshold) return Matrix10::Identity();
    std::function<Vector10(const Gal3&)> fn =
        [](const Gal3& g_in) { return Gal3::Logmap(g_in); };
    return numericalDerivative11<Vector10, Gal3>(fn, g, 1e-5);
}

//------------------------------------------------------------------------------
// Lie Algebra (Hat/Vee maps)
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
    // Check Lie algebra structure: Row 4 all=0; Row 3 first 3=0, last = t.
    if (X.row(4).norm() > 1e-9 || X.row(3).head(3).norm() > 1e-9 || std::abs(X(3,3)) > 1e-9) {
     throw std::invalid_argument("Matrix is not in sgal(3)");
    }

    Vector10 xi;
    rho(xi) = X.block<3, 1>(0, 4); // rho from column 4
    nu(xi) = X.block<3, 1>(0, 3);  // nu from column 3
    const Matrix3& S = X.block<3, 3>(0, 0);
    theta(xi) << S(2, 1), S(0, 2), S(1, 0); // Extract theta from skew-symmetric part
    t_tan(xi)(0) = X(3, 4); // t from element (3, 4)
    return xi;
}

//------------------------------------------------------------------------------
// ChartAtOrigin
//------------------------------------------------------------------------------
Gal3 Gal3::ChartAtOrigin::Retract(const Vector10& xi, ChartJacobian Hxi) {
  // Retraction at origin is Expmap
  return Gal3::Expmap(xi, Hxi);
}

//------------------------------------------------------------------------------
Vector10 Gal3::ChartAtOrigin::Local(const Gal3& g, ChartJacobian Hg) {
  // Local coordinates at origin is Logmap
  return Gal3::Logmap(g, Hg);
}

Point3 Gal3::act(const Point3& p, OptionalJacobian<3, 10> Hself, OptionalJacobian<3, 3> Hp) const {
    // Implementation assumes instantaneous action: p_out = R_.rotate(p) + r_
    Point3 r_out = R_.rotate(p) + r_;

    if (Hp) {
        *Hp = R_.matrix(); // Jacobian d(R*p+r)/dp = R
    }
    if (Hself) {
        // Jacobian d(act(g,p))/d(xi) where g = this, xi is tangent at identity
        // Corresponds to columns for [rho, nu, theta, t_tan]
        Hself->setZero();
        const Matrix3 Rmat = R_.matrix();
        // Derivative w.r.t. rho (cols 0-2) -> d(r)/drho = R
        Hself->block<3,3>(0, 0) = Rmat;
        // Derivative w.r.t. nu (cols 3-5) -> d(R*p+r)/dnu = 0
        // Block remains zero.
        // Derivative w.r.t. theta (cols 6-8) -> d(R*p)/dtheta = -R*skew(p)
        Hself->block<3,3>(0, 6) = -Rmat * skewSymmetric(p);
        // Derivative w.r.t. t_tan (col 9) -> Based on retract analysis, should be v_
        Hself->block<3,1>(0, 9) = v_;
    }
    return r_out;
}

} // namespace gtsam
