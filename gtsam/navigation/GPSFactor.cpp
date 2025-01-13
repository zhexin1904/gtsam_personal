/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for GPS factor
 *  @date   January 28, 2014
 **/

#include "GPSFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void GPSFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "GPSFactor on " << keyFormatter(key())
       << "\n";
  cout << "  GPS measurement: " << nT_.transpose() << "\n";
  cout << "  Lever arm: " << B_t_BG_.transpose() << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol) &&
         traits<Point3>::Equals(B_t_BG_, e->B_t_BG_, tol);
}

//***************************************************************************
Vector GPSFactor::evaluateError(const Pose3& p,
    OptionalMatrixType H) const {
  const Matrix3 rot = p.rotation().matrix();
  if (H) {
    H->resize(3, 6);

    H->block<3, 3>(0, 0) = -rot * skewSymmetric(B_t_BG_);
    H->block<3, 3>(0, 3) = rot;
  }

  return p.translation() - (nT_ - rot * B_t_BG_);
}

//***************************************************************************
pair<Pose3, Vector3> GPSFactor::EstimateState(double t1, const Point3& NED1,
    double t2, const Point3& NED2, double timestamp) {
  // Estimate initial velocity as difference in NED frame
  double dt = t2 - t1;
  Point3 nV = (NED2 - NED1) / dt;

  // Estimate initial position as linear interpolation
  Point3 nT = NED1 + nV * (timestamp - t1);

  // Estimate Rotation
  double yaw = atan2(nV.y(), nV.x());
  Rot3 nRy = Rot3::Yaw(yaw); // yaw frame
  Point3 yV = nRy.inverse() * nV; // velocity in yaw frame
  double pitch = -atan2(yV.z(), yV.x()), roll = 0;
  Rot3 nRb = Rot3::Ypr(yaw, pitch, roll);

  // Construct initial pose
  Pose3 nTb(nRb, nT); // nTb

  return make_pair(nTb, nV);
}
//***************************************************************************
void GPSFactor2::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactor2 on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_.transpose() << "\n";
  cout << "  Lever arm: " << B_t_BG_.transpose() << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor2::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol) &&
         traits<Point3>::Equals(B_t_BG_, e->B_t_BG_, tol);
}

//***************************************************************************
Vector GPSFactor2::evaluateError(const NavState& p,
    OptionalMatrixType H) const {
  const Matrix3 rot = p.attitude().matrix();
  if (H) {
    H->resize(3, 9);

    H->block<3, 3>(0, 0) = -rot * skewSymmetric(B_t_BG_);
    H->block<3, 3>(0, 3) = rot;
    H->block<3, 3>(0, 6).setZero();
  }

  return p.position() - (nT_ - rot * B_t_BG_);
}

//***************************************************************************

}/// namespace gtsam
