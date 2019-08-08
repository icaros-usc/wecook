//
// Created by hejia on 8/2/19.
//

#ifndef PR_TSR_SPOON_H
#define PR_TSR_SPOON_H

#include <Eigen/Core>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/math/math.hpp>

namespace wecook {
inline aikido::constraint::dart::TSR getDefaultSpoonTSR() {
  aikido::constraint::dart::TSR tsr;

  double spoonHandleLength = 0.08;
  double spoonHandleRadius = 0.02;
  // Transform w.r.t root
  tsr.mT0_w = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d orientation;
  orientation <<
      1, 0, 0,
      0, 1, 0,
      0, 0, 1;
  tsr.mT0_w.linear() = orientation;
  tsr.mT0_w.translation() = Eigen::Vector3d(0, 0, spoonHandleLength / 2.0);

  // Transform between end effector and w
  tsr.mTw_e = Eigen::Isometry3d::Identity();
  tsr.mTw_e.translation() = Eigen::Vector3d(-spoonHandleRadius, 0, 0);
  Eigen::Matrix3d rot;
  rot <<
      0.866, 0, -5,
      0, 1, 0,
      0.5, 0, 0.866;
//  rot <<
//              1, 0, 0,
//      0, 1, 0,
//      0, 0, 1;
//  Eigen::Matrix3d rot2;
//  rot2 <<
//      -1, 0, 0,
//      0, -1, 0,
//      0, 0, 1;
//  tsr.mTw_e.linear() = rot2 * rot;
  tsr.mTw_e.linear() = rot;
  // Rotation around object
  tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
  double verticalTolerance = 0.02;
  tsr.mBw(2, 0) = -verticalTolerance;
  tsr.mBw(2, 1) = verticalTolerance;
//  tsr.mBw(0, 0) = -0.1;
//  tsr.mBw(0, 1) = 0.1;
  tsr.mBw(5, 0) = -M_PI;
  tsr.mBw(5, 1) = M_PI;
  tsr.mBw(4, 0) = -M_PI / 8;
  tsr.mBw(4, 1) = M_PI / 8;
  tsr.mBw(3, 0) = -M_PI / 8;
  tsr.mBw(3, 1) = M_PI / 8;

  return tsr;
}
}

#endif //PR_TSR_SPOON_H
