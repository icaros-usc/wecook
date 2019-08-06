//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_POSE_H
#define WECOOK_POSE_H

#include <Eigen/Core>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/math/math.hpp>

namespace wecook {
inline aikido::constraint::dart::TSR getDefaultPoseTSR() {
  aikido::constraint::dart::TSR tsr;

  // Transform w.r.t root
  tsr.mT0_w = Eigen::Isometry3d::Identity();
  tsr.mTw_e = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rot;
  rot <<
      0, 0, -1,
      0, 1, 0,
      1, 0, 0;
  tsr.mTw_e.linear() = rot;

//  // Transform between end effector and w
//  double verticalOffset = 0.15;
//  double horizontalOffset = 0.25;

//  tsr.mTw_e.translation() = Eigen::Vector3d{0, horizontalOffset, verticalOffset};

  // Rotation around object
  tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
  double verticalTolerance = 0.02;
  double xTolerance = 0.02;
  double yTolerance = 0.02;
  tsr.mBw(2, 0) = -verticalTolerance;
  tsr.mBw(2, 1) = verticalTolerance;
  tsr.mBw(0, 0) = -xTolerance;
  tsr.mBw(0, 1) = xTolerance;
  tsr.mBw(1, 0) = -yTolerance;
  tsr.mBw(1, 1) = yTolerance;

  return tsr;
}
};

#endif //WECOOK_POSE_H
