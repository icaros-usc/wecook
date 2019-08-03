//
// Created by hejia on 8/2/19.
//

#ifndef PR_TSR_FORK_H
#define PR_TSR_FORsK_H

#include <Eigen/Core>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/math/math.hpp>

namespace wecook {
inline aikido::constraint::dart::TSR getDefaultSpoonTSR() {
  aikido::constraint::dart::TSR tsr;

  double forkLength = 0.15;
  // Transform w.r.t root
  tsr.mT0_w = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d orientation;
  orientation <<
      0, 0, 1,
      1, 0, 0,
      0, 1, 0;
  tsr.mT0_w.linear() = orientation;
  tsr.mT0_w.translation() = Eigen::Vector3d(0, -forkLength / 2.0, 0);

  // Transform between end effector and w
  tsr.mTw_e = Eigen::Isometry3d::Identity();
  tsr.mTw_e.translation() = Eigen::Vector3d(forkLength, 0, 0);
  Eigen::Matrix3d rot;
  rot <<
      1, 0, 0,
      0, 1, 0,
      0, 0, 1;
  tsr.mTw_e.linear() = rot;

  // Rotation around object
  tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
  double verticalTolerance = 0.02;
  tsr.mBw(2, 0) = -verticalTolerance;
  tsr.mBw(2, 1) = verticalTolerance;
  tsr.mBw(0, 0) = -0.1;
  tsr.mBw(0, 1) = 0.1;

  return tsr;
}
}

#endif //PR_TSR_FORK_H
