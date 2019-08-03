//
// Created by hejia on 8/2/19.
//

#ifndef PR_TSR_BOWL_H
#define PR_TSR_BOWL_H

#include <Eigen/Core>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/math/math.hpp>

namespace wecook {
  inline aikido::constraint::dart::TSR getDefaultBowlTSR() {
    aikido::constraint::dart::TSR tsr;

    // Transform w.r.t root
    tsr.mT0_w = Eigen::Isometry3d::Identity();
    tsr.mTw_e = Eigen::Isometry3d::Identity();

    // Transform between end effector and w
    double verticalOffset = 0.15;
    double horizontalOffset = 0.25;

    tsr.mTw_e.translation() = Eigen::Vector3d{0, horizontalOffset, verticalOffset};

    // Rotation around object
    tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
    tsr.mBw(5, 0) = -M_PI;
    tsr.mBw(5, 1) = M_PI;

    return tsr;
  }
};

#endif //PR_TSR_BOWL_H
