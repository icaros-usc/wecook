//
// Created by hejia on 8/5/19.
//

#ifndef WECOOK_FOOD_H
#define WECOOK_FOOD_H

#include <Eigen/Core>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/math/math.hpp>

namespace wecook {
inline aikido::constraint::dart::TSR getDefaultFoodTSR() {
  aikido::constraint::dart::TSR tsr;

  double foodHeight = 0.045;
  double foodWidth = 0.1;
  // Transform w.r.t root
  tsr.mT0_w = Eigen::Isometry3d::Identity();
//  tsr.mT0_w.translation() = Eigen::Vector3d(0, 0, 0)
  tsr.mTw_e = Eigen::Isometry3d::Identity();
  tsr.mTw_e.translation() = Eigen::Vector3d(0, 0, foodHeight / 2.0);

  // Rotation around object
  tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
//  tsr.mBw(5, 0) = -M_PI;
//  tsr.mBw(5, 1) = M_PI;

  return tsr;
}
};

#endif //WECOOK_FOOD_H
