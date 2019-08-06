//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_DELTAMOTIONPLANNER_H
#define WECOOK_DELTAMOTIONPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class DeltaMotionPlanner : public MotionPlanner {
 public:
  DeltaMotionPlanner(dart::dynamics::BodyNode *bn,
                     const Eigen::VectorXd &delta_x,
                     const dart::math::LinearJacobian &jac,
                     double repeat_time,
                     const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                     const dart::dynamics::MetaSkeletonPtr &skeleton) : MotionPlanner(stateSpace, skeleton),
                                                                 m_bn(bn),
                                                                 m_delta_x(delta_x),
                                                                 m_jac(jac),
                                                                 m_repeat_time(repeat_time) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  Eigen::VectorXd m_delta_x;
  dart::math::LinearJacobian m_jac;
  double m_repeat_time;
  dart::dynamics::BodyNode *m_bn = nullptr;
};

}

#endif //WECOOK_DELTAMOTIONPLANNER_H
