//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_CONFMOTIONPLANNER_H
#define WECOOK_CONFMOTIONPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class ConfMotionPlanner : public MotionPlanner {
 public:
  ConfMotionPlanner(const Eigen::VectorXd &goalConf,
                    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                    const dart::dynamics::MetaSkeletonPtr &skeleton) : MotionPlanner(stateSpace, skeleton), m_goalConf(goalConf) {
  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  Eigen::VectorXd m_goalConf;
};
}

#endif //WECOOK_CONFMOTIONPLANNER_H
