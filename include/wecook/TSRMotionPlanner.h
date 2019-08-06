//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_TSRMOTIONPLANNER_H
#define WECOOK_TSRMOTIONPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class TSRMotionPlanner : public MotionPlanner {
 public:
  TSRMotionPlanner(const aikido::constraint::dart::TSRPtr &goalTSR,
                   dart::dynamics::BodyNode *bn,
                   const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                   const dart::dynamics::MetaSkeletonPtr &skeleton) : MotionPlanner(stateSpace, skeleton),
                                                               m_goalTSR(goalTSR),
                                                               m_bn(bn) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
};

}

#endif //WECOOK_TSRMOTIONPLANNER_H
