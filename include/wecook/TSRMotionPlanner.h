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
                   const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                   const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                   const dart::dynamics::MetaSkeletonPtr &skeleton,
                   const std::shared_ptr<PreCondition> &condition = nullptr,
                   bool debug = false) : MotionPlanner(stateSpace, skeleton, condition),
                                         m_goalTSR(goalTSR),
                                         m_collisionFree(collisionFree),
                                         m_bn(bn),
                                         m_debug(debug) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  aikido::constraint::dart::CollisionFreePtr m_collisionFree = nullptr;
  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
  bool m_debug;
};

}

#endif //WECOOK_TSRMOTIONPLANNER_H
