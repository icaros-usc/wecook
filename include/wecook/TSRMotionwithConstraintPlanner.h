//
// Created by hejia on 8/11/19.
//

#ifndef WECOOK_TSRMOTIONWITHCONSTRAINTPLANNER_H
#define WECOOK_TSRMOTIONWITHCONSTRAINTPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class TSRMotionwithConstraintPlanner : public MotionPlanner {
 public:
  TSRMotionwithConstraintPlanner(const aikido::constraint::dart::TSRPtr &goalTSR,
                                 const aikido::constraint::dart::TSRPtr &constraintTSR,
                                 dart::dynamics::BodyNode *bn,
                                 const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                                 const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                                 const dart::dynamics::MetaSkeletonPtr &skeleton,
                                 bool debug = false) : MotionPlanner(stateSpace, skeleton),
                                                       m_goalTSR(goalTSR),
                                                       m_constraintTSR(constraintTSR),
                                                       m_collisionFree(collisionFree),
                                                       m_bn(bn),
                                                       m_debug(debug) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  aikido::constraint::dart::CollisionFreePtr m_collisionFree = nullptr;
  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
  aikido::constraint::dart::TSRPtr m_constraintTSR;
  bool m_debug;
};

}

#endif //WECOOK_TSRMOTIONWITHCONSTRAINTPLANNER_H
