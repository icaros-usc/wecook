//
// Created by hejia on 8/11/19.
//

#ifndef WECOOK_TSRMOTIONWITHCONSTRAINTNODE_H
#define WECOOK_TSRMOTIONWITHCONSTRAINTNODE_H

#include "MotionNode.h"

namespace wecook {

class TSRMotionwithConstraintNode : public MotionNode {
 public:
  TSRMotionwithConstraintNode(const aikido::constraint::dart::TSRPtr &goalTSR,
                              const aikido::constraint::dart::TSRPtr &constraintTSR,
                              dart::dynamics::BodyNode *bn,
                              const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                              const dart::dynamics::MetaSkeletonPtr &skeleton,
                              bool debug = false) : MotionNode(stateSpace, skeleton),
                                                       m_goalTSR(goalTSR),
                                                       m_constraintTSR(constraintTSR),
                                                       m_collisionFree(collisionFree),
                                                       m_bn(bn),
                                                       m_debug(debug) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result = nullptr);

 private:
  aikido::constraint::dart::CollisionFreePtr m_collisionFree = nullptr;
  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
  aikido::constraint::dart::TSRPtr m_constraintTSR;
  bool m_debug;
};

}

#endif //WECOOK_TSRMOTIONWITHCONSTRAINTNODE_H
