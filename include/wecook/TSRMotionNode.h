//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_TSRMOTIONNODE_H
#define WECOOK_TSRMOTIONNODE_H

#include "MotionNode.h"

namespace wecook {

class TSRMotionNode : public MotionNode {
 public:
  TSRMotionNode(const aikido::constraint::dart::TSRPtr &goalTSR,
                dart::dynamics::BodyNode *bn,
                const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                const dart::dynamics::MetaSkeletonPtr &skeleton,
                const std::shared_ptr<PreCondition> &condition = nullptr,
                bool debug = false) : MotionNode(stateSpace, skeleton, condition),
                                      m_goalTSR(goalTSR),
                                      m_collisionFree(collisionFree),
                                      m_bn(bn),
                                      m_debug(debug) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  void setTimeStep(double timeStep) {
    m_retimeTimeStep = timeStep;
  }

 private:
  aikido::constraint::dart::CollisionFreePtr m_collisionFree = nullptr;
  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
  bool m_debug;
  double m_retimeTimeStep = 0.02;
};

}

#endif //WECOOK_TSRMOTIONNODE_H
