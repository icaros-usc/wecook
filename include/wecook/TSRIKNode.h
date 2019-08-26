//
// Created by hejia on 8/20/19.
//

#ifndef WECOOK_TSRIKNODE_H
#define WECOOK_TSRIKNODE_H

#include "MotionNode.h"

namespace wecook {

class TSRIKNode : public MotionNode {
 public:
  TSRIKNode(const aikido::constraint::dart::TSRPtr &goalTSR,
            dart::dynamics::BodyNode *bn,
            const aikido::constraint::dart::CollisionFreePtr &collisionFree,
            int interpolating_num,
            const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
            const dart::dynamics::MetaSkeletonPtr &skeleton,
            const std::shared_ptr<PreCondition> &condition = nullptr,
            bool debug = false) : MotionNode(stateSpace, skeleton, condition),
                                     m_goalTSR(goalTSR),
                                     m_bn(bn),
                                     m_debug(debug),
                                     m_interpolating_num(interpolating_num),
                                     m_collisionFree(collisionFree) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

 private:
  aikido::constraint::dart::CollisionFreePtr m_collisionFree = nullptr;
  int m_interpolating_num;
  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
  bool m_debug;
};

}

#endif //WECOOK_TSRIKNODE_H