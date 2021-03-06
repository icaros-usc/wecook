//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_GRABMOTIONNODE_H
#define WECOOK_GRABMOTIONNODE_H
#include "MotionNode.h"

namespace wecook {

class GrabMotionNode : public MotionNode {
 public:
  GrabMotionNode(const dart::dynamics::SkeletonPtr &bodyToGrab,
                 bool grab,
                 const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                 const dart::dynamics::MetaSkeletonPtr &skeleton) : MotionNode(stateSpace,
                                                                               skeleton),
                                                                    m_bodyToGrab(bodyToGrab),
                                                                    m_grab(grab) {
  }

  void plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result = nullptr);

 private:
  dart::dynamics::SkeletonPtr m_bodyToGrab;
  bool m_grab;
};

}

#endif //WECOOK_GRABMOTIONNODE_H
