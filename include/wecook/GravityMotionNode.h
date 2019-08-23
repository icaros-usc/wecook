//
// Created by hejia on 8/15/19.
//

#ifndef WECOOK_GRAVITYMOTIONNODE_H
#define WECOOK_GRAVITYMOTIONNODE_H

#include "MotionNode.h"

namespace wecook {

class GravityMotionNode : public MotionNode {
 public:
  GravityMotionNode(dart::dynamics::SkeletonPtr objSkeleton,
                    double repeat_time,
                    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                    const dart::dynamics::MetaSkeletonPtr &skeleton,
                    const std::shared_ptr<PreCondition> &condition = nullptr) : MotionNode(stateSpace,
                                                                                              skeleton,
                                                                                              condition),
                                                                                   m_objSkeleton(objSkeleton),
                                                                                   m_repeat_time(repeat_time) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

 private:
  double m_repeat_time;
  dart::dynamics::SkeletonPtr m_objSkeleton = nullptr;
};

}

#endif //WECOOK_GRAVITYMOTIONNODE_H
