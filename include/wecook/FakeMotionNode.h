//
// Created by hejia on 8/16/19.
//

#ifndef WECOOK_FAKEMOTIONNODE_H
#define WECOOK_FAKEMOTIONNODE_H

#include "MotionNode.h"

namespace wecook {

class FakeMotionNode : public MotionNode {
 public:
  FakeMotionNode(dart::dynamics::SkeletonPtr objSkeleton,
                 Eigen::Vector3d targetTranslation,
                 const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                 const dart::dynamics::MetaSkeletonPtr &skeleton,
                 const std::shared_ptr<PreCondition> &condition = nullptr) : MotionNode(stateSpace,
                                                                                           skeleton,
                                                                                           condition),
                                                                                m_objSkeleton(objSkeleton),
                                                                                m_targetTranslation(targetTranslation) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

 private:
  Eigen::Vector3d m_targetTranslation;
  dart::dynamics::SkeletonPtr m_objSkeleton = nullptr;
};

}

#endif //WECOOK_FAKEMOTIONNODE_H
