//
// Created by hejia on 8/15/19.
//

#ifndef WECOOK_IKMOTIONNODE_H
#define WECOOK_IKMOTIONNODE_H

#include <dart/dynamics/Frame.hpp>

#include "MotionNode.h"

namespace wecook {

class IKMotionNode : public MotionNode {
 public:
  IKMotionNode(dart::dynamics::BodyNode *bn,
               const Eigen::Isometry3d &targetPose,
               dart::dynamics::Frame *incoordinatesOf,
               const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
               const dart::dynamics::MetaSkeletonPtr &skeleton,
               const std::shared_ptr<PreCondition> &condition = nullptr) : MotionNode(stateSpace,
                                                                                         skeleton,
                                                                                         condition),
                                                                              m_bn(bn),
                                                                              m_targetPose(targetPose),
                                                                              m_incoordinatesOf(incoordinatesOf) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg);

 private:
  Eigen::Isometry3d m_targetPose;
  dart::dynamics::BodyNode *m_bn = nullptr;
  dart::dynamics::Frame *m_incoordinatesOf = nullptr;
};

}

#endif //WECOOK_IKMOTIONNODE_H
