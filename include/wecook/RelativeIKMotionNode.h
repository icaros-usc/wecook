//
// Created by hejia on 8/25/19.
//

#ifndef WECOOK_RELATIVEIKMOTIONNODE_H
#define WECOOK_RELATIVEIKMOTIONNODE_H

#include <dart/dynamics/Frame.hpp>

#include "MotionNode.h"

namespace wecook {

class RelativeIKMotionNode : public MotionNode {
 public:
  RelativeIKMotionNode(dart::dynamics::BodyNode *bn,
                       const Eigen::Isometry3d &relT,
                       dart::dynamics::Frame *incoordinatesOf,
                       const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                       const dart::dynamics::MetaSkeletonPtr &skeleton,
                       const std::shared_ptr<PreCondition> &condition = nullptr) : MotionNode(stateSpace,
                                                                                              skeleton,
                                                                                              condition),
                                                                                   m_bn(bn),
                                                                                   m_relT(relT),
                                                                                   m_incoordinatesOf(incoordinatesOf) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

 private:
  Eigen::Isometry3d m_relT; // relative transformation matrix
  dart::dynamics::BodyNode *m_bn = nullptr;
  dart::dynamics::Frame *m_incoordinatesOf = nullptr;
};

}

#endif //WECOOK_RELATIVEIKMOTIONNODE_H
