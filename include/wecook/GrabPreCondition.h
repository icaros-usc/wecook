//
// Created by hejia on 8/16/19.
//

#ifndef WECOOK_GRABPRECONDITION_H
#define WECOOK_GRABPRECONDITION_H

#include <Eigen/Core>
#include <dart/dynamics/MetaSkeleton.hpp>
#include <libada/AdaHand.hpp>

#include "PreCondition.h"

class GrabPreCondition : public PreCondition {
 public:
  GrabPreCondition(const dart::dynamics::SkeletonPtr &skeleton, const ada::AdaHandPtr &handSkeleton, bool shouldGrabbed) :
      m_skeleton(skeleton), m_handSkeleton(handSkeleton), m_shouldGrabbed(shouldGrabbed) {

  }

  bool isSatisfied() {
    return m_shouldGrabbed == !m_handSkeleton->isGrabbing(m_skeleton->getName());
  }

 private:
  ada::AdaHandPtr m_handSkeleton;
  dart::dynamics::MetaSkeletonPtr m_skeleton;
  bool m_shouldGrabbed;
};

#endif //WECOOK_GRABPRECONDITION_H
