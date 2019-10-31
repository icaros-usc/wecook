//
// Created by hejia on 8/16/19.
//

#include "wecook/FakeMotionNode.h"

using namespace wecook;

void FakeMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg) {
  auto translation = m_objSkeleton->getBodyNode(0)->getTransform().translation();
  while (translation != m_targetTranslation) {
    auto transform = m_objSkeleton->getBodyNode(0)->getTransform();
    transform.translation() = translation + (m_targetTranslation - translation);
    dynamic_cast<dart::dynamics::FreeJoint *>(m_objSkeleton->getJoint(0))
        ->setTransform(transform);
    ros::Duration(0.5).sleep();
    translation = m_objSkeleton->getBodyNode(0)->getTransform().translation();
  }
}