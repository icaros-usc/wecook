//
// Created by hejia on 8/16/19.
//

#include "wecook/FakeMotionPlanner.h"

using namespace wecook;

void FakeMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  auto translation = m_objSkeleton->getBodyNode(0)->getTransform().translation();
  std::cout << "Object's current translation: " << translation << "Target translation: " << m_targetTranslation << std::endl;
  while (translation != m_targetTranslation) {
    auto transform = m_objSkeleton->getBodyNode(0)->getTransform();
    transform.translation() = translation + (m_targetTranslation - translation);
    dynamic_cast<dart::dynamics::FreeJoint *>(m_objSkeleton->getJoint(0))
        ->setTransform(transform);
    ros::Duration(0.5).sleep();
    translation = m_objSkeleton->getBodyNode(0)->getTransform().translation();
  }
}