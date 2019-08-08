//
// Created by hejia on 8/6/19.
//

#include "wecook/ConnMotionPlanner.h"

using namespace wecook;

//void ConnMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
//  using dart::dynamics::Joint;
//  using dart::dynamics::FreeJoint;
//  using dart::dynamics::WeldJoint;
//
//  if (m_grab) {
//    // conn
//    if (m_bodyToGrab->getNumBodyNodes() != 1) {
//      std::stringstream ss;
//      // TODO: use proper logging
//      ss << "[Hand::grab] Only Skeletons with one BodyNode may be "
//         << "grabbed. Skeleton '" << m_bodyToGrab->getName() << "' has "
//         << m_bodyToGrab->getNumBodyNodes() << " BodyNodes" << std::endl;
//
//      throw std::runtime_error(ss.str());
//    }
//
//    // TODO: this should be Skeleton::getRootJoint() once DART 6.2 is released
//    Joint* joint = m_bodyToGrab->getJoint(0);
//    FreeJoint* freeJoint = dynamic_cast<FreeJoint*>(joint);
//    if (freeJoint == nullptr)
//    {
//      std::stringstream ss;
//      // TODO: use proper logging
//      ss << "[Hand::grab] Only Skeletons with a root FreeJoint may "
//         << "be grabbed. Skeleton '" << m_bodyToGrab->getName() << "' has a "
//         << "root " << joint->getType() << std::endl;
//
//      throw std::runtime_error(ss.str());
//    }
//  } else {
//    // unconn
//  }
//}

void ConnMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  if (m_grab) {
    ada->getHand()->grab(m_bodyToGrab);
  } else {
    ada->getHand()->ungrab();
  }
}
