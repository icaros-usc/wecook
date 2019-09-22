//
// Created by hejia on 8/6/19.
//

#include "wecook/GrabMotionNode.h"

using namespace wecook;

void GrabMotionNode::plan(const std::shared_ptr<ada::Ada> &ada) {
  if (m_condition) {
    ROS_INFO("GrabMotionNode: waiting for condition...");
    while (!m_condition->isSatisfied()) {
      // sleep a little bit
      ros::Duration(1.).sleep();
    }
    ROS_INFO("GrabMotionNode: Condition is verified!");
  }

  if (m_grab) {
    ada->getHand()->grab(m_bodyToGrab);
  } else {
    ada->getHand()->ungrab();
  }
}
