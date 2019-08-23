//
// Created by hejia on 8/12/19.
//

#include "wecook/ConnMotionNode.h"

using namespace wecook;

void ConnMotionNode::plan(const std::shared_ptr<ada::Ada> &ada) {
  if (m_condition) {
    ROS_INFO("ConnMotionNode: waiting for condition...");
    while (!m_condition->isSatisfied()) {
      // sleep a little bit
      ros::Duration(1.).sleep();
    }
    ROS_INFO("ConnMotionNode: Condition is verified!");
  }

  if (m_conn) {
    m_containingMap->connect(m_bodyToConn, m_bodyThatConns, m_containerName, m_objectName);
  } else {
    m_containingMap->unconnect(m_bodyToConn, m_bodyThatConns, m_containerName, m_objectName);
  }
}