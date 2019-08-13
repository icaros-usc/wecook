//
// Created by hejia on 8/12/19.
//

#include "wecook/ConnMotionPlanner.h"

using namespace wecook;

void ConnMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  if (m_conn) {
    m_containingMap->connect(m_bodyToConn, m_bodyThatConns, m_containerName, m_objectName);
  } else {
    m_containingMap->unconnect(m_bodyToConn, m_bodyThatConns, m_containerName, m_objectName);
  }
}