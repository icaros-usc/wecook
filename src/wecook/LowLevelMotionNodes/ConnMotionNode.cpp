//
// Created by hejia on 8/12/19.
//

#include "wecook/LowLevelMotionNodes/ConnMotionNode.h"

using namespace wecook;

void ConnMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result) {
  if (m_conn) {
    m_containingMap->connect(m_bodyToConn, m_bodyThatConns, m_containerName, m_objectName);
  } else {
    m_containingMap->unconnect(m_bodyToConn, m_bodyThatConns, m_containerName, m_objectName);
  }

  if (result) {
    result->setStatus(Result::StatusType::SUCCEEDED);
  }
}