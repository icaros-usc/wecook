//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveActionNode.h"

using namespace wecook;

void PrimitiveActionNode::removeChild(std::shared_ptr<PrimitiveActionNode> &child) {
  for (auto itr = m_children.begin(); itr != m_children.end(); ++itr) {
    if (*itr == child) {
      m_children.erase(itr);
    }
  }
}

void PrimitiveActionNode::removeFather(std::shared_ptr<PrimitiveActionNode> &father) {
  for (auto itr = m_fathers.begin(); itr != m_fathers.end(); ++itr) {
    if (*itr == father) {
      m_fathers.erase(itr);
    }
  }
}


