//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveActionNode.h"

using namespace wecook;

void PrimitiveActionNode::removeChild(std::shared_ptr<PrimitiveActionNode> &child) {
  std::cout << child << std::endl;
  auto itr = m_children.begin();
  while (itr != m_children.end()) {
    std::cout << *itr << std::endl;
    if (*itr == child) {
      std::cout << "Removed tail!　" << m_grabbingObj << std::endl;
      itr = m_children.erase(itr);
      break;
    } else {
      ++itr;
    }
  }
}

void PrimitiveActionNode::removeFather(std::shared_ptr<PrimitiveActionNode> &father) {
  auto itr = m_fathers.begin();
  while (itr != m_fathers.end()) {
    if (*itr == father) {
      itr = m_fathers.erase(itr);
      break;
    } else {
      ++itr;
    }
  }
}

