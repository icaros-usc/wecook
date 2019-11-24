//
// Created by hejia on 8/6/19.
//

#include "wecook/LowLevelMotionNodes/GrabMotionNode.h"

using namespace wecook;

void GrabMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result) {
  if (m_grab) {
    ada->getHand()->grab(m_bodyToGrab);
  } else {
    ada->getHand()->ungrab();
  }

  if (result) {
    result->setStatus(Result::StatusType::SUCCEEDED);
  }
}
