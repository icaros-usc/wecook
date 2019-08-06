//
// Created by hejia on 8/6/19.
//

#include "wecook/ConnMotionPlanner.h"

using namespace wecook;

void ConnMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  if (m_grab) {
    ada->getHand()->grab(m_bodyToGrab);
  } else {
    ada->getHand()->ungrab();
  }
}