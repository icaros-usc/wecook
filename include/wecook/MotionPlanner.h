//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_MOTIONPLANNER_H
#define WECOOK_MOTIONPLANNER_H

#include <libada/Ada.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>

namespace wecook {

class MotionPlanner {
 public:
  MotionPlanner(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                const dart::dynamics::MetaSkeletonPtr &skeleton) : m_stateSpace(stateSpace), m_skeleton(skeleton) {

  }

  virtual void plan(const std::shared_ptr<ada::Ada> &ada) = 0;

  aikido::statespace::dart::MetaSkeletonStateSpacePtr m_stateSpace = nullptr;
  dart::dynamics::MetaSkeletonPtr m_skeleton = nullptr;



};
}

#endif //WECOOK_MOTIONPLANNER_H
