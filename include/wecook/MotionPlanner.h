//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_MOTIONPLANNER_H
#define WECOOK_MOTIONPLANNER_H

#include <libada/Ada.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include "PreCondition.h"

namespace wecook {

class MotionPlanner {
 public:
  MotionPlanner(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                const dart::dynamics::MetaSkeletonPtr &skeleton,
                const std::shared_ptr<PreCondition> &condition)
      : m_stateSpace(stateSpace), m_skeleton(skeleton), m_condition(condition) {

  }

  virtual void plan(const std::shared_ptr<ada::Ada> &ada) = 0;

  aikido::statespace::dart::MetaSkeletonStateSpacePtr m_stateSpace = nullptr;
  dart::dynamics::MetaSkeletonPtr m_skeleton = nullptr;
  std::shared_ptr<PreCondition> m_condition = nullptr;

};
}

#endif //WECOOK_MOTIONPLANNER_H
