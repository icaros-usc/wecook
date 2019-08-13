//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_GRABMOTIONPLANNER_H
#define WECOOK_GRABMOTIONPLANNER_H
#include "MotionPlanner.h"

namespace wecook {

class GrabMotionPlanner : public MotionPlanner {
 public:
  GrabMotionPlanner(const dart::dynamics::SkeletonPtr &bodyToGrab,
                    bool grab,
                    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                    const dart::dynamics::MetaSkeletonPtr &skeleton) : MotionPlanner(stateSpace, skeleton),
                                                                       m_bodyToGrab(bodyToGrab),
                                                                       m_grab(grab) {
  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  dart::dynamics::SkeletonPtr m_bodyToGrab;
  bool m_grab;
};

}

#endif //WECOOK_GRABMOTIONPLANNER_H
