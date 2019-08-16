//
// Created by hejia on 8/15/19.
//

#ifndef WECOOK_GRAVITYMOTIONPLANNER_H
#define WECOOK_GRAVITYMOTIONPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class GravityMotionPlanner : public MotionPlanner {
 public:
  GravityMotionPlanner(dart::dynamics::SkeletonPtr objSkeleton,
                       double repeat_time,
                       const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                       const dart::dynamics::MetaSkeletonPtr &skeleton,
                       const std::shared_ptr<PreCondition> &condition = nullptr) : MotionPlanner(stateSpace,
                                                                                                 skeleton,
                                                                                                 condition),
                                                                                   m_objSkeleton(objSkeleton),
                                                                                   m_repeat_time(repeat_time) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  double m_repeat_time;
  dart::dynamics::SkeletonPtr m_objSkeleton = nullptr;
};

}

#endif //WECOOK_GRAVITYMOTIONPLANNER_H
