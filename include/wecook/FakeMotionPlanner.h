//
// Created by hejia on 8/16/19.
//

#ifndef WECOOK_FAKEMOTIONPLANNER_H
#define WECOOK_FAKEMOTIONPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class FakeMotionPlanner : public MotionPlanner {
 public:
  FakeMotionPlanner(dart::dynamics::SkeletonPtr objSkeleton,
                    Eigen::Vector3d targetTranslation,
                    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                    const dart::dynamics::MetaSkeletonPtr &skeleton,
                    const std::shared_ptr<PreCondition> &condition = nullptr) : MotionPlanner(stateSpace,
                                                                                                 skeleton,
                                                                                                 condition),
                                                                                   m_objSkeleton(objSkeleton),
                                                                                   m_targetTranslation(targetTranslation) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  Eigen::Vector3d m_targetTranslation;
  dart::dynamics::SkeletonPtr m_objSkeleton = nullptr;
};

}

#endif //WECOOK_FAKEMOTIONPLANNER_H
