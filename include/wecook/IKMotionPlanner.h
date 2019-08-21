//
// Created by hejia on 8/15/19.
//

#ifndef WECOOK_IKMOTIONPLANNER_H
#define WECOOK_IKMOTIONPLANNER_H

#include <dart/dynamics/Frame.hpp>

#include "MotionPlanner.h"

namespace wecook {

class IKMotionPlanner : public MotionPlanner {
 public:
  IKMotionPlanner(dart::dynamics::BodyNode *bn,
                  const Eigen::Isometry3d &targetPose,
                  dart::dynamics::Frame *incoordinatesOf,
                  const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                  const dart::dynamics::MetaSkeletonPtr &skeleton,
                  const std::shared_ptr<PreCondition> &condition = nullptr) : MotionPlanner(stateSpace,
                                                                                            skeleton,
                                                                                            condition),
                                                                              m_bn(bn),
                                                                              m_targetPose(targetPose),
                                                                              m_incoordinatesOf(incoordinatesOf) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  Eigen::Isometry3d m_targetPose;
  dart::dynamics::BodyNode *m_bn = nullptr;
  dart::dynamics::Frame *m_incoordinatesOf = nullptr;
};

}

#endif //WECOOK_IKMOTIONPLANNER_H
