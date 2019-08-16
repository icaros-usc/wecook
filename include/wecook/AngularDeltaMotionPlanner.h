//
// Created by hejia on 8/15/19.
//

#ifndef WECOOK_ANGULARDELTAMOTIONPLANNER_H
#define WECOOK_ANGULARDELTAMOTIONPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class AngularDeltaMotionPlanner : public MotionPlanner {
 public:
  AngularDeltaMotionPlanner(dart::dynamics::BodyNode *bn,
                            const Eigen::VectorXd &delta_x,
                            dart::dynamics::Frame *incoordinatesOf,
                            double repeat_time,
                            const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                            const dart::dynamics::MetaSkeletonPtr &skeleton,
                            const std::shared_ptr<PreCondition> &condition = nullptr) : MotionPlanner(stateSpace,
                                                                                                      skeleton,
                                                                                                      condition),
                                                                                        m_bn(bn),
                                                                                        m_delta_x(delta_x),
                                                                                        m_incoordinatesOf(incoordinatesOf),
                                                                                        m_repeat_time(repeat_time) {

  }

//  ~AngularDeltaMotionPlanner() {
//    delete m_incoordinatesOf;
//  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  Eigen::VectorXd m_delta_x;
  double m_repeat_time;
  dart::dynamics::BodyNode *m_bn = nullptr;
  dart::dynamics::Frame *m_incoordinatesOf = nullptr;
};

}

#endif //WECOOK_ANGULARDELTAMOTIONPLANNER_H
