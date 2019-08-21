//
// Created by hejia on 8/20/19.
//

#ifndef WECOOK_TSRIKPLANNER_H
#define WECOOK_TSRIKPLANNER_H

#include "MotionPlanner.h"

namespace wecook {

class TSRIKPlanner : public MotionPlanner {
 public:
  TSRIKPlanner(const aikido::constraint::dart::TSRPtr &goalTSR,
               dart::dynamics::BodyNode *bn,
               int interpolating_num,
               const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
               const dart::dynamics::MetaSkeletonPtr &skeleton,
               const std::shared_ptr<PreCondition> &condition = nullptr,
               bool debug = false) : MotionPlanner(stateSpace, skeleton, condition),
                                     m_goalTSR(goalTSR),
                                     m_bn(bn),
                                     m_debug(debug),
                                     m_interpolating_num(interpolating_num) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

  int m_interpolating_num;
  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
  bool m_debug;
};

}

#endif //WECOOK_TSRIKPLANNER_H
