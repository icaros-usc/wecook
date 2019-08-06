//
// Created by hejia on 8/6/19.
//

#include "wecook/ConfMotionPlanner.h"

using namespace wecook;

void ConfMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  auto goalState = m_stateSpace->createState();
  m_stateSpace->convertPositionsToState(m_goalConf, goalState);

  auto trajectory = ada->planToConfiguration(m_stateSpace,
                                               m_skeleton,
                                               goalState,
                                               nullptr,
                                               1.0);
  auto future = ada->executeTrajectory(trajectory);
  future.wait();
}