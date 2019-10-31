//
// Created by hejia on 8/6/19.
//

#include "wecook/ConfMotionNode.h"

using namespace wecook;

void ConfMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg) {
  if (m_condition) {
    ROS_INFO("ConfMotionNode: waiting for condition...");
    while (!m_condition->isSatisfied()) {
      // sleep a little bit
      ros::Duration(1.).sleep();
    }
    ROS_INFO("ConfMotionNode: Condition is verified!");
  }

  auto goalState = m_stateSpace->createState();
  m_stateSpace->convertPositionsToState(m_goalConf, goalState);

  auto trajectory = ada->planToConfiguration(m_stateSpace,
                                             m_skeleton,
                                             goalState,
                                             nullptr,
                                             1.0);
  auto future = ada->executeTrajectory(trajectory);
  future.wait();

  if (adaImg) {
    auto future2 = adaImg->executeTrajectory(trajectory);
    future2.wait();
  }
}