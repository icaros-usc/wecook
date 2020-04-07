//
// Created by hejia on 8/6/19.
//

#include "wecook/LowLevelMotionNodes/ConfMotionNode.h"

using namespace wecook;

void
ConfMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result) {
    auto goalState = m_stateSpace->createState();
    m_stateSpace->convertPositionsToState(m_goalConf, goalState);

    ada->getHand()->executePreshape(m_goalConf);

//  auto trajectory = ada->planToConfiguration(m_stateSpace,
//                                             m_skeleton,
//                                             goalState,
//                                             nullptr,
//                                             1.0);
//  auto future = ada->executeTrajectory(trajectory);
//  future.wait();

//  if (adaImg) {
//    auto future2 = adaImg->executeTrajectory(trajectory);
//    future2.wait();
//  }

    ros::Duration(3).sleep();

    if (result) {
        result->setStatus(Result::StatusType::SUCCEEDED);
    }
}