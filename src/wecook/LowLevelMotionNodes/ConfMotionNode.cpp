//
// Created by hejia on 8/6/19.
//

#include "wecook/LowLevelMotionNodes/ConfMotionNode.h"

using namespace wecook;

void
ConfMotionNode::plan(const std::shared_ptr<ada::Ada> &adaPlan, const std::shared_ptr<ada::Ada> &adaExec,
                     Result *result) {
    auto goalState = m_stateSpace->createState();
    m_stateSpace->convertPositionsToState(m_adaPlanGoalConf, goalState);

    adaPlan->getHand()->executePreshape(m_adaPlanGoalConf);
    adaExec->getHand()->executePreshape(m_adaExecGoalConf);

    ros::Duration(3).sleep();

    if (result) {
        result->setStatus(Result::StatusType::SUCCEEDED);
    }
}