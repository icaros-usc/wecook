//
// Created by hejia on 8/6/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/LowLevelMotionNodes/LinearDeltaMotionNode.h"

using namespace wecook;

void LinearDeltaMotionNode::plan(const std::shared_ptr<ada::Ada> &adaPlan,
                                 const std::shared_ptr<ada::Ada> &adaExec,
                                 Result *result) {
    Eigen::VectorXd delta_q(6);

    for (int i = 0; i < m_repeat_time; i++) {
        auto jac = m_skeleton->getLinearJacobian(m_bn, m_incoordinatesOf);
        delta_q << aikido::common::pseudoinverse(jac) * m_delta_x;
        Eigen::VectorXd currPos = m_skeleton->getPositions();
        ros::Duration(0.05).sleep();
        Eigen::VectorXd new_pos = currPos + delta_q;

        std::cout << "curr pos: " << currPos << std::endl;
        std::cout << "new pos: " << new_pos << std::endl;

        // now check if it will be in collision
        if (m_collisionFree) {
            aikido::constraint::DefaultTestableOutcome collisionCheckOutcome;
            auto armState = m_stateSpace->createState();
            m_stateSpace->convertPositionsToState(new_pos, armState);
            auto fullCollisionFreeConstraint = adaPlan->getFullCollisionConstraint(m_stateSpace, m_skeleton,
                                                                                   m_collisionFree);
            auto collisionResult = fullCollisionFreeConstraint->isSatisfied(armState, &collisionCheckOutcome);

            // set the adaPlan back to the old position, since it's just for collision checking
            m_skeleton->setPositions(currPos);

            if (!collisionResult) {
                ROS_INFO("[RelativeIKMotionNode::plan] Robot arm will be in collision!");
                break;
            }
        }

        // moves adaPlan, adaPlan should always be simulated
        // we should first use adaPlan to plan a path for adaExec
        if (adaExec->ifSim()) {
            m_skeleton->setPositions(new_pos);
            adaExec->getArm()->getMetaSkeleton()->setPositions(new_pos);
        } else {
            auto traj = adaPlan->planToConfiguration(m_stateSpace, m_skeleton, new_pos, nullptr, 10);
            aikido::trajectory::TrajectoryPtr retime_traj = adaPlan->retimePath(m_skeleton, traj.get());
            auto future = adaExec->executeTrajectory(retime_traj);
            future.wait();
            m_skeleton->setPositions(new_pos);
        }
    }

    if (result) {
        result->setStatus(Result::StatusType::SUCCEEDED);
    }
}