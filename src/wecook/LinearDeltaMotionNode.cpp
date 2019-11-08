//
// Created by hejia on 8/6/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/LinearDeltaMotionNode.h"

using namespace wecook;

void LinearDeltaMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg) {
  Eigen::VectorXd delta_q(6);

  for (int i = 0; i < m_repeat_time; i++) {
    auto jac = m_skeleton->getLinearJacobian(m_bn, m_incoordinatesOf);
    delta_q << aikido::common::pseudoinverse(jac) * m_delta_x;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;

    // now check if it will be in collision
    if (m_collisionFree) {
      aikido::constraint::DefaultTestableOutcome collisionCheckOutcome;
      auto armState = m_stateSpace->createState();
      m_stateSpace->convertPositionsToState(new_pos, armState);
      auto fullCollisionFreeConstraint = ada->getFullCollisionConstraint(m_stateSpace, m_skeleton, m_collisionFree);
      auto collisionResult = fullCollisionFreeConstraint->isSatisfied(armState, &collisionCheckOutcome);
      if (!collisionResult) {
        // first set the old positions
        m_skeleton->setPositions(currPos);
        ROS_INFO("[RelativeIKMotionNode::plan] Robot arm will be in collision!");
        break;
      }
    }

    m_skeleton->setPositions(new_pos);
    if (adaImg) {
      adaImg->getArm()->getMetaSkeleton()->setPositions(new_pos);
    }
  }
}