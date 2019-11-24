//
// Created by hejia on 8/25/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/LowLevelMotionNodes/RelativeIKMotionNode.h"
#include "wecook/utils.h"

using namespace wecook;

void RelativeIKMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result) {
  Eigen::VectorXd delta_q(6);

  auto currentPose = m_bn->getTransform(m_incoordinatesOf);
  auto currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  auto targetPose = currentPose * m_relT;
  auto targetSpatialVector = TransformMatrix2SpatialVector(targetPose);

  while ((targetSpatialVector - currentSpatialVector).norm() > 0.25) {
    // assuming current skeleton is not in collision
    // everytime, before we update the skeleton, we
    // check if the new skeleton is in collision.
    auto jac = m_skeleton->getJacobian(m_bn, m_incoordinatesOf);
    delta_q << aikido::common::pseudoinverse(jac) * (targetSpatialVector - currentSpatialVector) * 0.03;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.05).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;

    // now check if it's in collision
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

    currentPose = m_bn->getTransform(m_incoordinatesOf);
    currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  }

  if (result) {
    result->setStatus(Result::StatusType::SUCCEEDED);
  }
}