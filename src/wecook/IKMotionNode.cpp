//
// Created by hejia on 8/15/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/IKMotionNode.h"
#include "wecook/utils.h"

using namespace wecook;

void IKMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg) {
  if (m_condition) {
    ROS_INFO("[IKMotionNode::plan]: Waiting for condition to be verified!");
    while (!m_condition->isSatisfied()) {
      // sleep a little bit
      ros::Duration(1.).sleep();
    }
    ROS_INFO("[IKMotionNode::plan]: Condition is verified!");
  }

  Eigen::VectorXd delta_q(6);

  auto currentPose = m_bn->getTransform(m_incoordinatesOf);
  auto currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  auto targetSpatialVector = TransformMatrix2SpatialVector(m_targetPose);

  while ((targetSpatialVector - currentSpatialVector).norm() > 0.45) {
    auto jac = m_skeleton->getJacobian(m_bn, m_incoordinatesOf);
    delta_q << aikido::common::pseudoinverse(jac) * (targetSpatialVector - currentSpatialVector) * 0.005;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);

    currentPose = m_bn->getTransform(m_incoordinatesOf);
    currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  }
}