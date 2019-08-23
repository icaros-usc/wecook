//
// Created by hejia on 8/15/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/IKMotionNode.h"
#include "wecook/utils.h"

using namespace wecook;

void IKMotionNode::plan(const std::shared_ptr<ada::Ada> &ada) {
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
  std::cout << currentPose.linear() << std::endl;
  std::cout << currentPose.translation() << std::endl;
  auto currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  std::cout << "Current pose: " << currentSpatialVector << std::endl;
  auto targetSpatialVector = TransformMatrix2SpatialVector(m_targetPose);
  std::cout << "Target pose: " << targetSpatialVector << std::endl;

  while ((targetSpatialVector - currentSpatialVector).norm() > 0.45) {
    std::cout << (targetSpatialVector - currentSpatialVector).norm() << std::endl;
    auto jac = m_skeleton->getJacobian(m_bn, m_incoordinatesOf);
    delta_q << aikido::common::pseudoinverse(jac) * (targetSpatialVector - currentSpatialVector) * 0.005;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);

    currentPose = m_bn->getTransform(m_incoordinatesOf);
    currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  }

  std::cout << "Final translation: " << m_bn->getTransform().translation() << " Final ori: " << m_bn->getTransform().linear() << std::endl;
}