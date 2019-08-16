//
// Created by hejia on 8/15/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/IKMotionPlanner.h"
#include "wecook/utils.h"

using namespace wecook;

void IKMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  Eigen::VectorXd delta_q(6);

  auto currentPose = m_bn->getTransform(m_incoordinatesOf);
  std::cout << currentPose.linear() << std::endl;
  std::cout << currentPose.translation() << std::endl;
  auto currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
//  currentSpatialVector(0, 0) = -0.1699226;
//  currentSpatialVector(1, 0) = -0.0829827;
//  currentSpatialVector(2, 0) = -0.139742;
  std::cout << "Current pose: " << currentSpatialVector << std::endl;
  auto targetSpatialVector = TransformMatrix2SpatialVector(m_targetPose);
  std::cout << "Target pose: " << targetSpatialVector << std::endl;
  auto delta_x = (targetSpatialVector - currentSpatialVector) / m_repeat_time;

  while ((targetSpatialVector - currentSpatialVector).norm() > 0.45) {
    std::cout << (targetSpatialVector - currentSpatialVector).norm() << std::endl;
    auto jac = m_skeleton->getJacobian(m_bn, m_incoordinatesOf);
//    std::cout << jac << std::endl;
    delta_q << aikido::common::pseudoinverse(jac) * (targetSpatialVector - currentSpatialVector) * 0.0005;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);

    currentPose = m_bn->getTransform(m_incoordinatesOf);
    currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  }

  std::cout << "Final translation: " << m_bn->getTransform().translation() << " Final ori: " << m_bn->getTransform().linear() << std::endl;
}