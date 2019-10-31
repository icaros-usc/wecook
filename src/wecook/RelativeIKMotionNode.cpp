//
// Created by hejia on 8/25/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/RelativeIKMotionNode.h"
#include "wecook/utils.h"

using namespace wecook;

void RelativeIKMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg) {
  Eigen::VectorXd delta_q(6);

  auto currentPose = m_bn->getTransform(m_incoordinatesOf);
  auto currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  auto targetPose = currentPose * m_relT;
  auto targetSpatialVector = TransformMatrix2SpatialVector(targetPose);

  while ((targetSpatialVector - currentSpatialVector).norm() > 0.35) {
    auto jac = m_skeleton->getJacobian(m_bn, m_incoordinatesOf);
    delta_q << aikido::common::pseudoinverse(jac) * (targetSpatialVector - currentSpatialVector) * 0.03;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);

    if (adaImg) {
      adaImg->getArm()->getMetaSkeleton()->setPositions(new_pos);
    }

    currentPose = m_bn->getTransform(m_incoordinatesOf);
    currentSpatialVector = TransformMatrix2SpatialVector(currentPose);
  }
}