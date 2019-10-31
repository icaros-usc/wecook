//
// Created by hejia on 8/15/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/AngularDeltaMotionNode.h"

using namespace wecook;

void AngularDeltaMotionNode::plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg) {
  Eigen::VectorXd delta_q(6);

  for (int i = 0; i < m_repeat_time; i++) {
    auto jac = m_skeleton->getAngularJacobian(m_bn, m_incoordinatesOf);
    auto pseudoinverse = aikido::common::pseudoinverse(jac);
    delta_q << pseudoinverse * m_delta_x;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);
  }
}