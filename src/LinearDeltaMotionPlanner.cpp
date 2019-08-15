//
// Created by hejia on 8/6/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/LinearDeltaMotionPlanner.h"

using namespace wecook;

void LinearDeltaMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  Eigen::VectorXd delta_q(6);

  for (int i = 0; i < m_repeat_time; i++) {
    auto jac = m_skeleton->getLinearJacobian(m_bn);
    delta_q << aikido::common::pseudoinverse(jac) * m_delta_x;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);
  }
}