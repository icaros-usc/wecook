//
// Created by hejia on 8/15/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/AngularDeltaMotionPlanner.h"

using namespace wecook;

void AngularDeltaMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  Eigen::VectorXd delta_q(6);

  for (int i = 0; i < m_repeat_time; i++) {
    auto jac = m_skeleton->getAngularJacobian(m_bn);
    std::cout << jac << std::endl;
    auto pseudoinverse = aikido::common::pseudoinverse(jac);
    std::cout << pseudoinverse << std::endl;
    delta_q << pseudoinverse * m_delta_x;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);
  }
}