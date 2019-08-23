//
// Created by hejia on 8/15/19.
//

#include <aikido/common/PseudoInverse.hpp>

#include "wecook/DeltaMotionNode.h"

using namespace wecook;

void DeltaMotionNode::plan(const std::shared_ptr<ada::Ada> &ada) {
  Eigen::VectorXd delta_q(6);

  for (int i = 0; i < m_repeat_time; i++) {
    auto jac = m_skeleton->getJacobian(m_bn, m_incoordinatesOf);
    std::cout << jac << std::endl;
    delta_q << aikido::common::pseudoinverse(jac) * m_delta_x;
    Eigen::VectorXd currPos = m_skeleton->getPositions();
    ros::Duration(0.1).sleep();
    Eigen::VectorXd new_pos = currPos + delta_q;
    m_skeleton->setPositions(new_pos);
  }
}