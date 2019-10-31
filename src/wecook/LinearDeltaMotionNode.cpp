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
    m_skeleton->setPositions(new_pos);
    if (adaImg) {
      adaImg->getArm()->getMetaSkeleton()->setPositions(new_pos);
    }
  }
}