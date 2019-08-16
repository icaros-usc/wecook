//
// Created by hejia on 8/15/19.
//

#include "wecook/GravityMotionPlanner.h"

using namespace wecook;

void GravityMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  Eigen::VectorXd delta_q(6);

  for (int i = 0; i < m_repeat_time; i++) {
    auto transform = m_objSkeleton->getBodyNode(0)->getTransform();
    transform.translation()[2] += -0.01;
    dynamic_cast<dart::dynamics::FreeJoint *>(m_objSkeleton->getJoint(0))
        ->setTransform(transform);
    ros::Duration(0.5).sleep();
  }
}