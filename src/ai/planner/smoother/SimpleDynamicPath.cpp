//
// Created by hejia on 9/17/19.
//
#include <iostream>

#include "SimpleDynamicPath.h"

using namespace wecook::planner::detail;

SimpleDynamicPath::SimpleDynamicPath() {

}

void SimpleDynamicPath::Init(const Eigen::VectorXd &maxVelocity,
                             const Eigen::VectorXd &maxAcceleration) {
  m_maxVelocity = maxVelocity;
  m_maxAcceleration = maxAcceleration;
}

void SimpleDynamicPath::SetMilestones(const std::vector<Eigen::VectorXd> &milestones,
                                      const std::vector<Eigen::VectorXd> &velocities) {
  m_milestones = milestones;
  m_velocities = velocities;
}

void SimpleDynamicPath::SetMilestones(const std::vector<Eigen::VectorXd> &milestones) {
  m_milestones = milestones;
}

void SimpleDynamicPath::Shortcut(int i1, int i2) {
  std::cout << "Shortcutting!!! " << i1 + 1 << " " << i2 + 1 << " " << m_milestones.size() << std::endl;
  m_milestones.erase(m_milestones.begin() + i1 + 1, m_milestones.begin() + i2);

  if (!m_velocities.empty()) {
    m_velocities.erase(m_velocities.begin() + i1 + 1, m_velocities.begin() + i2);
  }
  std::cout << "Finished!!! " << m_milestones.size() << std::endl;
}


