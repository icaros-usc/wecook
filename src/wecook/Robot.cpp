//
// Created by hejia on 8/26/19.
//

#include "wecook/Robot.h"

using namespace wecook;

void Robot::moveToHome() {
  m_ada->getArm()->getMetaSkeleton()->setPositions(m_homePositions);
  if (m_adaImg) m_adaImg->getArm()->getMetaSkeleton()->setPositions(m_homePositions);
}

void Robot::init(std::shared_ptr<aikido::planner::World> &env) {
  // sim robot initialize function
  createAda(env);
  if (m_ifSim) createAdaImg(env);
  moveToHome();
}

Eigen::Vector3d Robot::getPosition() {
  return m_transform.translation();
}

void Robot::end() {
  // release robot
  m_ada.reset();
  if (m_adaImg) m_adaImg.reset();
}

