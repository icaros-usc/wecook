//
// Created by hejia on 8/26/19.
//

#include "wecook/Agents/Human.h"

using namespace wecook;

void Human::init(std::shared_ptr<aikido::planner::World> &env) {
  // sim human initialize function
  createHuman(env);
  if (m_ifSim) createHumanImg(env);
  moveToHome();
}

Eigen::Vector3d Human::getPosition() {
  return m_transform.translation();
}

void Human::end() {
  m_human.reset();
  if (m_humanImg) m_humanImg.reset();
}

void Human::moveToHome() {
  m_human->getRightArm()->getMetaSkeleton()->setPositions(m_homePositions);
  if (m_humanImg) m_humanImg->getRightArm()->getMetaSkeleton()->setPositions(m_homePositions);
}