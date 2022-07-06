//
// Created by hejia on 8/26/19.
//

#include "wecook/Agents/Robot.h"

using namespace wecook;

void Robot::moveToHome() {
    m_adaPlan->getArm()->getMetaSkeleton()->setPositions(m_homePositions);
    if (m_adaExec) m_adaExec->getArm()->getMetaSkeleton()->setPositions(m_homePositions);
}

void Robot::init(std::shared_ptr<aikido::planner::World> &env) {
    createAdaPlan(env);
    createAdaExec(env);
    moveToHome();
}

Eigen::Vector3d Robot::getPosition() {
    return m_transform.translation();
}

void Robot::end() {
    // release robot
    m_adaPlan.reset();
    if (m_adaExec) m_adaExec.reset();
}

