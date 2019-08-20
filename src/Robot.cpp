//
// Created by hejia on 7/30/19.
//

#include <iostream>
#include <aikido/common/PseudoInverse.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>

#include "wecook/Robot.h"

using namespace wecook;

int Robot::execute(Action &action) {
  m_action.emplace_back(action);
}

void Robot::stop() {
  m_isEnd = true;
  m_thread.join();
}

void Robot::run() {
  while (!m_isEnd) {
    while (!m_subMotions.empty()) {
      m_isFree = false;
      for (auto &motion : m_subMotions) {
        motion->plan(m_ada);
      }
      m_subMotions.clear();
    }
    m_isFree = true;
  }
}

void Robot::moveToHome() {
  m_ada->getArm()->getMetaSkeleton()->setPositions(m_homePositions);
}
