//
// Created by hejia on 7/30/19.
//

#include <iostream>
#include <aikido/common/PseudoInverse.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>

#include "wecook/Agent.h"

using namespace wecook;

void Agent::stop() {
  m_isEnd = true;
  m_thread.join();
}

void Agent::run() {
  while (!m_isEnd) {
    if (m_currentTask) {
      m_isFree = false;
      auto start = m_currentTask->getHeadNode(m_pid);
      while (start) {
        m_currentActionNode = start;
        // wait until all agents involved in this action are ready
        m_syncCallback(m_currentActionNode);
        // plan motion
        for (auto &motion : start->m_motionSeqMap[m_pid]) {
          if (!motion) ROS_INFO_STREAM("motion is empty" << start->getAction().get_verb() << motion);
          motion->plan(m_ada);
        }
        // find next action
        ActionNode *next = nullptr;
        for (auto &child : start->getChildren()) {
          auto childPids = child->getAction().get_pids();
          if (std::find(childPids.begin(), childPids.end(), m_pid) != childPids.end()) {
            next = child;
            break;
          }
        }
        start = next;
      }
      // reset m_currentTask
      m_currentTask = nullptr;
    }
    m_isFree = true;
  }
}

void Agent::moveToHome() {
  m_ada->getArm()->getMetaSkeleton()->setPositions(m_homePositions);
}
