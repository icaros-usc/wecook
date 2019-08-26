//
// Created by hejia on 7/30/19.
//

#include <iostream>
#include <aikido/common/PseudoInverse.hpp>

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
      auto curr = m_currentTask->getHeadNode(m_pid);
      while (curr) {
        m_currentActionNode = curr;
        // wait until all agents involved in this action are ready
        m_syncCallback(m_currentActionNode);
        // execute primitive task graph
        auto ptg = m_currentActionNode->m_primitiveTaskGraph;
        // get the first primitive action node to execute
        auto currPAN = ptg.getHeadNode(m_pid);
        while (currPAN) {
          // need to wait all of currPAN's father nodes have been executed
          for (auto &father : currPAN->getFathers()) {
            while (!father->ifExecuted()) {
              ros::Duration(0.5).sleep();
            }
          }

          m_pap.execute(currPAN);

          // find next primitive action node to execute
          currPAN = nullptr;
          for (auto &child : currPAN->getChildren()) {
            if (child->getPid() == m_pid) {
              currPAN = child;
            }
          }
        }
//        // plan motion
//        for (auto &motion : curr->m_motionSeqMap[m_pid]) {
//          if (!motion) ROS_INFO_STREAM("motion is empty" << curr->getAction().get_verb() << motion);
//          motion->plan(m_ada);
//        }
        // find next action
        ActionNode *next = nullptr;
        for (auto &child : curr->getChildren()) {
          auto childPids = child->getAction().get_pids();
          if (std::find(childPids.begin(), childPids.end(), m_pid) != childPids.end()) {
            next = child;
            break;
          }
        }
        curr = next;
      }
      // reset m_currentTask
      m_currentTask.reset();
    }
    m_isFree = true;
  }
}

void Agent::moveToHome() {
  m_ada->getArm()->getMetaSkeleton()->setPositions(m_homePositions);
}
