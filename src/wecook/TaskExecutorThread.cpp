//
// Created by hejia on 8/27/19.
//

#include "wecook/TaskExecutorThread.h"

using namespace wecook;

void TaskExecutorThread::run() {
  ROS_INFO_STREAM(m_agent->getPid() + " starting execute the task in this task executor thread...");
  auto pid = m_agent->getPid();
  auto curr = m_currentTask->getHeadNode(pid);
  while (curr) {
    m_currentActionNode = curr;
    // wait until all agents involved in this action are ready
    //    m_syncCallback(m_currentActionNode);
    auto fatherNodes = curr->getFathers();
    for (auto &father : fatherNodes) {
      while (!father->m_ifExecuted) {
        ros::Duration(0.5).sleep();
      }
    }
    // execute primitive task graph
    auto ptg = m_currentActionNode->m_primitiveTaskGraph;
    // get the first primitive action node to execute
    auto currPAN = ptg.getHeadNode(pid);
    while (currPAN) {
      // need to wait all of currPAN's father nodes have been executed
      for (auto &father : currPAN->getFathers()) {
        while (!father->ifExecuted()) {
          ros::Duration(0.5).sleep();
        }
      }
      // if motionMutex is not nullptr
      // we need to first lock the mutex
      if (m_motionMutex) {
        boost::unique_lock<boost::mutex> lock(*m_motionMutex);
        m_pae->execute(currPAN);
        lock.unlock();
      } else {
        m_pae->execute(currPAN);
      }

      // find next primitive action node to execute
      auto last = currPAN;
      currPAN = nullptr;
      for (auto &child : last->getChildren()) {
        if (child->getPid() == pid) {
          currPAN = child;
        }
      }
    }
    curr->m_ifExecuted = true;
    // find next action
    ActionNode *next = nullptr;
    for (auto &child : curr->getChildren()) {
      auto childPids = child->getAction().get_pids();
      if (std::find(childPids.begin(), childPids.end(), pid) != childPids.end()) {
        next = child;
        break;
      }
    }
    curr = next;
  }

  m_isEnd = true;
}