//
// Created by hejia on 8/27/19.
//

#include "wecook/TaskExecutorThread.h"

using namespace wecook;

void TaskExecutorThread::run() {
  auto pid = m_agent->getPid();
  auto curr = m_currentTask->getHeadNode(pid);
  while (curr) {
    std::cout << pid << " " << curr->getAction().get_verb() << std::endl;
    m_currentActionNode = curr;
    // wait until all agents involved in this action are ready
    m_syncCallback(m_currentActionNode);
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

      m_pae->execute(currPAN);

      // find next primitive action node to execute
      auto last = currPAN;
      currPAN = nullptr;
      for (auto &child : last->getChildren()) {
        if (child->getPid() == pid) {
          currPAN = child;
        }
      }
    }

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
  std::cout << pid << " has finished all tasks" << std::endl;

  m_isEnd = true;
}