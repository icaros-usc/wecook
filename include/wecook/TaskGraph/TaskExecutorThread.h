//
// Created by hejia on 8/27/19.
//

#ifndef WECOOK_TASKEXECUTORTHREAD_H
#define WECOOK_TASKEXECUTORTHREAD_H

#include "wecook/Agents/Agent.h"
#include "wecook/TaskGraph/ActionNode.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveActionExecutor.h"
#include "wecook/TaskGraph/TaskGraph.h"

namespace wecook {

/*!
 * TaskExecutorThread is a class for executing tasks for each agent.
 * We need it mostly because of the need for task-level communication between agents.
 */
class TaskExecutorThread {
  typedef boost::function<void(ActionNode *)> syncCallback;

 public:
  TaskExecutorThread(const std::shared_ptr<Agent> &agent,
                     const std::shared_ptr<PrimitiveActionExecutor> &pae,
                     const std::shared_ptr<TaskGraph> &taskGraph)
      : m_pae(pae),
        m_agent(agent),
        m_currentTask(taskGraph),
        m_thread(&TaskExecutorThread::run, this) {

  }

  bool isEnd() {
    return m_isEnd;
  }

  inline ActionNode *getCurrentActionNode() const {
    return m_currentActionNode;
  }

  void setMotionMutex(boost::mutex *motionMutex) {
    m_motionMutex = motionMutex;
  }

 private:
  void run();

  std::shared_ptr<TaskGraph> m_currentTask = nullptr;
  std::shared_ptr<Agent> m_agent;
  std::shared_ptr<PrimitiveActionExecutor> m_pae;
  boost::thread m_thread;
  ActionNode *m_currentActionNode = nullptr;
  bool m_isEnd = false;
  boost::mutex *m_motionMutex = nullptr;
};

}

#endif //WECOOK_TASKEXECUTORTHREAD_H
