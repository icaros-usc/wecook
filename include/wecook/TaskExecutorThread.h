//
// Created by hejia on 8/27/19.
//

#ifndef WECOOK_TASKEXECUTORTHREAD_H
#define WECOOK_TASKEXECUTORTHREAD_H

#include "Agent.h"
#include "ActionNode.h"
#include "PrimitiveActionExecutor.h"
#include "TaskGraph.h"

namespace wecook {

class TaskExecutorThread {
  typedef boost::function<void(ActionNode *)> syncCallback;

 public:
  TaskExecutorThread(const std::shared_ptr<Agent> &agent,
                     const std::shared_ptr<PrimitiveActionExecutor> &pae,
                     const std::shared_ptr<TaskGraph> &taskGraph,
                     syncCallback callback)
      : m_pae(pae),
        m_agent(agent),
        m_currentTask(taskGraph),
        m_syncCallback(callback),
        m_thread(&TaskExecutorThread::run, this) {

  }

  bool isEnd() {
    return m_isEnd;
  }

  inline ActionNode *getCurrentActionNode() const {
    return m_currentActionNode;
  }

 private:
  void run();

  syncCallback m_syncCallback;
  std::shared_ptr<TaskGraph> m_currentTask = nullptr;
  std::shared_ptr<Agent> m_agent;
  std::shared_ptr<PrimitiveActionExecutor> m_pae;
  boost::thread m_thread;
  ActionNode *m_currentActionNode = nullptr;

  bool m_isEnd = false;
};

}

#endif //WECOOK_TASKEXECUTORTHREAD_H