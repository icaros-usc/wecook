//
// Created by hejia on 8/27/19.
//

#ifndef WECOOK_TASKEXECUTORTHREAD_H
#define WECOOK_TASKEXECUTORTHREAD_H

#include "Agent.h"

namespace wecook {

class TaskExecutorThread {
 public:
  TaskExecutorThread(std::shared_ptr<Agent> &agent,
                     std::shared_ptr<PrimitiveActionExecutor> &pae,
                     std::shared_ptr<TaskGraph> &taskGraph,
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
