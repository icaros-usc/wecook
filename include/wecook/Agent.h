//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_AGENT_H
#define WECOOK_AGENT_H

#include <boost/thread.hpp>
#include <libada/Ada.hpp>
#include <Eigen/Dense>

#include "Action.h"
#include "MotionNode.h"
#include "ConfMotionNode.h"
#include "GrabMotionNode.h"
#include "TSRMotionNode.h"
#include "LinearDeltaMotionNode.h"
#include "TaskGraph.h"
#include "PrimitiveActionExecutor.h"

namespace wecook {

typedef boost::function<void(ActionNode *)> syncCallback;

class Agent {

 public:
  Agent(const std::string &pid,
        bool ifSim)
      : m_thread(&Agent::run, this),
        m_pid(pid),
        m_ifSim(ifSim) {
  }

  void stop();

  inline bool isFree() {
    return m_isFree;
  }

  inline bool isEnd() {
    return m_isEnd;
  }

  inline void addNewTask(std::shared_ptr<TaskGraph> &taskGraph, syncCallback callback) {
    m_syncCallback = callback;
    m_currentTask = taskGraph;
  }

  inline ActionNode *getCurrentActionNode() {
    return m_currentActionNode;
  }

  inline bool ifSim() {
    return m_ifSim;
  }

  virtual std::string getType() = 0;

  virtual void init(std::shared_ptr<aikido::planner::World> &env) = 0;

  virtual Eigen::Vector3d getPosition() = 0;

 protected:
  void run();

  std::string m_pid;

  boost::thread m_thread;
  bool m_isFree = true;
  bool m_isEnd = false;
  ActionNode *m_currentActionNode;
  std::shared_ptr<TaskGraph> m_currentTask = nullptr;
  syncCallback m_syncCallback;
  PrimitiveActionExecutor m_pap;
  bool m_ifSim;
};
}
#endif //WECOOK_AGENT_H
