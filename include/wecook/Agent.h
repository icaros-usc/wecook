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

namespace wecook {

typedef boost::function<void(ActionNode *)> syncCallback;

class Agent {

 public:
  Agent(const Eigen::Isometry3d &transform,
        const std::string &pid,
        std::vector<double> homePositions = std::vector<double>{4.8, 2.9147, 1.009, 4.1957, 1.44237, 1.3166})
      : m_thread(&Agent::run, this),
        m_transform(transform),
        m_pid(pid) {
    m_homePositions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(homePositions.data(), homePositions.size());
  }

  void stop();

  inline void addSubMotions(const std::vector<std::shared_ptr<MotionNode>> &subMotions) {
    m_subMotions = subMotions;
  }

  inline bool isFree() {
    return m_isFree;
  }

  inline bool isEnd() {
    return m_isEnd;
  }

  inline aikido::planner::WorldPtr getWorld() {
    return m_ada->getWorld();
  }

  inline auto getArm() {
    return m_ada->getArm();
  }

  inline auto getHand() {
    return m_ada->getHand();
  }

  inline auto getStateSpace() {
    return m_ada->getStateSpace();
  }

  inline auto getMetaSkeleton() {
    return m_ada->getMetaSkeleton();
  }

  inline auto executeTrajectory(aikido::trajectory::TrajectoryPtr traj) {
    return m_ada->executeTrajectory(traj);
  }

  void createAda(const aikido::planner::WorldPtr &env) {
    m_ada = std::make_shared<ada::Ada>(env, true, m_pid, m_transform);
  }

  void moveToHome();

  inline void addNewTask(std::shared_ptr<TaskGraph> &taskGraph, syncCallback callback) {
    m_syncCallback = callback;
    m_currentTask = taskGraph;
  }

  inline ActionNode *getCurrentActionNode() {
    return m_currentActionNode;
  }

  std::shared_ptr<ada::Ada> m_ada = nullptr;

 private:
  void run();

  std::string m_pid;

  boost::thread m_thread;
  bool m_isFree = true;
  bool m_isEnd = false;
  Eigen::Isometry3d m_transform;
  std::vector<std::shared_ptr<MotionNode>> m_subMotions;
  Eigen::VectorXd m_homePositions;
  ActionNode *m_currentActionNode;
  std::shared_ptr<TaskGraph> m_currentTask = nullptr;
  syncCallback m_syncCallback;
};
}
#endif //WECOOK_AGENT_H
