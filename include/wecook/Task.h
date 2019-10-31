//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_TASK_H
#define WECOOK_TASK_H

#include <iostream>
#include <vector>

#include "Action.h"
#include "Object.h"

namespace wecook {
/*!
 * Task is a general class for task description, it matches with the taskMsg.
 */
class Task {
 public:
  /*! Type is an enum */
  enum Type {
    Planning, /*! planning task, currently only supports single agent \todo #9 */
    Following, /*! following task */
  };

  /*! motionPlannerType is an enum */
  enum MotionPlannerType {
    RRTConnect, /*! http://www.kuffner.org/james/papers/kuffner_icra2000.pdf */
  };

  struct Agent {
    Agent(const std::string &pid, const std::string &agentType, const std::vector<double> &pose) {
      m_pid = pid;
      m_type = agentType;
      m_pose = pose;
    }
    std::string m_pid;
    std::string m_type;
    std::vector<double> m_pose;
  };

  Task() = default;

  void addSubgoal(const Action &action) {
    m_subgoals.emplace_back(action);
  }

  void addObject(const Object &object) {
    m_objects.emplace_back(object);
  }

  void addContainingPair(const std::string &containerName, const std::string &objectName) {
    m_containingPairs.emplace_back(std::pair<std::string, std::string>(containerName, objectName));
  }

  void addPDDLDomain(const std::string &PDDLDomain) {
    m_PDDLDomain = PDDLDomain;
  }

  void setSim(const uint8_t &ifSim) {
    m_ifSim = ifSim;
  }

  void addPDDLProblem(const std::string &PDDLProblem) {
    m_PDDLProblem = PDDLProblem;
  }

  void addTaskType(const std::string &taskType) {
    m_type = taskType == "plan" ? Planning : Following;
  }

  void addAgent(const std::string &pid, const std::string &agentType, const std::vector<double> &pose) {
    m_agents.emplace_back(Agent(pid, agentType, pose));
  }

  void addMotionPlannerType(const std::string &type) {
    if (type == "RRTConnect") {
      m_motionPlannerType = RRTConnect;
    }
  }

  bool ifSim() const {
    return m_ifSim;
  }

  MotionPlannerType getMotionPlannerType() {
    return m_motionPlannerType;
  }

  std::vector<Agent> getAgents() const {
    return m_agents;
  }

  Type getTaskType() const {
    return m_type;
  }

  std::vector<Action> getSubgoals() const {
    return m_subgoals;
  }

  std::vector<Object> getObjects() const {
    return m_objects;
  }

  std::vector<std::pair<std::string, std::string>> getContainingPairs() const {
    return m_containingPairs;
  }

  std::string getPDDLDomain() const {
    return m_PDDLDomain;
  }

  std::string getPDDLProblem() const {
    return m_PDDLProblem;
  }

 private:
  std::vector<Action> m_subgoals;
  std::vector<Object> m_objects;
  std::vector<std::pair<std::string, std::string>> m_containingPairs;
  std::string m_PDDLDomain;
  std::string m_PDDLProblem;
  Type m_type; /*! indicate the task type */
  std::vector<Agent> m_agents; /*! array of agents */
  MotionPlannerType
      m_motionPlannerType = RRTConnect; /*! use it to decide if we are using dynamic motion planning algorithms*/
  bool m_ifSim = true;
};
}

#endif //WECOOK_TASK_H
