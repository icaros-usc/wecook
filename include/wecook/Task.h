//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_TASK_H
#define WECOOK_TASK_H

#include <iostream>

#include "Action.h"
#include "Object.h"

namespace wecook {
/*!
 * Task is a general class for task description, it matches with the taskMsg.
 */
class Task {
 public:
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

  void addPDDLProblem(const std::string &PDDLProblem) {
    m_PDDLProblem = PDDLProblem;
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

 private:
  std::vector<Action> m_subgoals;
  std::vector<Object> m_objects;
  std::vector<std::pair<std::string, std::string>> m_containingPairs;
  std::string m_PDDLDomain;
  std::string m_PDDLProblem;
};
}

#endif //WECOOK_TASK_H
