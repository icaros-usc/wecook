//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_TASK_H
#define WECOOK_TASK_H

#include <iostream>

#include "Action.h"
#include "Object.h"

namespace wecook {
class Task {
 public:
  Task() = default;

  void addSubgoal(const Action &action) {
    m_subgoals.emplace_back(action);
  }

  void addObject(const Object &object) {
    m_objects.emplace_back(object);
  }

  void print() {
    for (auto &action : m_subgoals) {
      std::cout << action.get_verb()
                << action.get_tool()
                << action.get_location()
                << action.get_ingredients()[0] << std::endl;
    }
  }

  std::vector<Action> getSubgoals() {
    return m_subgoals;
  }

  std::vector<Object> getObjects() {
    return m_objects;
  }

 private:
  std::vector<Action> m_subgoals;
  std::vector<Object> m_objects;
};
}

#endif //WECOOK_TASK_H
