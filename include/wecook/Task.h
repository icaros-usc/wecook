//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_TASK_H
#define WECOOK_TASK_H

#include "Action.h"

namespace wecook {
class Task {
 public:
  Task() = default;

  void addSubgoal(const Action &action) {
    m_subgoals.emplace_back(action);
  }

 private:
  std::vector<Action> m_subgoals;
};
}

#endif //WECOOK_TASK_H
