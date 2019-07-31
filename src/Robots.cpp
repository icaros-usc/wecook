//
// Created by hejia on 7/30/19.
//

#include "wecook/Robots.h"

using namespace wecook;

int Robots::execute(wecook::Task &task) {
  m_task.emplace_back(task);
}

void Robots::run() {
  while (!m_isEnd) {
    if (!m_task.empty()) {
      m_isFree = false;
      Task task = m_task[0];
      std::vector<Action> subgoals = task.getSubgoals();
      for (auto &action : subgoals) {
        std::cout << action.get_pid() << " "
                  << action.get_verb() << " "
                  << action.get_tool() << " "
                  << action.get_location() << " "
                  << action.get_ingredients()[0] << std::endl;
      }
      m_task.pop_back();
    }
    m_isFree = true;
  }
}