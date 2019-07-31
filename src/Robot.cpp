//
// Created by hejia on 7/30/19.
//

#include <iostream>

#include "wecook/Robot.h"

using namespace wecook;

int Robot::execute(Action &action) {
  m_action.emplace_back(action);
}

void Robot::stop() {
  m_isEnd = true;
  m_thread.join();
}

void Robot::run() {
  while (!m_isEnd) {
    if (!m_action.empty()) {
      m_isFree = false;
      Action action = m_action[0];

      // action execution
      std::cout << action.get_pid() << " "
                << action.get_verb() << " "
                << action.get_tool() << " "
                << action.get_location() << " "
                << action.get_ingredients()[0] << std::endl;
      m_action.pop_back();
    }
    m_isFree = true;
  }
}

