//
// Created by hejia on 7/30/19.
//

#include "wecook/Robots.h"

using namespace wecook;

int Robots::execute(wecook::Task &task) {
  m_task.emplace_back(task);
}

void Robots::stop() {
  for (auto &robot : m_robots) {
    robot.second->stop();
  }
  m_isEnd = true;
  m_thread.join();
}

void Robots::run() {
  while (!m_isEnd) {
    if (!m_task.empty()) {
      m_isFree = false;
      Task task = m_task[0];
      // TODO setup the scene
      std::vector<Action> subgoals = task.getSubgoals();
      for (auto &action : subgoals) {
        std::string pid = action.get_pid();
        while (!m_robots[pid]->isFree()) {
        }
        m_robots[pid]->execute(action);
      }
      m_task.pop_back();
    }
    m_isFree = true;
  }
}