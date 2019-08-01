//
// Created by hejia on 7/30/19.
//

#include "wecook/Robots.h"
#include "wecook/utils.h"

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
      std::vector<Object> objects = task.getObjects();
      for (auto &object : objects) {
        addBodyFromURDF(m_env.get(), object.getUrl(), object.getPose());
      }
      std::vector<Action> subgoals = task.getSubgoals();
      for (auto &action : subgoals) {
        std::vector<std::string> pids = action.get_pid();
        for (const auto &pid : pids) {
          while (!m_robots[pid]->isFree()) {
          }
        }
        m_actionPlanner.plan(action, m_robots);
        // Add skill call back
        // m_robots[pid]->execute(action);
      }
      m_task.pop_back();
    }
    m_isFree = true;
  }
}