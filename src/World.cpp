//
// Created by hejia on 7/30/19.
//

#include "wecook/World.h"
#include "wecook/utils.h"

using namespace wecook;

int World::addTask(wecook::Task &task) {
  m_tasks.emplace_back(task);
}

void World::stop() {
  for (auto &robot : m_robots) {
    robot.second->stop();
  }
  m_isEnd = true;
  m_thread.join();
}

void World::run() {
  while (!m_isEnd) {
    if (!m_tasks.empty()) {
      m_isFree = false;
      Task task = m_tasks[0];
      setup(task);
      std::vector<Action> subgoals = task.getSubgoals();
      for (auto &action : subgoals) {
        std::vector<std::string> pids = action.get_pid();
        for (const auto &pid : pids) {
          while (!m_robots[pid]->isFree()) {
          }
        }
        m_actionPlanner.plan(action, m_robots);
      }
      // wait for robots to be free
      for (const auto &robot : m_robots) {
        while (!robot.second->isFree()) {
        }
      }
      clean(task);
      m_tasks.pop_back();
    }
    m_isFree = true;
  }
}

void World::setup(const Task &task) {
  std::vector<Object> objects = task.getObjects();
  for (auto &object : objects) {
    addBodyFromURDF(m_env.get(), object.getUrl(), object.getPose(), object.getName());
  }
}

void World::clean(const Task &task) {
  std::vector<Object> objects = task.getObjects();
  for (auto &object : objects) {
    auto skeleton = m_env->getSkeleton(object.getName());
    m_env->removeSkeleton(skeleton);
  }
}