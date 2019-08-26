//
// Created by hejia on 7/30/19.
//

#include "wecook/TaskGraph.h"
#include "wecook/World.h"
#include "wecook/utils.h"
#include "wecook/PrimitiveTaskGraph.h"

using namespace wecook;

int World::addTask(wecook::Task &task) {
  m_tasks.emplace_back(task);
}

void World::stop() {
  for (auto &robot : m_agents) {
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
      auto taskGraph = std::make_shared<TaskGraph>(subgoals);
      auto primitiveTaskGraph = std::make_shared<PrimitiveTaskGraph>();
      // transfer taskGraph to primitive taskGraph
      m_actionPlanner.compile(taskGraph, primitiveTaskGraph, m_agents, m_containingMap);

      // now dispatch taskgraph to all agents
      for (const auto &agent : m_agents) {
        agent.second->addNewTask(taskGraph, boost::bind(&World::syncToActionNode, this, ::_1));
      }

      // wait for robots to be free
      for (const auto &agent : m_agents) {
        while (!agent.second->isFree()) {
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
  m_containingMap = std::make_shared<ContainingMap>(task, m_env);
}

void World::clean(const Task &task) {
  std::vector<Object> objects = task.getObjects();
  for (auto &object : objects) {
    auto skeleton = m_env->getSkeleton(object.getName());
    m_env->removeSkeleton(skeleton);
  }
  m_containingMap->unconnectAll();
  m_containingMap.reset();
}

void World::syncToActionNode(ActionNode *actionNode) {
  auto pids = actionNode->getAction().get_pids();
  for (const auto &pid : pids) {
    auto robot = m_agents[pid];
    while (robot->getCurrentActionNode() != actionNode) ros::Duration(0.5).sleep();
  }
}
