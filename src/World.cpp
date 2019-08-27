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

void World::initialize() {
  m_env = std::make_shared<aikido::planner::World>("wecook");

  for (const auto &agent : m_agents) {
    agent.second->init(m_env);
  }

  m_viewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(m_env, "wecook", "map");
  m_viewer->setAutoUpdate(true);
  ros::Duration(1.).sleep();
}

void World::stop() {
  for (auto &agent : m_agents) {
    agent.second->stop();
  }
  m_isEnd = true;
  m_thread.join();
}

void World::run() {
  while (!m_isEnd) {
    if (!m_tasks.empty()) {
      m_isFree = false;
      Task task = m_tasks[0];
      setupTask(task);
      std::vector<Action> subgoals = task.getSubgoals();
      auto taskGraph = std::make_shared<TaskGraph>(subgoals);
      auto primitiveTaskGraph = std::make_shared<PrimitiveTaskGraph>();
      // transfer taskGraph to primitive taskGraph
      m_actionPlanner.compile(taskGraph, primitiveTaskGraph, m_agents, m_containingMap);

      // now dispatch taskgraph to all agents
      for (const auto &agent : m_agents) {
        agent.second->addNewTask(taskGraph, boost::bind(&World::syncToActionNode, this, ::_1));
      }

      // wait for agents to be free
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

void World::setupTask(const Task &task) {
  std::vector<Object> objects = task.getObjects();

  m_objectMgr = std::make_shared<ObjectMgr>();

  m_objectMgr->init(objects, m_ifSim, m_env);

  m_containingMap = std::make_shared<ContainingMap>(task, m_objectMgr, m_env);
}

void World::clean(const Task &task) {
  std::vector<Object> objects = task.getObjects();

  m_containingMap->unconnectAll();
  m_containingMap.reset();

  m_objectMgr->clear(objects, m_ifSim, m_env);
  m_objectMgr.reset();
}

void World::syncToActionNode(ActionNode *actionNode) {
  auto pids = actionNode->getAction().get_pids();
  for (const auto &pid : pids) {
    auto robot = m_agents[pid];
    while (robot->getCurrentActionNode() != actionNode) ros::Duration(0.5).sleep();
  }
}
