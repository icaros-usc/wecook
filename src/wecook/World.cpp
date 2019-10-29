//
// Created by hejia on 7/30/19.
//

#include "wecook/TaskGraph.h"
#include "wecook/World.h"
#include "wecook/utils.h"
#include "wecook/Robot.h"
#include "wecook/PrimitiveTaskGraph.h"

using namespace wecook;

int World::addTask(wecook::Task &task) {
  m_tasks.emplace_back(task);
}

void World::initialize() {
  m_env = std::make_shared<aikido::planner::World>("wecook");
  m_viewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(m_env, "wecook", "map");
  m_viewer->setAutoUpdate(true);
  ros::Duration(1.).sleep();
}

void World::stop() {
  m_isEnd = true;
  m_thread.join();
}

void World::run() {
  while (!m_isEnd) {
    if (!m_tasks.empty()) {
      m_isFree = false;
      Task task = m_tasks[0]; // get the first element in task queue
      // We have two modes planning and following
      auto taskType = task.getTaskType();
      if (taskType == Task::Planning) {
        // TODO
      } else {
        // Do following mode
        setupFollowingTask(task);

        std::vector<Action> subgoals = task.getSubgoals();
        auto taskGraph = std::make_shared<TaskGraph>(subgoals);

        // transfer taskGraph to primitive taskGraph
        m_actionPlanner.compile(taskGraph, m_agents, m_containingMap, m_objectMgr);

        // now dispatch taskgraph to all agents
        for (const auto &agent : m_agents) {
          auto taskExecutorThread = std::make_shared<TaskExecutorThread>(agent.second,
                                                                         m_primitiveActionExecutor,
                                                                         taskGraph,
                                                                         boost::bind(&World::syncToActionNode,
                                                                                     this,
                                                                                     ::_1));
          m_mapTaskExecutorThread.insert(std::make_pair(agent.first, taskExecutorThread));
        }

        // wait for agents to be free
        for (const auto &taskExecutorThread : m_mapTaskExecutorThread) {
          while (!taskExecutorThread.second->isEnd()) {
            ros::Duration(0.5).sleep();
          }
        }

        cleanFollowingTask(task);
        m_tasks.pop_back();
      }
    }
    m_isFree = true;
  }
}

void World::setupFollowingTask(const Task &task) {
  // First add agents to the world
  auto agents = task.getAgents();

  for (const auto &agent : agents) {
    // create each agent
    std::vector<double> pose = agent.m_pose;
    auto transform = vectorToIsometry(pose);
    if (agent.m_type == "r") {
      // This is a robot agent
      auto pAgent = std::make_shared<Robot>(transform, agent.m_pid, m_ifSim);
      addAgent(agent.m_pid, pAgent);
    }
  }

  for (const auto &agent : m_agents) {
    agent.second->init(m_env);
  }

  std::vector<Object> objects = task.getObjects();

  m_objectMgr = std::make_shared<ObjectMgr>();

  m_objectMgr->init(objects, m_ifSim, m_env);

  m_containingMap = std::make_shared<ContainingMap>(task, m_objectMgr, m_env);

  // intialize primitive action executor
  m_primitiveActionExecutor = std::make_shared<PrimitiveActionExecutor>(m_agents, m_objectMgr, m_containingMap);
}

void World::cleanFollowingTask(const Task &task) {
  std::vector<Object> objects = task.getObjects();

  m_primitiveActionExecutor.reset();

  m_containingMap->unconnectAll();
  m_containingMap.reset();

  m_objectMgr->clear(objects, m_ifSim, m_env);
  m_objectMgr.reset();

  m_mapTaskExecutorThread.clear();

  // Remove all agents
  for (auto &agent : m_agents) {
    agent.second->end();
  }
}

void World::setupPlanningTask(const Task &task) {
  // 1. Parse PDDL files
  auto pddlDomain = task.getPDDLDomain();
  auto pddlProblem = task.getPDDLProblem();

  // TODO
}

void World::cleanPlanningTask(const Task &task) {
  // TODO
}

void World::syncToActionNode(ActionNode *actionNode) {
  auto pids = actionNode->getAction().get_pids();
  for (const auto &pid : pids) {
    auto thread = m_mapTaskExecutorThread[pid];
    while (thread->getCurrentActionNode() != actionNode) ros::Duration(0.5).sleep();
  }
}
