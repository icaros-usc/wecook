//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_WORLD_H
#define WECOOK_WORLD_H

#include <vector>
#include <map>
#include <boost/thread.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <libada/Ada.hpp>

#include "Task.h"
#include "Agent.h"
#include "utils.h"
#include "ActionPlanner.h"
#include "ContainingMap.h"
#include "ObjectMgr.h"
#include "TaskExecutorThread.h"

namespace wecook {
class World {
 public:
  World(bool ifSim) : m_ifSim(ifSim), m_thread(&World::run, this) {

  }

  void addAgent(const std::string &pid, const std::shared_ptr<Agent> &agent) {
    m_agents.emplace(std::pair<std::string, std::shared_ptr<Agent>>(pid, agent));
  }

  void init() {
    initialize();
  }

  int addTask(Task &task);

  inline bool isFree() {
    return m_isFree;
  }

  void stop();

  void syncToActionNode(ActionNode *actionNode);

  inline bool ifSim() {
    return m_ifSim;
  }

 private:
  void initialize();

  /**
   * Main thread runs, where world keeps execute the task in the m_tasks
   */
  void run();

  /**
   * This function sets up the world for a following task based on the task description.
   * @param task
   */
  void setupFollowingTask(const Task &task);

  /**
   * This function cleans up the world for a following task based on the task description.
   * @param task
   */
  void cleanFollowingTask(const Task &task);

  /**
   * This function sets up the world for a planning task based on the task description.
   * It will do following things:
   * 1. Parse the pddl files (Task).
   * 2. Parse the urdf files (Scene Graph).
   * 3. Construct the task-motion domain based on Task and Scene Graph.
   * ref: http://www.roboticsproceedings.org/rss12/p02.pdf
   * @param task
   */
  void setupPlanningTask(const Task &task);

  /**
   * This function cleans up the world for a planning task based on the task description.
   * @param task
   */
  void cleanPlanningTask(const Task &task);

  aikido::rviz::WorldInteractiveMarkerViewerPtr m_viewer = nullptr;
  aikido::planner::WorldPtr m_env = nullptr;
  boost::thread m_thread;
  std::map<std::string, std::shared_ptr<Agent>> m_agents;
  bool m_isFree = true;
  bool m_isEnd = false;
  std::vector<Task> m_tasks;
  ActionPlanner m_actionPlanner;
  std::shared_ptr<ContainingMap> m_containingMap = nullptr;
  bool m_ifSim = true;
  std::shared_ptr<ObjectMgr> m_objectMgr = nullptr;
  std::shared_ptr<PrimitiveActionExecutor> m_primitiveActionExecutor = nullptr;
  std::map<std::string, std::shared_ptr<TaskExecutorThread>> m_mapTaskExecutorThread;
};
}
#endif //WECOOK_WORLD_H
