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

 private:
  void initialize();

  void run();

  void setupTask(const Task &task);

  void clean(const Task &task);

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
  std::vector<std::shared_ptr<TaskExecutorThread>> m_vecTaskExecutorThread;
};
}
#endif //WECOOK_WORLD_H
