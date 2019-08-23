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

namespace wecook {
class World {
 public:
  World(const std::map<std::string, std::shared_ptr<Agent>> &robots) : m_thread(&World::run, this), m_robots(robots) {
    m_env = std::make_shared<aikido::planner::World>("wecook");
    // construct robot in the env
    for (const auto &robot : m_robots) {
      robot.second->createAda(m_env);
      robot.second->moveToHome();
      auto skeleton = robot.second->m_ada->getArm()->getMetaSkeleton();
      ROS_INFO_STREAM("Agent initial position: " << skeleton->getPositions());
    }

    m_viewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(m_env, "wecook", "map");
    m_viewer->setAutoUpdate(true);
    ros::Duration(1.).sleep();
  }

  int addTask(Task &task);

  inline bool isFree() {
    return m_isFree;
  }

  void stop();

  void syncToActionNode(ActionNode *actionNode);

 private:
  void run();

  void setup(const Task &task);

  void clean(const Task &task);

  aikido::rviz::WorldInteractiveMarkerViewerPtr m_viewer = nullptr;
  aikido::planner::WorldPtr m_env = nullptr;
  boost::thread m_thread;
  std::map<std::string, std::shared_ptr<Agent>> m_robots;
  bool m_isFree = true;
  bool m_isEnd = false;
  std::vector<Task> m_tasks;
  ActionPlanner m_actionPlanner;
  std::shared_ptr<ContainingMap> m_containingMap = nullptr;
};
}
#endif //WECOOK_WORLD_H
