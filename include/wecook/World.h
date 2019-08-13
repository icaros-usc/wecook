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
#include "Robot.h"
#include "utils.h"
#include "ActionPlanner.h"
#include "ContainingMap.h"

namespace wecook {
class World {
 public:
  World(const std::map<std::string, std::shared_ptr<Robot>> &robots) : m_thread(&World::run, this), m_robots(robots) {
    m_env = std::make_shared<aikido::planner::World>("wecook");
    // construct robot in the env
    for (const auto &robot : m_robots) {
      robot.second->createAda(m_env);
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

 private:
  void run();

  void setup(const Task &task);

  void clean(const Task &task);

  aikido::rviz::WorldInteractiveMarkerViewerPtr m_viewer = nullptr;
  aikido::planner::WorldPtr m_env = nullptr;
  boost::thread m_thread;
  std::map<std::string, std::shared_ptr<Robot>> m_robots;
  bool m_isFree = true;
  bool m_isEnd = false;
  std::vector<Task> m_tasks;
  ActionPlanner m_actionPlanner;
  std::shared_ptr<ContainingMap> m_containingMap = nullptr;
};
}
#endif //WECOOK_WORLD_H
