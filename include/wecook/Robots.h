//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_ROBOTS_H
#define WECOOK_ROBOTS_H

#include <vector>
#include <map>
#include <boost/thread.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <libada/Ada.hpp>

#include "Task.h"
#include "Robot.h"
#include "utils.h"

namespace wecook {
class Robots {
 public:
  Robots(const std::map<std::string, std::shared_ptr<Robot>> &robots) : m_thread(&Robots::run, this), m_robots(robots) {
    m_env = std::make_shared<aikido::planner::World>("wecook");
    // construct robot in the env
    for (const auto &robot : m_robots) {
      robot.second->createAda(m_env);
    }
    // add table in the env
    std::string tableURDFUri = "package://wecook_assets/data/furniture/table.urdf";
    std::vector<double> tablePose = std::vector<double>();
    tablePose.emplace_back(0.5);
    tablePose.emplace_back(0.0);
    tablePose.emplace_back(0.0);
    tablePose.emplace_back(0.707107);
    tablePose.emplace_back(0);
    tablePose.emplace_back(0);
    tablePose.emplace_back(0.707107);
    addBodyFromURDF(m_env.get(), tableURDFUri, tablePose);

    m_viewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(m_env, "wecook", "map");
    m_viewer->setAutoUpdate(true);
    ros::Duration(1.).sleep();
  }

  int execute(Task &task);
  inline bool isFree() {
    return m_isFree;
  }

  void stop();

 private:
  void run();

  aikido::rviz::WorldInteractiveMarkerViewerPtr m_viewer = nullptr;
  aikido::planner::WorldPtr m_env = nullptr;
  boost::thread m_thread;
  std::map<std::string, std::shared_ptr<Robot>> m_robots;
  bool m_isFree = true;
  bool m_isEnd = false;
  std::vector<Task> m_task;
};
}
#endif //WECOOK_ROBOTS_H
