//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_TASKMANAGER_H
#define WECOOK_TASKMANAGER_H

#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <boost/thread.hpp>
#include <queue>

#include "wecook/Task.h"
#include "wecook/Action.h"
#include "wecook/TaskMsg.h"
#include "wecook/ActionMsg.h"
#include "wecook/World.h"
#include "wecook/Object.h"
#include "wecook/ObjectMsg.h"

namespace wecook {
class TaskManager {
 public:
  TaskManager(const ros::NodeHandle &n, const std::map<std::string, std::shared_ptr<Agent>> &agents);

  void start();
  void stop(int signum);
  void run();
 private:
  std::queue<Task> m_taskQueue;
  bool m_isEnd;
  ros::Subscriber m_Listener;
  ros::NodeHandle m_nh;
  World m_world;

  void addNewTask(const TaskMsg::ConstPtr &msg);
};
}

#endif //WECOOK_TASKMANAGER_H
