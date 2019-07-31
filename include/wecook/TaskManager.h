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
#include "wecook/Robots.h"

namespace wecook {
class TaskManager {
 public:
  TaskManager(const ros::NodeHandle &n, const std::map<std::string, std::shared_ptr<Robot>> &robots);

  void start();
  void stop(int signum);
  void run();
 private:
  std::queue<Task> m_taskQueue;
  bool m_isEnd;
  ros::Subscriber m_Listener;
  ros::NodeHandle m_nh;
  Robots m_robots;

  void addNewTask(const TaskMsg::ConstPtr &msg);

  int execute(Task &task);
};
}

#endif //WECOOK_TASKMANAGER_H
