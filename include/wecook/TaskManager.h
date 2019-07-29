//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_TASKMANAGER_H
#define WECOOK_TASKMANAGER_H

#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include "wecook/Task.h"
#include "wecook/Action.h"
#include "wecook/TaskMsg.h"
#include "wecook/ActionMsg.h"

namespace wecook {
class TaskManager {
 public:
  TaskManager(const ros::NodeHandle &n);

  void start();
 private:
  std::vector<Task> m_taskQueue;
  bool m_isFree;
  ros::Subscriber m_Listener;
  ros::NodeHandle m_nh;

  void addNewTask(const TaskMsg::ConstPtr &msg);
};
}

#endif //WECOOK_TASKMANAGER_H
