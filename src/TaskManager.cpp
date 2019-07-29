#include "wecook/TaskManager.h"

using namespace wecook;

TaskManager::TaskManager(const ros::NodeHandle &nh) : m_nh(nh) {

}

void TaskManager::start() {
  m_Listener = m_nh.subscribe("WeCookDispatch", 1000, &TaskManager::addNewTask, this);
}

void TaskManager::addNewTask(const TaskMsg::ConstPtr &msg) {
  ROS_INFO("Receive Msg");
  Task task = Task();
  for (auto actionMsg : msg->actions) {
    Action action(actionMsg.location, actionMsg.ingredients, actionMsg.verb, actionMsg.tool);
    task.addSubgoal(action);
  }
  m_taskQueue.emplace_back(task);
}

