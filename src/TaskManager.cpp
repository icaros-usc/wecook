#include "wecook/TaskManager.h"

using namespace wecook;

std::vector<Task> TaskManager::m_taskQueue{};

TaskManager::TaskManager(ros::NodeHandle nh) : m_nh(nh) {

}

void TaskManager::start() {
  m_Listener = m_nh.subscribe("WeCookDispatch", 1000, TaskManager::addNewTask);
}

void TaskManager::addNewTask(const TaskMsg::ConstPtr &msg) {
  Task task = Task();
  for (auto actionMsg : msg->actions) {
    Action action(actionMsg.location, actionMsg.ingredients, actionMsg.verb, actionMsg.tool);
    task.addSubgoal(action);
  }
  TaskManager::m_taskQueue.emplace_back(task);
}

