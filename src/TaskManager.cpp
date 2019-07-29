#include "wecook/TaskManager.h"

using namespace wecook;

TaskManager::TaskManager(const ros::NodeHandle &nh) : m_nh(nh), m_isEnd(false) {

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
  m_taskQueue.push(task);
}

void TaskManager::run() {
  ros::Rate loop_rate(10);
  while (!m_isEnd) {
    ros::spinOnce();
    loop_rate.sleep();
    if (!m_taskQueue.empty()) {
      Task task = m_taskQueue.front();
      this->execute(task);
      m_taskQueue.pop();
    }
  }
}

int TaskManager::execute(Task &task) {
  task.print();
}

