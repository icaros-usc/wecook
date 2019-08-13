#include "wecook/TaskManager.h"

using namespace wecook;

TaskManager::TaskManager(const ros::NodeHandle &nh, const std::map<std::string, std::shared_ptr<Robot>> &robots) : m_nh(nh), m_isEnd(false), m_world(robots) {

}

void TaskManager::start() {
  m_Listener = m_nh.subscribe("WeCookDispatch", 1000, &TaskManager::addNewTask, this);
}

void TaskManager::stop(int signum) {
  ROS_INFO("SIGINT received... Stopping...");
  m_world.stop();
  m_isEnd = true;
  ros::shutdown();
}

void TaskManager::addNewTask(const TaskMsg::ConstPtr &msg) {
  ROS_INFO("Receive Msg");
  Task task = Task();
  for (const auto &objectMsg : msg->scene.objects) {
    Object object(objectMsg.name, objectMsg.url, objectMsg.pose);
    task.addObject(object);
  }
  for (const auto &actionMsg : msg->actions) {
    Action action(actionMsg.pid, actionMsg.location, actionMsg.ingredients, actionMsg.verb, actionMsg.tool);
    task.addSubgoal(action);
  }
  for (const auto &containingMsg : msg->scene.containingMap) {
    task.addContainingPair(containingMsg.pair[0], containingMsg.pair[1]);
  }
  m_taskQueue.push(task);
}

void TaskManager::run() {
  while (!m_isEnd) {
    if (!m_taskQueue.empty() && m_world.isFree()) {
      Task task = m_taskQueue.front();
      m_world.addTask(task);
      m_taskQueue.pop();
    }
  }
}
