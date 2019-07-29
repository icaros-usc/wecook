#include <ros/ros.h>
#include <std_msgs/String.h>
#include "wecook/TaskManager.h"

using namespace wecook;

int main(int argc, char **argv) {
  ros::init(argc, argv, "wecook");
  ros::NodeHandle n;

  TaskManager taskManager = TaskManager(n);
  taskManager.start();

  ros::spin();

  return 0;
}

