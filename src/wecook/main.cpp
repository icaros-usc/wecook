#include <ros/ros.h>
#include <std_msgs/String.h>
#include <utility>
#include <csignal>

#include "wecook/TaskManager.h"
#include "wecook/Robot.h"

using namespace wecook;

static TaskManager *taskManager;
extern "C" void sig_handler(int signum) { taskManager->stop(signum); }

int main(int argc, char **argv) {
  signal(SIGINT, sig_handler);
  ros::init(argc, argv, "wecook", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  // first construct world
  std::shared_ptr<World> world = std::make_shared<World>(true);

  // initialize world
  world->init();

  taskManager = new TaskManager(n);
  taskManager->setWorld(world);
  taskManager->start();

  boost::thread t{boost::bind(&TaskManager::run, taskManager)};

  ros::spin();

  delete taskManager;

  return 0;
}

