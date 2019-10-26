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

  // create one robot
  Eigen::Isometry3d robotPose1 = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rot;
  rot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  robotPose1.translation() = Eigen::Vector3d(-0.2, 0.15, 0.7);
  robotPose1.linear() = rot;
  std::string pid1 = "p1";
  auto pRobot1 = std::make_shared<Robot>(robotPose1, pid1, world->ifSim());
  world->addAgent(pid1, pRobot1);
  // create another robot
  Eigen::Isometry3d robotPose2 = Eigen::Isometry3d::Identity();
  robotPose2.translation() = Eigen::Vector3d(-0.2, -0.20, 0.7);
  robotPose2.linear() = rot;
  std::string pid2 = "p2";
  auto pRobot2 = std::make_shared<Robot>(robotPose2, pid2, world->ifSim());
  world->addAgent(pid2, pRobot2);

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

