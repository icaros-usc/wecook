#include <ros/ros.h>
#include <std_msgs/String.h>
#include <utility>
#include <csignal>

#include "wecook/TaskManager.h"

using namespace wecook;

static TaskManager * taskManager;
extern "C" void sig_handler(int signum) {taskManager->stop(signum);}

int main(int argc, char **argv) {
  signal(SIGINT, sig_handler);
  ros::init(argc, argv, "wecook", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  std::map<std::string, std::shared_ptr<Robot>> robots{};
  Eigen::Isometry3d robotPose1 = Eigen::Isometry3d::Identity();
  robotPose1.translation() = Eigen::Vector3d(-0.15, -0.35, 0.3);
  std::string robotName1 = "j2n6s200_1";
  std::shared_ptr<Robot> pRobot1 = std::make_shared<Robot>(robotPose1, robotName1);
  Eigen::Isometry3d robotPose2 = Eigen::Isometry3d::Identity();
  robotPose2.translation() = Eigen::Vector3d(-0.15, 0.35, 0.3);
  std::string robotName2 = "j2n6s200_2";
  std::shared_ptr<Robot> pRobot2 = std::make_shared<Robot>(robotPose2, robotName2);
  robots.emplace(std::pair<std::string, std::shared_ptr<Robot>>{"p1", pRobot1});
  robots.emplace(std::pair<std::string, std::shared_ptr<Robot>>{"p2", pRobot2});

  taskManager = new TaskManager(n, robots);
  taskManager->start();

//  ros::spin();
  boost::thread t{boost::bind(&TaskManager::run, taskManager)};

//  t.join();
  ros::spin();

  delete taskManager;

  return 0;
}

