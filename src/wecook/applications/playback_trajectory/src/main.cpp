//
// Created by hejia on 10/30/19.
//

#include <fstream>
#include <iostream>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <libada/Ada.hpp>
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/trajectory/Interpolated.hpp"

#include <aikido/statespace/GeodesicInterpolator.hpp>

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::robot::Robot;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

dart::common::Uri adaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.urdf"};
dart::common::Uri adaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.srdf"};

static const double planningTimeout{5.};

void waitForUser(const std::string &msg) {
  ROS_INFO(msg.c_str());
  std::cin.get();
}

Eigen::VectorXd getCurrentConfig(ada::Ada &robot) {
  using namespace Eigen;
  IOFormat CommaInitFmt(
      StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  // TODO (Tapo): Change this back once the robot vs. arm is cleared
  auto defaultPose = robot.getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
  return defaultPose;
}

void executeTraj(
    ada::Ada &robot,
    const MetaSkeletonStateSpacePtr &armSpace,
    const MetaSkeletonPtr &armSkeleton,
    const MetaSkeletonStateSpacePtr &handSpace,
    const MetaSkeletonPtr &handSkeleton,
    const MetaSkeletonStateSpacePtr &metaSpace,
    const MetaSkeletonPtr &metaSkeleton) {
  // First we need to read in the trajectory file
  std::ifstream trajectoryFile("cached.txt");
  std::vector<std::vector<double>> trajectory;
  std::string line;
  while (getline(trajectoryFile, line)) {
    std::vector<double> joints;
    std::string item;
    std::stringstream ss(line);
    while (std::getline(ss, item, ' ')) {
      if (item != "\n") {
        joints.push_back(::atof(item.c_str()));
        ROS_INFO_STREAM(::atof(item.c_str()));
      }
    }
    trajectory.emplace_back(joints);
  }
  trajectoryFile.close();

  waitForUser("Playing back. Press [Enter] to proceed.");

  auto satisfied = std::make_shared<aikido::constraint::Satisfied>(armSpace);

  std::shared_ptr<aikido::trajectory::Interpolated> traj;
  std::shared_ptr<aikido::statespace::GeodesicInterpolator>
      interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(armSpace);
  traj = std::make_shared<aikido::trajectory::Interpolated>(armSpace, interpolator);

  double t = 0;
  for (auto &joints : trajectory) {
    auto state = armSpace->createState();
    auto positions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joints.data(), joints.size());
    armSpace->convertPositionsToState(positions, state);
    traj->addWaypoint(t, state);
    t += 0.3;
  }

  auto future = robot.executeTrajectory(std::move(traj));

  future.wait();
}

int main(int argc, char **argv) {
  bool adaSim = true;

  // Default options for flags
  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help", "Produce help message")(
      "adasim,a", po::bool_switch(&adaSim), "Run ADA in sim");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << po_desc << std::endl;
    return 0;
  }

  std::cout << "Simulation Mode: " << adaSim << std::endl;

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "simple_trajectories");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(
      new aikido::planner::World("simple_trajectories"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  ada::Ada robot(env, adaSim, adaUrdfUri, adaSrdfUri);
  auto robotSkeleton = robot.getMetaSkeleton();

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/simple_trajectories";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
          << execTopicName
          << "' InteractiveMarker topic in RViz.");
  aikido::rviz::InteractiveMarkerViewer viewer(execTopicName, baseFrameName, env);

  auto space = robot.getStateSpace();
  auto collision = robot.getSelfCollisionConstraint(space, robotSkeleton);

  dart::dynamics::MetaSkeletonPtr metaSkeleton = robot.getMetaSkeleton();
  auto metaSpace = std::make_shared<MetaSkeletonStateSpace>(metaSkeleton.get());

  auto armSkeleton = robot.getArm()->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());

  auto handSkeleton = robot.getHand()->getMetaSkeleton();
  auto handSpace = std::make_shared<MetaSkeletonStateSpace>(handSkeleton.get());

  if (adaSim) {
    auto home = std::vector<double>{4.8, 2.9147, 1.009, 4.1957, 1.44237, 1.3166};

    Eigen::VectorXd homeEigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(home.data(), home.size());

    armSkeleton->setPositions(homeEigen);
  }

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);
  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  if (!adaSim) {
    ROS_INFO("Start trajectory executor");
    robot.startTrajectoryExecutor();
  }

  executeTraj(robot, armSpace, armSkeleton, handSpace, handSkeleton, metaSpace, metaSkeleton);

  waitForUser("Press [ENTER] to exit. ");
  ros::shutdown();
  return 0;
}