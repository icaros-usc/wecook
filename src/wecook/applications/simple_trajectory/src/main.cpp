//
// Created by hejia on 11/7/19.
//
// joint range
// 0: -3.14, 3.14; 1: 0.82, 5.46; 2: 0.33, 5.95; 3: -3.14, 3.14; 4: -3.14, 3.14; 5: -3.14, 3.14


#include <iostream>
#include <Eigen/Dense>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
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
    "package://ada_description/robots_urdf/ada_with_camera_fixed.urdf"};
dart::common::Uri adaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera_fixed.srdf"};

static const double planningTimeout{5.};

void waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  std::cin.get();
}

Eigen::VectorXd getCurrentConfig(ada::Ada& robot)
{
  using namespace Eigen;
  IOFormat CommaInitFmt(
      StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  // TODO (Tapo): Change this back once the robot vs. arm is cleared
  auto defaultPose = robot.getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
  return defaultPose;
}

void executeTraj(
    ada::Ada& robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton)
{
  waitForUser("Plan to move hand. Press [Enter] to proceed.");

  auto satisfied = std::make_shared<aikido::constraint::Satisfied>(armSpace);



  std::shared_ptr<aikido::trajectory::Interpolated> traj;
  std::shared_ptr<aikido::statespace::GeodesicInterpolator> interpolator=std::make_shared<aikido::statespace::GeodesicInterpolator>(armSpace);
  traj = std::make_shared<aikido::trajectory::Interpolated>(armSpace, interpolator);

  Eigen::VectorXd positions = robot.getArm()->getMetaSkeleton()->getPositions();
  Eigen::VectorXd positions2(6);
  Eigen::VectorXd positions3(6);
  Eigen::VectorXd positions4(6);
  Eigen::VectorXd positions5(6);
  Eigen::VectorXd positions6(6);
  positions2 << positions;
  positions3 << positions;
  positions4 << positions;
  positions5 << positions;
  positions6 << positions;
  positions2(0) = positions2(0) + 0.5;
  positions2(2) = positions2(2) + 0.2;

  positions3(0) = 3.14124;
  positions3(2) = positions3(2) + 0.4;

  positions4(0) = 3.15;
  positions4(2) = positions3(2) + 0.4;

  positions5(0) = 3.2;
  positions5(2) = positions3(2) + 0.4;

  positions6(0) = 3.3;
  positions6(2) = positions3(2) + 0.4;

  std::cout << positions2;
  std::cout << positions3;
  std::cout << positions4;

  auto state1 = armSpace->createState();
  armSpace->convertPositionsToState(positions, state1);

  auto state2 = armSpace->createState();
  armSpace->convertPositionsToState(positions2, state2);

  auto state3 = armSpace->createState();
  armSpace->convertPositionsToState(positions3, state3);

  auto state4 = armSpace->createState();
  armSpace->convertPositionsToState(positions4, state4);

  auto state5 = armSpace->createState();
  armSpace->convertPositionsToState(positions5, state5);

  auto state6 = armSpace->createState();
  armSpace->convertPositionsToState(positions6, state6);

  auto collision = robot.getSelfCollisionConstraint(robot.getArm()->getStateSpace(), robot.getArm()->getMetaSkeleton());
  aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
  if (!collision->isSatisfied(state1, &collisionCheckOutcome))
  {
    throw std::runtime_error(
        "Robot is in collison at state1: " + collisionCheckOutcome.toString());
  }
  if (!collision->isSatisfied(state2, &collisionCheckOutcome))
  {
    throw std::runtime_error(
        "Robot is in collison at state2: " + collisionCheckOutcome.toString());
  }
  if (!collision->isSatisfied(state3, &collisionCheckOutcome))
  {
    throw std::runtime_error(
        "Robot is in collison at state3: " + collisionCheckOutcome.toString());
  }

  traj->addWaypoint(0.0, state1);
  traj->addWaypoint(1.0, state2);
  traj->addWaypoint(2.0, state3);
  traj->addWaypoint(3.0, state4);
  traj->addWaypoint(4.0, state5);
  traj->addWaypoint(5.0, state6);
  ROS_INFO_STREAM("Evaluate the found trajectory at half way");
  auto state = armSpace->createState();
  traj->evaluate(0.5, state);
  //robot.moveArmOnTrajectory(traj,ada::TrajectoryPostprocessType::PARABOLIC_RETIME);
  //auto smoothTrajectory
  //    = robot.smoothPath(armSkeleton, trajectory.get(), satisfied);
  std::cout << traj.get() << std::endl;
//  auto smoothTrajectory
//      = robot.smoothPath(robot.getArm()->getMetaSkeleton(), traj.get(), satisfied);
  //aikido::trajectory::TrajectoryPtr timedTrajectory
  //   = std::move(robot.retimePath(robot.getArm()->getMetaSkeleton(),traj.get()));
//  waitForUser("Press key to move arm to goal");

  auto smoothTrajectory
      = robot.smoothPath(armSkeleton, traj.get(), satisfied);
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));
  auto future = robot.executeTrajectory(std::move(timedTrajectory));

  future.wait();
  getCurrentConfig(robot);
}

int main(int argc, char** argv) {
  bool adaSim = true;

  // Default options for flags
  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help", "Produce help message")(
      "adasim,a", po::bool_switch(&adaSim), "Run ADA in sim");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
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
  ada::Ada robot(env, adaSim, "p1", true);
  auto robotSkeleton = robot.getMetaSkeleton();

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/simple_trajectories";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
          << execTopicName
          << "' InteractiveMarker topic in RViz.");
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      env, execTopicName, baseFrameName);

  auto space = robot.getStateSpace();
  auto collision = robot.getSelfCollisionConstraint(space, robotSkeleton);

  dart::dynamics::MetaSkeletonPtr metaSkeleton = robot.getMetaSkeleton();
  auto metaSpace = std::make_shared<MetaSkeletonStateSpace>(metaSkeleton.get());

  auto armSkeleton = robot.getArm()->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());


  if (adaSim)
  {
    Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
    home[1] = 3.14;
    home[2] = 3.14;
    armSkeleton->setPositions(home);

    auto startState
        = space->getScopedStateFromMetaSkeleton(robotSkeleton.get());

    aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
    if (!collision->isSatisfied(startState, &collisionCheckOutcome))
    {
      throw std::runtime_error(
          "Robot is in collison: " + collisionCheckOutcome.toString());
    }
  }

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);
  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  if (!adaSim)
  {
    ROS_INFO("Start trajectory executor");
    robot.startTrajectoryExecutor();
  }

  executeTraj(robot, armSpace, armSkeleton);

  waitForUser("Press [ENTER] to exit. ");
  ros::shutdown();
  return 0;
}
