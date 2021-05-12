//
// Created by hejia on 11/21/19.
//

#include <iostream>
#include <Eigen/Dense>

#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <libada/Ada.hpp>

#include "wecook/LowLevelMotionNodes/TSRMotionNode.h"
#include "wecook/utils.h"

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::robot::Robot;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

static const double planningTimeout{5.};

void waitForUser(const std::string &msg) {
  ROS_INFO(msg.c_str());
  std::cin.get();
}

Eigen::Isometry3d vectorToIsometry(std::vector<double> &poseVector) {
  double *ptr = &poseVector[0];
  Eigen::Map<Eigen::VectorXd> p(ptr, 7);

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = p.head(3);
  Eigen::Quaterniond q(p[3], p[4], p[5], p[6]);
  pose.linear() = Eigen::Matrix3d(q);
  return pose;
}

std::shared_ptr<::dart::dynamics::Skeleton> addBodyFromURDF(aikido::planner::World *world,
                                                            const std::string uri,
                                                            std::vector<double> objectPose) {
  auto transform = vectorToIsometry(objectPose);

  dart::utils::DartLoader urdfLoader;
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  const auto skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint *>(skeleton->getJoint(0))
      ->setTransform(transform);

  world->addSkeleton(skeleton);
  return skeleton;
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

int main(int argc, char **argv) {
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "test_collisions");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(
      new aikido::planner::World("test_collisions"));

  // Load ADA
  ROS_INFO("Loading ADA and ADA Img.");
  auto robotPosition = std::vector<double>{0., 0., 0.75, 1., 0., 0., 0.};
  auto robot = std::make_shared<ada::Ada>(env,
                                          true,
                                          "p0",
                                          vectorToIsometry(robotPosition),
                                          true);
  auto robotImg = std::make_shared<ada::Ada>(env,
                                             true,
                                             "p0_img",
                                             vectorToIsometry(robotPosition),
                                             false);

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/test_collisions";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
          << execTopicName
          << "' InteractiveMarker topic in RViz.");
  aikido::rviz::InteractiveMarkerViewer viewer(
      execTopicName, baseFrameName, env);

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);
  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  std::string tableURDFUri = "package://wecook_assets/data/furniture/table.urdf";
  std::vector<double> tablePose = std::vector<double>();
  tablePose.emplace_back(0.5);
  tablePose.emplace_back(0.0);
  tablePose.emplace_back(0.0);
  tablePose.emplace_back(0.707107);
  tablePose.emplace_back(0);
  tablePose.emplace_back(0);
  tablePose.emplace_back(0.707107);

  aikido::planner::WorldPtr world = robot->getWorld();
  auto skeletonTable = addBodyFromURDF(world.get(), tableURDFUri, tablePose);

  auto armSkeleton = robot->getArm()->getMetaSkeleton();
  auto armSpace = robot->getArm()->getStateSpace();
  auto handSkeleton = robot->getHand()->getMetaSkeleton();
  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
  Eigen::VectorXd positionLowerLimits = armSpace->getProperties().getPositionLowerLimits();
  std::cout << "Joint lower limits: " << positionLowerLimits << std::endl;
  Eigen::VectorXd positionUpperLimits = armSpace->getProperties().getPositionUpperLimits();
  std::cout << "Joint upper limits: " << positionUpperLimits << std::endl;

  // Setting up collision detection
  dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup =
      collisionDetector->createCollisionGroup(robot->getMetaSkeleton().get());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup =
      collisionDetector->createCollisionGroup(skeletonTable.get());
  std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
      std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace,
                                                                armSkeleton,
                                                                collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);

  // First construct a tsr
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>();
  goalTSR->mTw_e.translation() = Eigen::Vector3d(0.04, -0.3, 0.70);
  Eigen::Matrix3d rot;
  rot << 1., 0., 0., 0., -1., 0., 0., 0., -1.;
  goalTSR->mTw_e.linear() = rot;
  auto epsilon = 0.005;
  goalTSR->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
//  goalTSR->mT0_w.translation() = vectorToIsometry(tablePose).translation();
  // costruct a tsr motion node
  auto tsrMotionNode = std::make_shared<wecook::TSRMotionNode>(goalTSR,
                                                               robot->getHand()->getEndEffectorBodyNode(),
                                                               collisionFreeConstraint,
                                                               armSpace,
                                                               armSkeleton,
                                                               false);

  wecook::MotionNode::Result result{};

  tsrMotionNode->plan(robot, robotImg, &result);

  waitForUser("Press [ENTER] to exit. ");
  ros::shutdown();
  return 0;
}