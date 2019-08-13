//
// Created by hejia on 8/2/19.
//
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/common/PseudoInverse.hpp>

#include "wecook/tsr/knife.h"
#include "wecook/tsr/pose.h"
#include "wecook/tsr/spoon.h"
#include "wecook/tsr/pot.h"
#include "wecook/tsr/bowl.h"
#include "wecook/utils.h"
#include "wecook/ActionPlanner.h"
#include "wecook/TSRMotionPlanner.h"
#include "wecook/TSRMotionwithConstraintPlanner.h"
#include "wecook/ConfMotionPlanner.h"
#include "wecook/DeltaMotionPlanner.h"
#include "wecook/GrabMotionPlanner.h"
#include "wecook/ConnMotionPlanner.h"
#include "wecook/strutils.h"

using namespace wecook;

void ActionPlanner::plan(Action &action,
                         std::map<std::string, std::shared_ptr<Robot>> &robots,
                         std::shared_ptr<ContainingMap> &containingMap) {
  if (action.get_verb() == "cut") {
    planCut(action, robots, containingMap);
  } else if (action.get_verb() == "transfer") {
    planTransfer(action, robots, containingMap);
  } else if (action.get_verb() == "stir") {
    planStir(action, robots, containingMap);
  } else if (action.get_verb() == "handover") {
    planHandover(action, robots, containingMap);
  } else if (action.get_verb().find('_') != std::string::npos) {
    planHolding(action, robots, containingMap);
  }
}

void ActionPlanner::planHandover(Action &action,
                                 std::map<std::string, std::shared_ptr<Robot>> &robots,
                                 std::shared_ptr<ContainingMap> &containingMap) {
  std::vector<std::shared_ptr<MotionPlanner>> subMotionsM;
  // this action will invlove two agents, robot master and robot slave
  // robot master is the agent initiating the interaction
  auto robotM = robots[action.get_pid()[0]];
  auto robotMHand = robotM->getHand();
  auto robotMArm = robotM->getArm();
  auto armMSkeleton = robotMArm->getMetaSkeleton();
  auto armMSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armMSkeleton.get());
  auto handMSkeleton = robotMHand->getMetaSkeleton();
  auto handMSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handMSkeleton.get());

  // first plan robot master to grasp the object
  auto worldM = robotM->getWorld();
  auto objectName = action.get_ingredients()[0];
  auto objectSkeleton = worldM->getSkeleton(objectName);
  auto objectPose = getObjectPose(objectSkeleton, containingMap);
  aikido::constraint::dart::TSR objectTSR = getDefaultBowlTSR();
  Eigen::Matrix3d rot;
  rot <<
      -1, 0, 0,
      0, 1, 0,
      0, 0, -1;
  Eigen::Matrix3d rot2;
  rot2 <<
       0.7071072, 0.7071063, 0.0000000,
      -0.7071063, 0.7071072, 0.0000000,
      0.0000000, 0.0000000, 1.0000000;
  objectTSR.mTw_e.linear() = rot2 * rot;
  objectTSR.mTw_e.translation() = Eigen::Vector3d(-0.05, -0.05, 0.08);
  objectTSR.mBw = Eigen::Matrix<double, 6, 2>::Zero();
  objectTSR.mBw(2, 0) = -0.01;
  objectTSR.mBw(2, 1) = 0.01;
  objectTSR.mBw(0, 0) = -0.01;
  objectTSR.mBw(0, 1) = 0.01;
  objectTSR.mBw(5, 0) = -M_PI;
  objectTSR.mBw(5, 1) = M_PI;
  objectTSR.mBw(4, 0) = -M_PI / 8;
  objectTSR.mBw(4, 1) = M_PI / 8;
  objectTSR.mBw(3, 0) = -M_PI / 8;
  objectTSR.mBw(3, 1) = M_PI / 8;
  objectTSR.mT0_w = objectPose * objectTSR.mT0_w;
  auto goalTSRM1 = std::make_shared<aikido::constraint::dart::TSR>(objectTSR);
  auto motionM1 = std::make_shared<TSRMotionPlanner>(goalTSRM1,
                                                     robotMHand->getEndEffectorBodyNode(),
                                                     nullptr,
                                                     armMSpace,
                                                     armMSkeleton,
                                                     false);
  subMotionsM.emplace_back(motionM1);

  // grab the bowl
  auto conf = Eigen::Vector2d();
  conf << 0.75, 0.75;
  auto motionM2 = std::make_shared<ConfMotionPlanner>(conf, handMSpace, handMSkeleton);
  subMotionsM.emplace_back(motionM2);
  auto motionM3 =
      std::make_shared<GrabMotionPlanner>(objectSkeleton, true, armMSpace, armMSkeleton);
  subMotionsM.emplace_back(motionM3);

  // move the bowl to the centor of two agents
  // first get robot slave
  auto robotS = robots[action.get_pid()[1]];
  auto robotSHand = robotS->getHand();
  auto robotSArm = robotS->getArm();
  auto armSSkeleton = robotSArm->getMetaSkeleton();
  auto armSSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSSkeleton.get());
  auto handSSkeleton = robotSHand->getMetaSkeleton();
  auto handSSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSSkeleton.get());
  // now get positions of two agents
  auto robotMSkeleton = robotM->m_ada->getMetaSkeleton();
  auto transformM = getObjectPose(robotMSkeleton, containingMap);
  auto robotSSkeleton = robotS->m_ada->getMetaSkeleton();
  auto transformS = getObjectPose(robotSSkeleton, containingMap);
  auto handoverPose = Eigen::Isometry3d::Identity();
  handoverPose.translation() = Eigen::Vector3d((transformM.translation()[0] + transformS.translation()[0]) / 2,
                                               (transformM.translation()[1] + transformS.translation()[1]) / 2,
                                               (transformM.translation()[2] + transformS.translation()[2]) / 2);
  handoverPose.translation()[2] = 0.8;
  handoverPose.translation()[0] = 0.3;
  // move bowl to handover point
  auto handoverTSR = getDefaultPoseTSR();
  Eigen::Vector3d direction = Eigen::Vector3d(handoverPose.translation()[0] - objectPose.translation()[0],
                                              handoverPose.translation()[1] - objectPose.translation()[1],
                                              handoverPose.translation()[2] - objectPose.translation()[2]);
  double distance = direction.norm();
  std::cout << "Distance: " << direction.norm() << std::endl;
  double epsilon = 0.01;
  auto constraintTSR = std::make_shared<aikido::constraint::dart::TSR>();
  constraintTSR->mT0_w = dart::math::computeTransform(direction / direction.norm(),
                                                      objectPose.translation(),
                                                      dart::math::AxisType::AXIS_Z);
  constraintTSR->mTw_e = constraintTSR->mT0_w.inverse() * objectPose;
  constraintTSR->mBw << -epsilon, epsilon, -epsilon, epsilon,
      std::min(0., distance), std::max(0., distance), -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI
      / 16;
  auto goalTSRM2 = std::make_shared<aikido::constraint::dart::TSR>();
  auto offset = Eigen::Isometry3d::Identity();
  offset(2, 3) = distance;
  goalTSRM2->mT0_w = constraintTSR->mT0_w * offset;
  goalTSRM2->mTw_e = constraintTSR->mTw_e;
  goalTSRM2->mBw << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon,
      -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16;

  auto motionM4 = std::make_shared<TSRMotionwithConstraintPlanner>(goalTSRM2,
                                                                   constraintTSR,
                                                                   objectSkeleton->getBodyNode(0),
                                                                   nullptr,
                                                                   armMSpace,
                                                                   armMSkeleton,
                                                                   false);
  subMotionsM.emplace_back(motionM4);

  robotM->addSubMotions(subMotionsM);
}

void ActionPlanner::planHolding(Action &action,
                                std::map<std::string, std::shared_ptr<Robot>> &robots,
                                std::shared_ptr<ContainingMap> &containingMap) {

}

void ActionPlanner::planStir(Action &action,
                             std::map<std::string, std::shared_ptr<Robot>> &robots,
                             std::shared_ptr<ContainingMap> &containingMap) {
  std::vector<std::shared_ptr<MotionPlanner>> subMotions;
  // since this action will only involve one agent
  auto robot = robots[action.get_pid()[0]];
  auto robotHand = robot->getHand();
  auto robotArm = robot->getArm();
  auto armSkeleton = robotArm->getMetaSkeleton();
  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
  auto handSkeleton = robotHand->getMetaSkeleton();
  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());

  // create motion that moves hand to be above spoon
  // first construct spoon tsr
  auto world = robot->getWorld();
  auto spoonName = action.get_tool();
  auto spoonSkeleton = world->getSkeleton(spoonName);
  auto spoonPose = getObjectPose(spoonSkeleton, containingMap);
  aikido::constraint::dart::TSR spoonTSR = getDefaultSpoonTSR();
  spoonTSR.mT0_w = spoonPose * spoonTSR.mT0_w;
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(spoonTSR);
  auto motion1 = std::make_shared<TSRMotionPlanner>(goalTSR,
                                                    robotHand->getEndEffectorBodyNode(),
                                                    nullptr,
                                                    armSpace,
                                                    armSkeleton,
                                                    false);
  subMotions.emplace_back(motion1);

  // grab spoon
  auto conf = Eigen::Vector2d();
  conf << 0.75, 0.75;
  auto motion2 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
  subMotions.emplace_back(motion2);

  auto motion3 =
      std::make_shared<GrabMotionPlanner>(spoonSkeleton, true, armSpace, armSkeleton);
  subMotions.emplace_back(motion3);

  // Move spoon to be inside pot
  auto locationName = action.get_location()[0];
  auto locationSkeleton = world->getSkeleton(locationName);
  auto locationPose = getObjectPose(locationSkeleton, containingMap);
  aikido::constraint::dart::TSR locationTSR = getDefaultPotTSR();
  locationTSR.mT0_w = locationPose * locationTSR.mT0_w;
  Eigen::Matrix3d rot;
  rot <<
      -1, 0, 0,
      0, 1, 0,
      0, 0, -1;
  Eigen::Matrix3d rot2 = Eigen::Matrix3d::Identity();
  rot2 <<
       -1, 0, 0,
      0, -1, 0,
      0, 0, 1;
  locationTSR.mTw_e.linear() = rot * rot2;
  locationTSR.mTw_e.translation() = Eigen::Vector3d(0, 0, 0.25);
  auto goalTSR2 = std::make_shared<aikido::constraint::dart::TSR>(locationTSR);
  // First setup collision detector
  dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup>
      armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), spoonSkeleton.get());
  std::shared_ptr<dart::collision::CollisionGroup>
      envCollisionGroup = collisionDetector->createCollisionGroup(locationSkeleton->getBodyNode(0));
  std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
      std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
  auto motion4 = std::make_shared<TSRMotionPlanner>(goalTSR2,
                                                    spoonSkeleton->getBodyNode(0),
                                                    collisionFreeConstraint,
                                                    armSpace,
                                                    armSkeleton,
                                                    false);
  subMotions.emplace_back(motion4);

  // Start stiring
  for (int j = 0; j < 3; j++) {
    dart::math::LinearJacobian jac;
    auto bn = robotHand->getEndEffectorBodyNode();
    jac = armSkeleton->getLinearJacobian(bn);
    Eigen::VectorXd delta_x(3);
    delta_x << -0.001, 0., -0.;
    auto motion5 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
    subMotions.emplace_back(motion5);

    delta_x << +0.001, 0., 0.00;
    auto motion6 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
    subMotions.emplace_back(motion6);
  }

  // Put spoon back
  aikido::constraint::dart::TSR poseTSR = getDefaultPoseTSR();
  poseTSR.mT0_w = spoonPose * poseTSR.mT0_w;
  auto goalTSR3 = std::make_shared<aikido::constraint::dart::TSR>(poseTSR);
  auto motion7 =
      std::make_shared<TSRMotionPlanner>(goalTSR3, spoonSkeleton->getBodyNode(0), nullptr, armSpace, armSkeleton);
  subMotions.emplace_back(motion7);

  // release
  // ungrab spoon
  conf << 0., 0.;
  auto motion8 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
  subMotions.emplace_back(motion8);

  auto motion9 =
      std::make_shared<GrabMotionPlanner>(spoonSkeleton, false, armSpace, armSkeleton);
  subMotions.emplace_back(motion9);

  robot->addSubMotions(subMotions);
}

void ActionPlanner::planCut(Action &action,
                            std::map<std::string, std::shared_ptr<Robot>> &robots,
                            std::shared_ptr<ContainingMap> &containingMap) {
  std::vector<std::shared_ptr<MotionPlanner>> subMotions;

  auto robot = robots[action.get_pid()[0]];
  auto robotHand = robot->getHand();
  auto robotArm = robot->getArm();
  auto armSkeleton = robotArm->getMetaSkeleton();
  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
  auto handSkeleton = robotHand->getMetaSkeleton();
  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());

  // create move hand to be above knife
  // first construct knife tsr
  auto world = robot->getWorld();
  auto knifeName = action.get_tool();
  auto knifeSkeleton = world->getSkeleton(knifeName);
  auto knifePose = getObjectPose(knifeSkeleton, containingMap);
  aikido::constraint::dart::TSR knifeTSR = getDefaultKnifeTSR();
  knifeTSR.mT0_w = knifePose * knifeTSR.mT0_w;
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(knifeTSR);
  auto motion1 =
      std::make_shared<TSRMotionPlanner>(goalTSR, robotHand->getEndEffectorBodyNode(), nullptr, armSpace, armSkeleton);
  subMotions.emplace_back(motion1);

  // grab knife
  auto conf = Eigen::Vector2d();
  conf << 0.75, 0.75;
  auto motion2 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
  subMotions.emplace_back(motion2);
  auto motion7 =
      std::make_shared<GrabMotionPlanner>(knifeSkeleton, true, armSpace, armSkeleton);
  subMotions.emplace_back(motion7);

  // Move knife to be above food
  auto foodName = action.get_ingredients()[0];
  auto foodSkeleton = world->getSkeleton(foodName);
  auto foodPose = getObjectPose(foodSkeleton, containingMap);
  auto foodTSR = std::make_shared<aikido::constraint::dart::TSR>();
  foodTSR->mT0_w = foodPose * foodTSR->mT0_w;
  foodTSR->mTw_e.linear() = knifePose.linear();
  double knifeHandleLength = 0.08;
  double knifeBladeLength = 0.08;
  double foodHeight = 0.045;
  foodTSR->mTw_e.translation() = Eigen::Vector3d(-(knifeHandleLength + knifeBladeLength / 2), 0, 0.045);
  auto epsilon = 0.02;
  foodTSR->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
//  auto motion3 = std::make_shared<TSRMotionPlanner>(goalTSR2, knifeSkeleton->getBodyNode(0), armSpace, armSkeleton);
  auto motion3 = std::make_shared<TSRMotionPlanner>(foodTSR,
                                                    knifeSkeleton->getBodyNode(0),
                                                    nullptr,
                                                    armSpace,
                                                    armSkeleton);
  subMotions.emplace_back(motion3);

  // Cutting
  for (int j = 0; j < 3; j++) {
    dart::math::LinearJacobian jac;
    auto bn = robotHand->getEndEffectorBodyNode();
    jac = armSkeleton->getLinearJacobian(bn);
    Eigen::VectorXd delta_x(3);
    delta_x << 0., 0., -0.001;
    auto motion4 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
    subMotions.emplace_back(motion4);

    delta_x << 0., 0., 0.001;
    auto motion5 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
    subMotions.emplace_back(motion5);
  }

  aikido::constraint::dart::TSR poseTSR = getDefaultPoseTSR();
  poseTSR.mT0_w = knifePose * poseTSR.mT0_w;
  auto goalTSR3 = std::make_shared<aikido::constraint::dart::TSR>(poseTSR);
  auto motion6 = std::make_shared<TSRMotionPlanner>(goalTSR3, knifeSkeleton->getBodyNode(0),
                                                    nullptr, armSpace, armSkeleton);
  subMotions.emplace_back(motion6);

  // ungrab knife
  conf << 0., 0.;
  auto motion8 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
  subMotions.emplace_back(motion8);

  auto motion9 =
      std::make_shared<GrabMotionPlanner>(knifeSkeleton, false, armSpace, armSkeleton);
  subMotions.emplace_back(motion9);

  robot->addSubMotions(subMotions);
}

void ActionPlanner::planTransfer(Action &action,
                                 std::map<std::string, std::shared_ptr<Robot>> &robots,
                                 std::shared_ptr<ContainingMap> &containingMap) {
  std::vector<std::shared_ptr<MotionPlanner>> subMotions;

  auto robot = robots[action.get_pid()[0]];
  auto robotHand = robot->getHand();
  auto robotArm = robot->getArm();
  auto armSkeleton = robotArm->getMetaSkeleton();
  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
  auto handSkeleton = robotHand->getMetaSkeleton();
  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
  auto world = robot->getWorld();

  if (action.get_tool() == "hand") {
    auto foodName = action.get_ingredients()[0];
    auto foodSkeleton = world->getSkeleton(foodName);
    auto foodPose = getObjectPose(foodSkeleton, containingMap);
    auto foodTSR = std::make_shared<aikido::constraint::dart::TSR>();
    foodTSR->mT0_w = foodPose * foodTSR->mT0_w;
    Eigen::Matrix3d rot;
    rot <<
        -1, 0, 0,
        0, 1, 0,
        0, 0, -1;
    foodTSR->mTw_e.linear() = rot;
    foodTSR->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.10);
    auto epsilon = 0.02;
    foodTSR->mBw
        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI, M_PI;
    auto motion1 = std::make_shared<TSRMotionPlanner>(foodTSR,
                                                      robotHand->getEndEffectorBodyNode(),
                                                      nullptr,
                                                      armSpace,
                                                      armSkeleton,
                                                      false);
    subMotions.emplace_back(motion1);

    // unconnect food with old container
    auto oldLocationName = action.get_location()[0];
    auto oldLocationSkeleton = world->getSkeleton(oldLocationName);
    auto motion8 = std::make_shared<ConnMotionPlanner>(foodSkeleton,
                                                       oldLocationSkeleton,
                                                       oldLocationName,
                                                       foodName,
                                                       containingMap,
                                                       false,
                                                       armSpace,
                                                       armSkeleton);
    subMotions.emplace_back(motion8);

    // grab food
    auto conf = Eigen::Vector2d();
    conf << 0.75, 0.75;
    auto motion2 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
    subMotions.emplace_back(motion2);
    auto motion3 =
        std::make_shared<GrabMotionPlanner>(foodSkeleton, true, armSpace, armSkeleton);
    subMotions.emplace_back(motion3);

    // move food a little bit up
    for (int j = 0; j < 1; j++) {
      dart::math::LinearJacobian jac;
      auto bn = robotHand->getEndEffectorBodyNode();
      jac = armSkeleton->getLinearJacobian(bn);
      Eigen::VectorXd delta_x(3);

      delta_x << 0., 0., 0.001;
      auto motion7 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
      subMotions.emplace_back(motion7);
    }

    // move food to new position
    auto locationName = action.get_location()[1];
    auto locationSkeleton = world->getSkeleton(locationName);
    auto locationPose = getObjectPose(locationSkeleton, containingMap);
    auto locationTSR = std::make_shared<aikido::constraint::dart::TSR>();
    locationTSR->mT0_w = locationPose * locationTSR->mT0_w;
    locationTSR->mTw_e.linear() = rot;
    locationTSR->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.15);
    locationTSR->mBw
        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI / 8, M_PI / 8, -M_PI / 8, M_PI
        / 8, -M_PI, M_PI;
    auto motion4 = std::make_shared<TSRMotionPlanner>(locationTSR,
                                                      robotHand->getEndEffectorBodyNode(),
                                                      nullptr,
                                                      armSpace,
                                                      armSkeleton,
                                                      false);
    subMotions.emplace_back(motion4);

    // ungrab food
    conf << 0., 0.;
    auto motion5 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
    subMotions.emplace_back(motion5);
    auto motion6 =
        std::make_shared<GrabMotionPlanner>(foodSkeleton, false, armSpace, armSkeleton);
    subMotions.emplace_back(motion6);

    // connect food and new location
    auto motion9 = std::make_shared<ConnMotionPlanner>(foodSkeleton,
                                                       locationSkeleton,
                                                       locationName,
                                                       foodName,
                                                       containingMap,
                                                       true,
                                                       armSpace,
                                                       armSkeleton);
    subMotions.emplace_back(motion9);
  } else if (action.get_tool() == action.get_location()[0]) {

  } else {

  }
  robot->addSubMotions(subMotions);
}