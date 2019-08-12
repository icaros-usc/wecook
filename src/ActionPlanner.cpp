//
// Created by hejia on 8/2/19.
//
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/common/PseudoInverse.hpp>

#include "wecook/tsr/knife.h"
#include "wecook/tsr/food.h"
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
#include "wecook/ConnMotionPlanner.h"
#include "wecook/strutils.h"

using namespace wecook;

void ActionPlanner::plan(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
  if (action.get_verb() == "cut") {
    planCut(action, robots);
  } else if (action.get_verb() == "transfer") {
    planTransfer(action, robots);
  } else if (action.get_verb() == "stir") {
    planStir(action, robots);
  } else if (action.get_verb() == "handover") {
    planHandover(action, robots);
  } else if (action.get_verb().find('_') != std::string::npos) {
    planHolding(action, robots);
  }
}

void ActionPlanner::planHandover(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
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
  auto objectPose = getObjectPose(objectSkeleton);
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
      std::make_shared<ConnMotionPlanner>(objectSkeleton, robotMHand->getMetaSkeleton(), true, armMSpace, armMSkeleton);
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
  auto transformM = getObjectPose(robotMSkeleton);
  auto robotSSkeleton = robotS->m_ada->getMetaSkeleton();
  auto transformS = getObjectPose(robotSSkeleton);
  auto handoverPose = Eigen::Isometry3d::Identity();
  handoverPose.translation() = Eigen::Vector3d((transformM.translation()[0] + transformS.translation()[0]) / 2,
                                               (transformM.translation()[1] + transformS.translation()[1]) / 2,
                                               (transformM.translation()[2] + transformS.translation()[2]) / 2);
  handoverPose.translation()[2] = 0.9;
  handoverPose.translation()[0] = 0.10;
  // move bowl to handover point
  auto handoverTSR = getDefaultPoseTSR();
  handoverTSR.mT0_w = handoverPose * handoverTSR.mT0_w;
  handoverTSR.mBw(5, 0) = -M_PI / 8;
  handoverTSR.mBw(5, 1) = M_PI / 8;
  handoverTSR.mBw(4, 0) = -M_PI / 8;
  handoverTSR.mBw(4, 1) = M_PI / 8;
  handoverTSR.mBw(3, 0) = -M_PI / 8;
  handoverTSR.mBw(3, 1) = M_PI / 8;
  auto goalTSRM2 = std::make_shared<aikido::constraint::dart::TSR>(handoverTSR);
  // the motion should be buit with constraint
  auto constraintTSR = getDefaultPoseTSR();
  constraintTSR.mBw(0, 0) = -10000;
  constraintTSR.mBw(0, 1) = 10000;
  constraintTSR.mBw(1, 0) = -10000;
  constraintTSR.mBw(1, 1) = 10000;
  constraintTSR.mBw(2, 0) = -10000;
  constraintTSR.mBw(2, 1) = 10000;
  constraintTSR.mBw(5, 0) = -M_PI;
  constraintTSR.mBw(5, 1) = M_PI;
  constraintTSR.mBw(4, 0) = 0;
  constraintTSR.mBw(4, 1) = 0;
  constraintTSR.mBw(3, 0) = 0;
  constraintTSR.mBw(3, 1) = 0;
  auto constraintTSRPtr = std::make_shared<aikido::constraint::dart::TSR>(constraintTSR);
  auto motionM4 = std::make_shared<TSRMotionwithConstraintPlanner>(goalTSRM2,
                                                                   constraintTSRPtr,
                                                                   objectSkeleton->getBodyNode(0),
                                                                   nullptr,
                                                                   armMSpace,
                                                                   armMSkeleton,
                                                                   false);
  subMotionsM.emplace_back(motionM4);

  robotM->addSubMotions(subMotionsM);
}

//void ActionPlanner::planHolding(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
//
//}
//
//void ActionPlanner::planStir(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
//  std::vector<std::shared_ptr<MotionPlanner>> subMotions;
//  // since this action will only involve one agent
//  auto robot = robots[action.get_pid()[0]];
//  auto robotHand = robot->getHand();
//  auto robotArm = robot->getArm();
//  auto armSkeleton = robotArm->getMetaSkeleton();
//  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
//  auto handSkeleton = robotHand->getMetaSkeleton();
//  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
//
//  // create motion that moves hand to be above spoon
//  // first construct spoon tsr
//  auto world = robot->getWorld();
//  auto spoonName = action.get_tool();
//  auto spoonSkeleton = world->getSkeleton(spoonName);
//  auto spoonPose = getObjectPose(spoonSkeleton);
//  aikido::constraint::dart::TSR spoonTSR = getDefaultSpoonTSR();
//  spoonTSR.mT0_w = spoonPose * spoonTSR.mT0_w;
//  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(spoonTSR);
//  auto motion1 = std::make_shared<TSRMotionPlanner>(goalTSR, robotHand->getEndEffectorBodyNode(), nullptr, armSpace, armSkeleton, false);
//  subMotions.emplace_back(motion1);
//
//  // grab spoon
//  auto conf = Eigen::Vector2d();
//  conf << 0.75, 0.75;
//  auto motion2 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
//  subMotions.emplace_back(motion2);
//  auto motion3 = std::make_shared<ConnMotionPlanner>(spoonSkeleton, robotHand->getMetaSkeleton(), true, armSpace, armSkeleton);
//  subMotions.emplace_back(motion3);
//
//  // Move spoon to be inside pot
//
//  auto locationName = action.get_location()[0];
//  auto locationSkeleton = world->getSkeleton(locationName);
//  auto locationPose = getObjectPose(locationSkeleton);
//  aikido::constraint::dart::TSR locationTSR = getDefaultPotTSR();
//  locationTSR.mT0_w = locationPose * locationTSR.mT0_w;
//  Eigen::Matrix3d rot;
//  rot <<
//    -1, 0, 0,
//    0, 1, 0,
//    0, 0, -1;
//  Eigen::Matrix3d rot2 = Eigen::Matrix3d::Identity();
//  rot2 <<
//    -1, 0, 0,
//    0, -1, 0,
//    0, 0, 1;
//  locationTSR.mTw_e.linear() = rot * rot2;
//  locationTSR.mTw_e.translation() = Eigen::Vector3d(0, 0, 0.25);
//  auto goalTSR2 = std::make_shared<aikido::constraint::dart::TSR>(locationTSR);
//  // First setup collision detector
//  dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
//  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), spoonSkeleton.get());
//  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup = collisionDetector->createCollisionGroup(locationSkeleton.get());
//  std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint = std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
//  collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
//  auto motion4 = std::make_shared<TSRMotionPlanner>(goalTSR2, spoonSkeleton->getBodyNode(0), collisionFreeConstraint, armSpace, armSkeleton, false);
//  subMotions.emplace_back(motion4);
//
//  // Start stiring
//  for (int j = 0; j < 3; j++) {
//    dart::math::LinearJacobian jac;
//    auto bn = robotHand->getEndEffectorBodyNode();
//    jac = armSkeleton->getLinearJacobian(bn);
//    Eigen::VectorXd delta_x(3);
//    delta_x << -0.001, 0., -0.;
//    auto motion5 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
//    subMotions.emplace_back(motion5);
//
//    delta_x << +0.001, 0., 0.00;
//    auto motion6 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
//    subMotions.emplace_back(motion6);
//  }
//
//  // Put spoon back
//  aikido::constraint::dart::TSR poseTSR = getDefaultPoseTSR();
//  poseTSR.mT0_w = spoonPose * poseTSR.mT0_w;
//  auto goalTSR3 = std::make_shared<aikido::constraint::dart::TSR>(poseTSR);
//  auto motion7 = std::make_shared<TSRMotionPlanner>(goalTSR3, spoonSkeleton->getBodyNode(0), nullptr, armSpace, armSkeleton);
//  subMotions.emplace_back(motion7);
//
//  // release
//  // ungrab knife
//  conf << 0., 0.;
//  auto motion8 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
//  subMotions.emplace_back(motion8);
//  auto motion9 = std::make_shared<ConnMotionPlanner>(spoonSkeleton, robotHand->getMetaSkeleton(), false, armSpace, armSkeleton);
//  subMotions.emplace_back(motion9);
//
//  robot->addSubMotions(subMotions);
//}
//
//void ActionPlanner::planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
//  std::vector<std::shared_ptr<MotionPlanner>> subMotions;
//
//  auto robot = robots[action.get_pid()[0]];
//  auto robotHand = robot->getHand();
//  auto robotArm = robot->getArm();
//  auto armSkeleton = robotArm->getMetaSkeleton();
//  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
//  auto handSkeleton = robotHand->getMetaSkeleton();
//  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
//
//  // create move hand to be above knife
//  // first construct knife tsr
//  auto world = robot->getWorld();
//  auto knifeName = action.get_tool();
//  auto knifeSkeleton = world->getSkeleton(knifeName);
//  auto knifePose = getObjectPose(knifeSkeleton);
//  aikido::constraint::dart::TSR knifeTSR = getDefaultKnifeTSR();
//  knifeTSR.mT0_w = knifePose * knifeTSR.mT0_w;
//  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(knifeTSR);
//  auto motion1 = std::make_shared<TSRMotionPlanner>(goalTSR, robotHand->getEndEffectorBodyNode(), nullptr, armSpace, armSkeleton);
//  subMotions.emplace_back(motion1);
//
//  // grab knife
//  auto conf = Eigen::Vector2d();
//  conf << 0.75, 0.75;
//  auto motion2 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
//  subMotions.emplace_back(motion2);
//  auto motion7 = std::make_shared<ConnMotionPlanner>(knifeSkeleton, robotHand->getMetaSkeleton(), true, armSpace, armSkeleton);
//  subMotions.emplace_back(motion7);
//
//  // Move knife to be above food
//  auto foodName = action.get_ingredients()[0];
//  auto foodSkeleton = world->getSkeleton(foodName);
//  auto foodPose = getObjectPose(foodSkeleton);
//  auto foodTSR = getDefaultFoodTSR();
//  foodTSR.mT0_w = foodPose * foodTSR.mT0_w;
//  auto goalTSR2 = std::make_shared<aikido::constraint::dart::TSR>(foodTSR);
////  auto motion3 = std::make_shared<TSRMotionPlanner>(goalTSR2, knifeSkeleton->getBodyNode(0), armSpace, armSkeleton);
//  auto motion3 = std::make_shared<TSRMotionPlanner>(goalTSR2, robotHand->getEndEffectorBodyNode()->getChildBodyNode(0), nullptr, armSpace, armSkeleton);
//  subMotions.emplace_back(motion3);
//
//  // Cutting
//  for (int j = 0; j < 3; j++) {
//    dart::math::LinearJacobian jac;
//    auto bn = robotHand->getEndEffectorBodyNode();
//    jac = armSkeleton->getLinearJacobian(bn);
//    Eigen::VectorXd delta_x(3);
//    delta_x << 0., 0., -0.001;
//    auto motion4 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
//    subMotions.emplace_back(motion4);
//
//    delta_x << 0., 0., 0.001;
//    auto motion5 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
//    subMotions.emplace_back(motion5);
//  }
//
//  aikido::constraint::dart::TSR poseTSR = getDefaultPoseTSR();
//  poseTSR.mT0_w = knifePose * poseTSR.mT0_w;
//  auto goalTSR3 = std::make_shared<aikido::constraint::dart::TSR>(poseTSR);
//  auto motion6 = std::make_shared<TSRMotionPlanner>(goalTSR3, knifeSkeleton->getBodyNode(0),
//                                                    nullptr, armSpace, armSkeleton);
//  subMotions.emplace_back(motion6);
//
//  // ungrab knife
//  conf << 0., 0.;
//  auto motion8 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
//  subMotions.emplace_back(motion8);
//  auto motion9 = std::make_shared<ConnMotionPlanner>(knifeSkeleton, robotHand->getMetaSkeleton(), false, armSpace, armSkeleton);
//  subMotions.emplace_back(motion9);
//
//  robot->addSubMotions(subMotions);
//}
//
//void ActionPlanner::planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
//  std::vector<std::shared_ptr<MotionPlanner>> subMotions;
//
//  auto robot = robots[action.get_pid()[0]];
//  auto robotHand = robot->getHand();
//  auto robotArm = robot->getArm();
//  auto armSkeleton = robotArm->getMetaSkeleton();
//  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
//  auto handSkeleton = robotHand->getMetaSkeleton();
//  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
//  auto world = robot->getWorld();
//
//  if (action.get_tool() == "hand") {
//    auto foodName = action.get_ingredients()[0];
//    auto foodSkeleton = world->getSkeleton(foodName);
//    auto foodPose = getObjectPose(foodSkeleton);
//    aikido::constraint::dart::TSR foodTSR = getDefaultFoodTSR();
//    foodTSR.mT0_w.translation() = Eigen::Vector3d(0., 0., 0.08);
//    foodTSR.mT0_w = foodPose * foodTSR.mT0_w;
//    auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(foodTSR);
//    auto motion1 = std::make_shared<TSRMotionPlanner>(goalTSR, robotHand->getEndEffectorBodyNode(), nullptr, armSpace, armSkeleton);
//    subMotions.emplace_back(motion1);
//
//    // grab food
//    auto conf = Eigen::Vector2d();
//    conf << 0.75, 0.75;
//    auto motion2 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
//    subMotions.emplace_back(motion2);
//    auto motion3 = std::make_shared<ConnMotionPlanner>(foodSkeleton, robotHand->getMetaSkeleton(), true, armSpace, armSkeleton);
//    subMotions.emplace_back(motion3);
//
//    // move food a little bit up
//    // Cutting
//    for (int j = 0; j < 1; j++) {
//      dart::math::LinearJacobian jac;
//      auto bn = robotHand->getEndEffectorBodyNode();
//      jac = armSkeleton->getLinearJacobian(bn);
//      Eigen::VectorXd delta_x(3);
//
//      delta_x << 0., 0., 0.001;
//      auto motion7 = std::make_shared<DeltaMotionPlanner>(bn, delta_x, jac, 30, armSpace, armSkeleton);
//      subMotions.emplace_back(motion7);
//    }
//
//    // move food to new position
//    auto locationName = action.get_location()[1];
//    auto locationSkeleton = world->getSkeleton(locationName);
//    auto locationPose = getObjectPose(locationSkeleton);
//    auto locationTSR = getDefaultBowlTSR();
//    locationTSR.mT0_w.translation() = Eigen::Vector3d(0., 0., 0.15);
//    locationTSR.mT0_w = locationPose * locationTSR.mT0_w;
//    Eigen::Matrix3d rot;
//    rot <<
//        1, 0, 0,
//        0, -1, 0,
//        0, 0, -1;
//    Eigen::Matrix3d rot2;
//    rot2 <<
//      -1, 0, 0,
//      0, -1, 0,
//      0, 0, 1;
//    locationTSR.mTw_e.linear() = rot2 * rot;
//    auto goalTSR2 = std::make_shared<aikido::constraint::dart::TSR>(locationTSR);
//    auto motion4 = std::make_shared<TSRMotionPlanner>(goalTSR2, robotHand->getEndEffectorBodyNode(), nullptr, armSpace, armSkeleton, false);
//    subMotions.emplace_back(motion4);
//
//    // ungrab food
//    conf << 0., 0.;
//    auto motion5 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
//    subMotions.emplace_back(motion5);
//    auto motion6 = std::make_shared<ConnMotionPlanner>(foodSkeleton, robotHand->getMetaSkeleton(), false, armSpace, armSkeleton);
//    subMotions.emplace_back(motion6);
//
//    // merge food and new location
////    auto motion8 = std::make_shared<ConnMotionPlanner>(foodSkeleton, robotHand->getMetaSkeleton(), true, armSpace, armSkeleton);
////    subMotions.emplace_back(motion8);
//  } else if (action.get_tool() == action.get_location()[0]) {
//
//  } else {
//
//  }
//  robot->addSubMotions(subMotions);
//}

void ActionPlanner::planHolding(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

}

void ActionPlanner::planStir(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

}

void ActionPlanner::planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

}

void ActionPlanner::planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
}