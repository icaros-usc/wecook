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
#include "wecook/utils.h"
#include "wecook/ActionPlanner.h"
#include "wecook/TSRMotionPlanner.h"
#include "wecook/ConfMotionPlanner.h"
#include "wecook/DeltaMotionPlanner.h"
#include "wecook/ConnMotionPlanner.h"

using namespace wecook;

void ActionPlanner::plan(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
  if (action.get_verb() == "cut") {
    planCut(action, robots);
  } else if (action.get_verb() == "transfer") {
    planTransfer(action, robots);
  } else if (action.get_verb() == "stir") {
    planStir(action, robots);
  }
}

void ActionPlanner::planStir(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

}

void ActionPlanner::planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
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
  auto knifePose = getObjectPose(knifeSkeleton);
  aikido::constraint::dart::TSR knifeTSR = getDefaultKnifeTSR();
  knifeTSR.mT0_w = knifePose * knifeTSR.mT0_w;
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(knifeTSR);
  auto motion1 = std::make_shared<TSRMotionPlanner>(goalTSR, robotHand->getEndEffectorBodyNode(), armSpace, armSkeleton);
  subMotions.emplace_back(motion1);

  // grab knife
  auto conf = Eigen::Vector2d();
  conf << 0.75, 0.75;
  auto motion2 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
  subMotions.emplace_back(motion2);
  auto motion7 = std::make_shared<ConnMotionPlanner>(knifeSkeleton, true, armSpace, armSkeleton);
  subMotions.emplace_back(motion7);

  // Move knife to be above food
  auto foodName = action.get_ingredients()[0];
  auto foodSkeleton = world->getSkeleton(foodName);
  auto foodPose = getObjectPose(foodSkeleton);
  auto foodTSR = getDefaultFoodTSR();
  foodTSR.mT0_w = foodPose * foodTSR.mT0_w;
  auto goalTSR2 = std::make_shared<aikido::constraint::dart::TSR>(foodTSR);
//  auto motion3 = std::make_shared<TSRMotionPlanner>(goalTSR2, knifeSkeleton->getBodyNode(0), armSpace, armSkeleton);
  auto motion3 = std::make_shared<TSRMotionPlanner>(goalTSR2, robotHand->getEndEffectorBodyNode()->getChildBodyNode(0), armSpace, armSkeleton);
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
//  auto motion6 = std::make_shared<TSRMotionPlanner>(goalTSR3, knifeSkeleton->getBodyNode(0), armSpace, armSkeleton);
  auto motion6 = std::make_shared<TSRMotionPlanner>(goalTSR3, robotHand->getEndEffectorBodyNode()->getChildBodyNode(0), armSpace, armSkeleton);
  subMotions.emplace_back(motion6);

  // ungrab knife
  conf << 0., 0.;
  auto motion8 = std::make_shared<ConfMotionPlanner>(conf, handSpace, handSkeleton);
  subMotions.emplace_back(motion8);
  auto motion9 = std::make_shared<ConnMotionPlanner>(knifeSkeleton, false, armSpace, armSkeleton);
  subMotions.emplace_back(motion9);

  robot->addSubMotions(subMotions);
}

//void ActionPlanner::planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
//  auto robot = robots[action.get_pid()[0]];
//  auto robotHand = robot->getHand();
//  auto robotArm = robot->getArm();
//  auto armSkeleton = robotArm->getMetaSkeleton();
//  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
//  // get world
//  auto world = robot->getWorld();
//  auto knifeName = action.get_tool();
//  auto knifeSkeleton = world->getSkeleton(knifeName);
//  // go to grab knife
//  auto knifePose = getObjectPose(knifeSkeleton);
//
//  aikido::constraint::dart::TSR knifeTSR = getDefaultKnifeTSR();
//  knifeTSR.mT0_w = knifePose * knifeTSR.mT0_w;
////  knifeTSR.mTw_e = Eigen::Isometry3d::Identity();
//  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(knifeTSR);
//
//  auto ik = dart::dynamics::InverseKinematics::create(robotHand->getEndEffectorBodyNode());
//  ik->setDofs(armSkeleton->getDofs());
//
//  auto rng = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
//  aikido::constraint::dart::InverseKinematicsSampleable ikSampleable(armSpace,
//                                                                     armSkeleton,
//                                                                     goalTSR,
//                                                                     aikido::constraint::dart::createSampleableBounds(armSpace, std::move(rng)),
//                                                                     ik,
//                                                                     10);
//  auto generator = ikSampleable.createSampleGenerator();
//  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations;
//  auto goalState = armSpace->createState();
//  static const std::size_t maxSnapSamples{2};
//  std::size_t snapSamples = 0;
//  while (snapSamples < maxSnapSamples && generator->canSample()) {
//    std::lock_guard<std::mutex> lock(armSkeleton->getBodyNode(0)->getSkeleton()->getMutex());
//    bool sampled = generator->sample(goalState);
//    ++snapSamples;
//
//    if (!sampled) {
//      continue;
//    }
//    configurations.emplace_back(goalState.clone());
//  }
//
//  if (!configurations.empty()) {
//    ROS_INFO("Found a valid goal state!");
//    ROS_INFO("Start planning a path to the goal configuration!");
//    auto trajectory = robot->m_ada->planToConfiguration(armSpace,
//                                                        armSkeleton,
//                                                        configurations[0],
//                                                        nullptr,
//                                                        10);
//    if (trajectory) {
//      ROS_INFO("Found the trajectory!");
//      aikido::trajectory::TrajectoryPtr timedTrajectory
//          = robot->m_ada->smoothPath(armSkeleton, trajectory.get(), std::make_shared<aikido::constraint::Satisfied>(armSpace));
//      auto future = robot->executeTrajectory(timedTrajectory);
//      future.wait();
//    }
//  } else {
//    ROS_INFO("Didn't find a valid goal state!");
//  }
//
//  // grab knife
//  robot->closeHand();
//  robotHand->grab(knifeSkeleton);
//
//  // move to above food item
//  // assume all ingredients have been
//  // put together
//  auto foodName = action.get_ingredients()[0];
//  auto foodSkeleton = world->getSkeleton(foodName);
//  auto foodPose = getObjectPose(foodSkeleton);
//  auto foodTSR = getDefaultFoodTSR();
//  foodTSR.mT0_w = foodPose * foodTSR.mT0_w;
//  auto goalTSR2 = std::make_shared<aikido::constraint::dart::TSR>(foodTSR);
//
//
//
//  auto ik2 = dart::dynamics::InverseKinematics::create(robotHand->getEndEffectorBodyNode()->getChildBodyNode(0));
//
//  ik2->setDofs(armSkeleton->getDofs());
//
//  std::cout << rng.get() << std::endl;
//  auto rng2 = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
//  aikido::constraint::dart::InverseKinematicsSampleable ikSampleable2(armSpace,
//                                                                      armSkeleton,
//                                                                      goalTSR2,
//                                                                      aikido::constraint::dart::createSampleableBounds(armSpace, std::move(rng2)),
//                                                                      ik2,
//                                                                      10);
//  ROS_INFO("This?");
//  auto generator2 = ikSampleable2.createSampleGenerator();
//  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations2;
//
//  auto goalState2 = armSpace->createState();
//  static const std::size_t maxSnapSamples2{2};
//  std::size_t snapSamples2 = 0;
//  while (snapSamples2 < maxSnapSamples2 && generator2->canSample()) {
//    std::lock_guard<std::mutex> lock(armSkeleton->getBodyNode(0)->getSkeleton()->getMutex());
//    bool sampled = generator2->sample(goalState2);
//    ++snapSamples2;
//
//    if (!sampled) {
//      continue;
//    }
//    configurations2.emplace_back(goalState2.clone());
//  }
//
//  if (!configurations2.empty()) {
//    ROS_INFO("Found a valid goal state!");
//    ROS_INFO("Start planning a path to the goal configuration!");
//    auto trajectory = robot->m_ada->planToConfiguration(armSpace,
//                                                        armSkeleton,
//                                                        configurations2[0],
//                                                        nullptr,
//                                                        10);
//    if (trajectory) {
//      ROS_INFO("Found the trajectory!");
//      aikido::trajectory::TrajectoryPtr timedTrajectory
//          = robot->m_ada->smoothPath(armSkeleton, trajectory.get(), std::make_shared<aikido::constraint::Satisfied>(armSpace));
//      auto future = robot->executeTrajectory(timedTrajectory);
//      future.wait();
//    }
//  } else {
//    ROS_INFO("Didn't find a valid goal state!");
//  }
//
//  using Jacobian = dart::math::LinearJacobian;
//  Jacobian jac;
//  Eigen::VectorXd delta_x(3);
//  Eigen::VectorXd delta_q(6);
//
//  for (int j = 0; j < 3; j++) {
//    // move knife downwards
//    for(int i = 0; i < 30; i++) {
//      jac = armSkeleton->getLinearJacobian(robotHand->getEndEffectorBodyNode());
//      delta_x << 0.0,0,-0.001;
//      delta_q = aikido::common::pseudoinverse(jac)*delta_x;
//      Eigen::VectorXd currPos = armSkeleton->getPositions();
//      ros::Duration(0.1).sleep();
////    std::cout <<"delta_q is: " << delta_q << std::endl;
////    std::cout <<"currPos is: " << currPos << std::endl;
//      Eigen::VectorXd new_pos = currPos + delta_q;
//      armSkeleton->setPositions(new_pos);
//    }
//    // raise knife
//    for(int i = 0; i < 30; i++) {
//      jac = armSkeleton->getLinearJacobian(robotHand->getEndEffectorBodyNode());
//      delta_x << 0.0,0,0.001;
//      delta_q = aikido::common::pseudoinverse(jac)*delta_x;
//      Eigen::VectorXd currPos = armSkeleton->getPositions();
//      ros::Duration(0.1).sleep();
////    std::cout <<"delta_q is: " << delta_q << std::endl;
////    std::cout <<"currPos is: " << currPos << std::endl;
//      Eigen::VectorXd new_pos = currPos + delta_q;
//      armSkeleton->setPositions(new_pos);
//    }
//  }
//
//  // put knife back
//  aikido::constraint::dart::TSR poseTSR = getDefaultPoseTSR();
//  poseTSR.mT0_w = knifePose * poseTSR.mT0_w;
//  auto goalTSR3 = std::make_shared<aikido::constraint::dart::TSR>(poseTSR);
//  auto ik3 = dart::dynamics::InverseKinematics::create(robotHand->getEndEffectorBodyNode());
//  ik3->setDofs(armSkeleton->getDofs());
//  auto rng3 = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
//  aikido::constraint::dart::InverseKinematicsSampleable ikSampleable3(armSpace,
//                                                                      armSkeleton,
//                                                                      goalTSR3,
//                                                                      aikido::constraint::dart::createSampleableBounds(armSpace, std::move(rng3)),
//                                                                      ik3,
//                                                                     10);
//  auto generator3 = ikSampleable3.createSampleGenerator();
//  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations3;
//  auto goalState3 = armSpace->createState();
//  static const std::size_t maxSnapSamples3{2};
//  std::size_t snapSamples3 = 0;
//  while (snapSamples3 < maxSnapSamples3 && generator3->canSample()) {
//    std::lock_guard<std::mutex> lock(armSkeleton->getBodyNode(0)->getSkeleton()->getMutex());
//    bool sampled = generator3->sample(goalState3);
//    ++snapSamples3;
//
//    if (!sampled) {
//      continue;
//    }
//    configurations3.emplace_back(goalState3.clone());
//  }
//
//  if (!configurations3.empty()) {
//    ROS_INFO("Found a valid goal state!");
//    ROS_INFO("Start planning a path to the goal configuration!");
//    auto trajectory = robot->m_ada->planToConfiguration(armSpace,
//                                                        armSkeleton,
//                                                        configurations3[0],
//                                                        nullptr,
//                                                        10);
//    if (trajectory) {
//      ROS_INFO("Found the trajectory!");
//      aikido::trajectory::TrajectoryPtr timedTrajectory
//          = robot->m_ada->smoothPath(armSkeleton, trajectory.get(), std::make_shared<aikido::constraint::Satisfied>(armSpace));
//      auto future = robot->executeTrajectory(timedTrajectory);
//      future.wait();
//    }
//  } else {
//    ROS_INFO("Didn't find a valid goal state!");
//  }
//
//  // drop knife
//  robot->openHand();
//  robotHand->ungrab();
//}

void ActionPlanner::planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

}