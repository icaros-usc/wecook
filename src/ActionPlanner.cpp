//
// Created by hejia on 8/2/19.
//
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>

#include "wecook/tsr/knife.h"
#include "wecook/utils.h"
#include "wecook/ActionPlanner.h"

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
  auto robot = robots[action.get_pid()[0]];
  auto robotHand = robot->getHand();
  auto robotArm = robot->getArm();
  auto armSkeleton = robotArm->getMetaSkeleton();
  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
  // get world
  auto world = robot->getWorld();
  auto knifeName = action.get_tool();
  auto knifeSkeleton = world->getSkeleton(knifeName);
  // go to grab knife
  auto knifePose = getObjectPose(knifeSkeleton);

  aikido::constraint::dart::TSR knifeTSR = getDefaultKnifeTSR();
  knifeTSR.mT0_w = knifePose * knifeTSR.mT0_w;
//  knifeTSR.mTw_e = Eigen::Isometry3d::Identity();
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(knifeTSR);

  auto ik = dart::dynamics::InverseKinematics::create(robotHand->getEndEffectorBodyNode());
  ik->setDofs(armSkeleton->getDofs());

  auto rng = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
  aikido::constraint::dart::InverseKinematicsSampleable ikSampleable(armSpace,
                                                                     armSkeleton,
                                                                     goalTSR,
                                                                     aikido::constraint::dart::createSampleableBounds(armSpace, std::move(rng)),
                                                                     ik,
                                                                     10);
  auto generator = ikSampleable.createSampleGenerator();
  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations;
  auto goalState = armSpace->createState();
  static const std::size_t maxSnapSamples{2};
  std::size_t snapSamples = 0;
  while (snapSamples < maxSnapSamples && generator->canSample()) {
    std::lock_guard<std::mutex> lock(armSkeleton->getBodyNode(0)->getSkeleton()->getMutex());
    bool sampled = generator->sample(goalState);
    ++snapSamples;

    if (!sampled) {
      continue;
    }
    configurations.emplace_back(goalState.clone());
  }

  if (!configurations.empty()) {
    ROS_INFO("Found a valid goal state!");
    ROS_INFO("Start planning a path to the goal configuration!");
    auto trajectory = robot->m_ada->planToConfiguration(armSpace,
                                                        armSkeleton,
                                                        configurations[0],
                                                        nullptr,
                                                        10);
    if (trajectory) {
      ROS_INFO("Found the trajectory!");
      aikido::trajectory::TrajectoryPtr timedTrajectory
          = robot->m_ada->smoothPath(armSkeleton, trajectory.get(), std::make_shared<aikido::constraint::Satisfied>(armSpace));
      auto future = robot->executeTrajectory(timedTrajectory);
      future.wait();
    }
  } else {
    ROS_INFO("Didn't find a valid goal state!");
  }

  // grab knife

}

void ActionPlanner::planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

}