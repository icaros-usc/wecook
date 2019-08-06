//
// Created by hejia on 7/30/19.
//

#include <iostream>
#include <aikido/common/PseudoInverse.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>

#include "wecook/Robot.h"

using namespace wecook;

int Robot::execute(Action &action) {
  m_action.emplace_back(action);
}

void Robot::stop() {
  m_isEnd = true;
  m_thread.join();
}

void Robot::run() {
  while (!m_isEnd) {
    while (!m_subMotions.empty()) {
      m_isFree = false;
      for (auto &motion : m_subMotions) {
        motion->plan(m_ada);
      }

      m_subMotions.clear();
    }
    m_isFree = true;
  }
}

aikido::trajectory::TrajectoryPtr Robot::planToTSR(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space,
                                                   const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
                                                   const dart::dynamics::BodyNodePtr &bn,
                                                   const aikido::constraint::dart::TSRPtr &tsr,
                                                   const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                                                   double timelimit,
                                                   size_t maxNumTrails) {
  m_ada->planToTSR(space,
                   metaSkeleton,
                   bn,
                   tsr,
                   collisionFree,
                   timelimit,
                   maxNumTrails);
}

void Robot::closeHand() {
  auto handSkeleton = m_ada->getHand()->getMetaSkeleton();
  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());

  auto goalState = handSpace->createState();
  auto initialPose = handSkeleton->getPositions();
  std::cout << "Initial pose: " << initialPose << std::endl;
  Eigen::Vector2d preshape;
  preshape << 0.75, 0.75;
  handSpace->convertPositionsToState(preshape, goalState);

  auto trajectory = m_ada->planToConfiguration(handSpace,
                                               handSkeleton,
                                               goalState,
                                               nullptr,
                                               1.0);
  auto future = m_ada->executeTrajectory(trajectory);
  future.wait();
}

void Robot::openHand() {
  auto handSkeleton = m_ada->getHand()->getMetaSkeleton();
  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());

  auto goalState = handSpace->createState();

  Eigen::Vector2d preshape;
  preshape << 0., 0.;
  handSpace->convertPositionsToState(preshape, goalState);

  auto trajectory = m_ada->planToConfiguration(handSpace,
                                               handSkeleton,
                                               goalState,
                                               nullptr,
                                               1.0);
  auto future = m_ada->executeTrajectory(trajectory);
  future.wait();
}

//void Robot::addSubMotions(const std::vector<std::shared_ptr<MotionPlanner>> &subMotions) {
//  m_subMotions = subMotions;
//}

//void Robot::plan(const ConfMotionPlanner &motion) {
//  auto stateSpace = motion.m_stateSpace;
//  auto skeleton = motion.m_skeleton;
//  auto conf = motion.m_goalConf;
//  auto goalState = stateSpace->createState();
//  stateSpace->convertPositionsToState(conf, goalState);
//
//  auto trajectory = m_ada->planToConfiguration(stateSpace,
//                                               skeleton,
//                                               goalState,
//                                               nullptr,
//                                               1.0);
//  auto future = m_ada->executeTrajectory(trajectory);
//  future.wait();
//}
//
//void Robot::plan(const ConnMotionPlanner &motion) {
//  auto grab = motion.m_grab;
//  if (grab) {
//    m_ada->getHand()->grab(motion.m_bodyToGrab);
//  } else {
//    m_ada->getHand()->ungrab();
//  }
//}
//
//void Robot::plan(const DeltaMotionPlanner &motion) {
//  auto skeleton = motion.m_skeleton;
//  auto stateSpace = motion.m_stateSpace;
//  Eigen::VectorXd delta_q(6);
//
//  for (int i = 0; i < motion.m_repeat_time; i++) {
//    delta_q << aikido::common::pseudoinverse(motion.m_jac) * motion.m_delta_x;
//    Eigen::VectorXd currPos = skeleton->getPositions();
//    ros::Duration(0.1).sleep();
//    Eigen::VectorXd new_pos = currPos + delta_q;
//    skeleton->setPositions(new_pos);
//  }
//}
//
//void Robot::plan(const TSRMotionPlanner &motion) {
//  auto goalTSR = motion.m_goalTSR;
//  auto stateSpace = motion.m_stateSpace;
//  auto skeleton = motion.m_skeleton;
//  auto bn = motion.m_bn;
//
//  auto ik = dart::dynamics::InverseKinematics::create(bn);
//  ik->setDofs(skeleton->getDofs());
//  auto rng = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
//  aikido::constraint::dart::InverseKinematicsSampleable ikSampleable(stateSpace,
//                                                                     skeleton,
//                                                                     goalTSR,
//                                                                     aikido::constraint::dart::createSampleableBounds(stateSpace, std::move(rng)),
//                                                                     ik,
//                                                                     10);
//  auto generator = ikSampleable.createSampleGenerator();
//  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations;
//  auto goalState = stateSpace->createState();
//  static const std::size_t maxSnapSamples{2};
//  std::size_t snapSamples = 0;
//  while (snapSamples < maxSnapSamples && generator->canSample()) {
//    std::lock_guard<std::mutex> lock(skeleton->getBodyNode(0)->getSkeleton()->getMutex());
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
//    auto trajectory = m_ada->planToConfiguration(stateSpace,
//                                                        skeleton,
//                                                        configurations[0],
//                                                        nullptr,
//                                                        10);
//    if (trajectory) {
//      ROS_INFO("Found the trajectory!");
//      aikido::trajectory::TrajectoryPtr timedTrajectory
//          = m_ada->smoothPath(skeleton, trajectory.get(), std::make_shared<aikido::constraint::Satisfied>(stateSpace));
//      auto future = m_ada->executeTrajectory(timedTrajectory);
//      future.wait();
//    }
//  } else {
//    ROS_INFO("Didn't find a valid goal state!");
//  }
//}



