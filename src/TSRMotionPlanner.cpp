//
// Created by hejia on 8/6/19.
//

#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include "wecook/TSRMotionPlanner.h"

using namespace wecook;

void TSRMotionPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  auto ik = dart::dynamics::InverseKinematics::create(m_bn);
  ik->setDofs(m_skeleton->getDofs());
  auto rng = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
  aikido::constraint::dart::InverseKinematicsSampleable ikSampleable(m_stateSpace,
                                                                     m_skeleton,
                                                                     m_goalTSR,
                                                                     aikido::constraint::dart::createSampleableBounds(
                                                                         m_stateSpace,
                                                                         std::move(rng)),
                                                                     ik,
                                                                     10);
  auto generator = ikSampleable.createSampleGenerator();
  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations;
  auto goalState = m_stateSpace->createState();
  static const std::size_t maxSnapSamples{2};
  std::size_t snapSamples = 0;
  while (snapSamples < maxSnapSamples && generator->canSample()) {
    std::lock_guard<std::mutex> lock(m_skeleton->getBodyNode(0)->getSkeleton()->getMutex());
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
    auto trajectory = ada->planToConfiguration(m_stateSpace,
                                               m_skeleton,
                                               configurations[0],
                                               nullptr,
                                               10);
    if (trajectory) {
      ROS_INFO("Found the trajectory!");
      aikido::trajectory::TrajectoryPtr timedTrajectory
          =
          ada->smoothPath(m_skeleton, trajectory.get(), std::make_shared<aikido::constraint::Satisfied>(m_stateSpace));
      auto future = ada->executeTrajectory(timedTrajectory);
      future.wait();
    }
  } else {
    ROS_INFO("Didn't find a valid goal state!");
  }
}