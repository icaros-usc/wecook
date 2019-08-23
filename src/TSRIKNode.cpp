//
// Created by hejia on 8/20/19.
//

#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>

#include "wecook/TSRIKNode.h"

using namespace wecook;

void TSRIKNode::plan(const std::shared_ptr<ada::Ada> &ada) {
  if (m_condition) {
    ROS_INFO("[TSRIKNode::plan]: Waiting for condition to be verified!");
    while (!m_condition->isSatisfied()) {
      // sleep a little bit
      ros::Duration(1.).sleep();
    }
    ROS_INFO("[TSRIKNode::plan]: Condition is verified!");
  }

  // first we need to sample a valid goal state
  if (!m_bn)
    throw std::invalid_argument("[TSRIKNode::plan]: m_bn is NULL");
  auto ik = dart::dynamics::InverseKinematics::create(m_bn);
  if (!m_skeleton)
    throw std::invalid_argument("[TSRIKNode::plan]: m_skeleton is NULL");
  if (m_skeleton->getNumDofs() == 0)
    throw std::invalid_argument("[TSRIKNode::plan]: m_skeleton has 0 degrees of freedom.");
  ik->setDofs(m_skeleton->getDofs());

  auto rng = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));

  try {
    auto startState = m_stateSpace->getScopedStateFromMetaSkeleton(m_skeleton.get());
    aikido::constraint::dart::InverseKinematicsSampleable ikSampleable(m_stateSpace,
                                                                       m_skeleton,
                                                                       m_goalTSR,
                                                                       aikido::constraint::dart::createSampleableBounds(
                                                                           m_stateSpace,
                                                                           std::move(rng)),
                                                                       ik,
                                                                       10,
                                                                       m_debug);
    auto generator = ikSampleable.createSampleGenerator();
    std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations;
    auto goalState = m_stateSpace->createState();
    static const std::size_t maxSnapSamples{20};
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

    if (!m_debug)
      m_stateSpace->setState(m_skeleton.get(), startState.getState());
    if (!configurations.empty()) {
      ROS_INFO("Found a valid goal state!");
      ROS_INFO("Start planning a path to the goal configuration!");

      std::unique_lock<std::mutex> lock(m_skeleton->getBodyNode(0)->getSkeleton()->getMutex());

      // run snap planner
      auto snapPlanner = std::make_shared<aikido::planner::SnapConfigurationToConfigurationPlanner>(m_stateSpace,
                                                                                                    std::make_shared<
                                                                                                        aikido::statespace::GeodesicInterpolator>(
                                                                                                        m_stateSpace));
      auto collisionConstraint = ada->getFullCollisionConstraint(m_stateSpace, m_skeleton, m_collisionFree);
      auto problem = aikido::planner::ConfigurationToConfiguration(m_stateSpace,
                                                                   startState,
                                                                   configurations[0],
                                                                   collisionConstraint);
      aikido::planner::SnapConfigurationToConfigurationPlanner::Result pResult;
      auto utimedTrajectory = snapPlanner->plan(problem, &pResult);

      lock.unlock();
      if (utimedTrajectory) {
        ROS_INFO("Found the trajectory!");
        std::vector<aikido::constraint::ConstTestablePtr> constraints;
        if (m_collisionFree)
          constraints.push_back(m_collisionFree);
        auto testable = std::make_shared<aikido::constraint::TestableIntersection>(m_stateSpace, constraints);
        std::unique_lock<std::mutex> slock(m_skeleton->getBodyNode(0)->getSkeleton()->getMutex());
        aikido::trajectory::TrajectoryPtr timedTrajectory = ada->smoothPath(m_skeleton, utimedTrajectory.get(), testable);
        m_stateSpace->setState(m_skeleton.get(), startState.getState());
        slock.unlock();
        auto future = ada->executeTrajectory(utimedTrajectory);
        future.wait();
      } else {
        ROS_INFO("[TSRIKNode::plan]: Didn't find a valid trajectory!");
      }
    } else {
      ROS_INFO("Didn't find a valid goal state!");
    }
  } catch (const std::invalid_argument &ia) {
    std::cerr << "Invalid argument: " << ia.what() << '\n';
  }
}