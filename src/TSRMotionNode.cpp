//
// Created by hejia on 8/6/19.
//

#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include "wecook/TSRMotionNode.h"

using namespace wecook;

void TSRMotionNode::plan(const std::shared_ptr<ada::Ada> &ada) {
  ROS_INFO("Debug");
  if (m_condition) {
    ROS_INFO("Waiting for condition to be verified!");
    while (!m_condition->isSatisfied()) {
      // sleep a little bit
      ros::Duration(1.).sleep();
    }
    ROS_INFO("Condition is verified!");
  }

  if (!m_bn)
    throw std::invalid_argument("[TSRMotionNode::plan]: m_bn is NULL");
  auto ik = dart::dynamics::InverseKinematics::create(m_bn);

  if (!m_skeleton)
    throw std::invalid_argument("[TSRMotionNode::plan]: m_skeleton is NULL");
  if (m_skeleton->getNumDofs() == 0)
    throw std::invalid_argument("[TSRMotionNode::plan]: m_skeleton has 0 degrees of freedom.");
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
    static const std::size_t maxSnapSamples{30};
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
      // rank configurations we got
      aikido::distance::ConstConfigurationRankerPtr configurationRanker(nullptr);
      auto nominalState = m_stateSpace->createState();
      configurationRanker =
          std::make_shared<const aikido::distance::NominalConfigurationRanker>(m_stateSpace, m_skeleton, startState);
      configurationRanker->rankConfigurations(configurations);
      auto trajectory = ada->planToConfiguration(m_stateSpace,
                                                 m_skeleton,
                                                 configurations[0],
                                                 m_collisionFree,
                                                 10);
      if (trajectory) {
        ROS_INFO("Found the trajectory!");
        ROS_INFO_STREAM("[TSRMotionNode::plan]: The trajectory has "
                            << dynamic_cast<aikido::trajectory::Interpolated *>(trajectory.get())->getNumWaypoints()
                            << " waypoints");
        std::vector<aikido::constraint::ConstTestablePtr> constraints;
        if (m_collisionFree)
          constraints.push_back(m_collisionFree);
        auto testable = std::make_shared<aikido::constraint::TestableIntersection>(m_stateSpace, constraints);
        std::unique_lock<std::mutex> lock(m_skeleton->getBodyNode(0)->getSkeleton()->getMutex());
        aikido::trajectory::TrajectoryPtr timedTrajectory = ada->smoothPath(m_skeleton,
                                                                            trajectory.get(),
                                                                            testable);
        m_stateSpace->setState(m_skeleton.get(), startState.getState());
        lock.unlock();
        ROS_INFO_STREAM("[TSRMotionNode::plan]: The smoothed trajectory has "
                            << dynamic_cast<aikido::trajectory::Spline *>(timedTrajectory.get())->getNumWaypoints()
                            << " waypoints");
        auto future = ada->executeTrajectory(timedTrajectory);
        future.wait();
      } else {
        ROS_INFO("[TSRMotionNode::plan]: Didn't find a valid trajectory!");
      }
    } else {
      ROS_INFO("Didn't find a valid goal state!");
    }
  } catch (const std::invalid_argument &ia) {
    std::cerr << "Invalid argument: " << ia.what() << '\n';
  }
}