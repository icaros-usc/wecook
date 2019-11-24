//
// Created by hejia on 8/6/19.
//
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>

#include "wecook/LowLevelMotionNodes/TSRMotionNode.h"
#include "ai/planner/smoother/SmootherHelpers.h"

using namespace wecook;

void TSRMotionNode::validate() {
  if (!m_bn)
    throw std::invalid_argument("[TSRMotionNode::plan]: m_bn is NULL");
  if (!m_skeleton)
    throw std::invalid_argument("[TSRMotionNode::plan]: m_skeleton is NULL");
  if (m_skeleton->getNumDofs() == 0)
    throw std::invalid_argument("[TSRMotionNode::plan]: m_skeleton has 0 degrees of freedom.");
}

void TSRMotionNode::computeIK(
    std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> &configurations,
    const aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState &startState) {
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
  auto goalState = m_stateSpace->createState();
  static const std::size_t maxNumSamples{30};
  std::size_t numSamples = 0;
  while (numSamples < maxNumSamples && generator->canSample()) {
    std::lock_guard<std::mutex> lock(m_skeleton->getBodyNode(0)->getSkeleton()->getMutex());
    bool sampled = generator->sample(goalState);
    ++numSamples;
    if (!sampled) {
      continue;
    }
    configurations.emplace_back(goalState.clone());
  }

  // rank the generated configurations
  if (!configurations.empty()) {
    aikido::distance::ConstConfigurationRankerPtr configurationRanker(nullptr);
    auto nominalState = m_stateSpace->createState();
    configurationRanker =
        std::make_shared<const aikido::distance::NominalConfigurationRanker>(m_stateSpace, m_skeleton, startState);
    configurationRanker->rankConfigurations(configurations);
  }

  if (!m_debug)
    m_stateSpace->setState(m_skeleton.get(), startState.getState());
}

aikido::trajectory::TrajectoryPtr TSRMotionNode::planToConfiguration(
    const std::shared_ptr<ada::Ada> &ada,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const aikido::statespace::StateSpace::State *startState,
    const aikido::statespace::StateSpace::State *goalState,
    const aikido::constraint::dart::CollisionFreePtr &collisionFree,
    aikido::planner::Planner::Result *pResult) {
  using aikido::planner::ompl::planOMPL;
  using aikido::planner::ConfigurationToConfiguration;
  using aikido::planner::SnapConfigurationToConfigurationPlanner;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = aikido::statespace::dart::MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // First test with Snap Planner
  aikido::trajectory::TrajectoryPtr untimedTrajectory;

  auto collisionConstraint
      = ada->getFullCollisionConstraint(space, metaSkeleton, collisionFree);

  auto problem = ConfigurationToConfiguration(
      space, startState, goalState, collisionConstraint);
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      space, std::make_shared<aikido::statespace::GeodesicInterpolator>(space));
  untimedTrajectory = planner->plan(problem, pResult);

  // Return if the trajectory is non-empty
  if (untimedTrajectory)
    return untimedTrajectory;

  auto plannerOMPL = std::
  make_shared<aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner<::ompl::geometric::RRTConnect>>(space,
                                                                                                             ada->cloneRNG().get());

  aikido::planner::ompl::ompl_dynamic_pointer_cast<::ompl::geometric::RRTConnect>(plannerOMPL->getOMPLPlanner())->setRange(
      0.1);

  untimedTrajectory = plannerOMPL->plan(problem, pResult);

  return untimedTrajectory;
}

aikido::trajectory::TrajectoryPtr TSRMotionNode::smoothTrajectory(aikido::trajectory::TrajectoryPtr untimedTrajectory,
                                                                  const std::shared_ptr<ada::Ada> &ada) {
  auto num_waypoints = dynamic_cast<aikido::trajectory::Interpolated *>(untimedTrajectory.get())->getNumWaypoints();
  ROS_INFO_STREAM("[TSRMotionNode::smoothTrajectory]: The trajectory has "
                      << num_waypoints
                      << " waypoints");

  auto robot = m_skeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());

  if (num_waypoints == 2) {
    // we still need to retime the shortcutting path
    aikido::trajectory::TrajectoryPtr timedTrajectory = ada->retimePath(m_skeleton, untimedTrajectory.get());
    return timedTrajectory;
  } else {
    std::vector<aikido::constraint::ConstTestablePtr> constraints;
    if (m_collisionFree)
      constraints.push_back(m_collisionFree);
    auto testable = std::make_shared<aikido::constraint::TestableIntersection>(m_stateSpace, constraints);
    aikido::trajectory::TrajectoryPtr timedTrajectory =
        planner::hauserSmoothPathHauserPath(ada, m_stateSpace, m_skeleton, untimedTrajectory.get(), testable);
    return timedTrajectory;
  }
}

void TSRMotionNode::executeTrajectory(const aikido::trajectory::TrajectoryPtr &timedTrajectory,
                                      const std::shared_ptr<ada::Ada> &ada,
                                      const std::shared_ptr<ada::Ada> &adaImg,
                                      const aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState &startState) {
  m_stateSpace->setState(m_skeleton.get(), startState.getState());

  if (!ada->ifSim()) ada->startTrajectoryExecutor();
  auto future = ada->executeTrajectory(timedTrajectory);
  try {
    future.get();
  }
  catch (const std::exception &e) {
    ROS_ERROR("[TSRMotionNode::executeTrajectory:] trajectory execution failed: %s", e.what());
  }
  if (ada->ifSim()) {
    auto futureImg = adaImg->executeTrajectory(timedTrajectory);
    try {
      futureImg.get();
    }
    catch (const std::exception &e) {
      ROS_ERROR("[TSRMotionNode::executeTrajectory:] trajectory execution failed: %s", e.what());
    }
  }
  if (!ada->ifSim()) ada->stopTrajectoryExecutor();
}

void TSRMotionNode::plan(const std::shared_ptr<ada::Ada> &ada,
                         const std::shared_ptr<ada::Ada> &adaImg,
                         Result *result) {
  validate();

  // startState is important, since planning and computation will change the skeleton's position
  auto startState = m_stateSpace->getScopedStateFromMetaSkeleton(m_skeleton.get());

  /*--------------------------------Compute IK---------------------------------------------------------------*/
  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> configurations;
  computeIK(configurations, startState);
  if (configurations.empty()) {
    ROS_ERROR("[TSRMotionNode::plan]: No valid ik solution found!");
    if (result) {
      result->setStatus(MotionNode::Result::StatusType::INVALID_IK);
    }
    return;
  }

  /*--------------------------------Plan trajectory-----------------------------------------------------------*/
  ROS_INFO("[TSRMotionNode::plan]: Start planning a path to the goal configuration!");
  std::size_t curr_conf = 0;
  while (curr_conf < configurations.size()) {
    aikido::planner::Planner::Result planningResult{};
    auto trajectory = planToConfiguration(ada,
                                          m_stateSpace,
                                          m_skeleton,
                                          startState,
                                          configurations[curr_conf],
                                          m_collisionFree,
                                          &planningResult);
    if (trajectory) {
      auto timedTrajectory = smoothTrajectory(trajectory, ada);
      executeTrajectory(timedTrajectory, ada, adaImg, startState);
      if (result) {
        result->setStatus(MotionNode::Result::StatusType::SUCCEEDED);
      }
      return;
    } else if (planningResult.getStatus() == aikido::planner::Planner::Result::StatusType::INVALID_START) {
      ROS_ERROR("[TSRMotionNode::plan]: The start state is invalid");
      if (result) {
        result->setStatus(MotionNode::Result::StatusType::INVALID_START);
      }
      return;
    } else if (planningResult.getStatus() == aikido::planner::Planner::Result::StatusType::INVALID_GOAL) {
      ROS_WARN("[TSRMotionNode::plan]: The goal state is invalid, trying different goal");
      if (result) {
        result->setStatus(MotionNode::Result::StatusType::INVALID_GOAL);
      }
    }
    curr_conf += 1;
  }
}