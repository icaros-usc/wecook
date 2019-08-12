//
// Created by hejia on 8/11/19.
//

#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/dart/FrameDifferentiable.hpp>
#include <aikido/constraint/dart/FrameTestable.hpp>
#include <aikido/constraint/NewtonsMethodProjectable.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include "wecook/TSRMotionwithConstraintPlanner.h"

using namespace wecook;

void TSRMotionwithConstraintPlanner::plan(const std::shared_ptr<ada::Ada> &ada) {
  auto rng = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
  auto crrtParameters = aikido::robot::util::CRRTPlannerParameters{rng.get()};
  // Create seed constraint
  std::shared_ptr<aikido::constraint::Sampleable>
      seedConstraint = aikido::constraint::dart::createSampleableBounds(m_stateSpace, crrtParameters.rng->clone());
  // Create an IK solver with metaSkeleton dofs
  auto ik = dart::dynamics::InverseKinematics::create(m_bn);
  ik->setDofs(m_skeleton->getDofs());
  auto goalSampleable = std::make_shared<aikido::constraint::dart::InverseKinematicsSampleable>(m_stateSpace,
                                                                                                m_skeleton,
                                                                                                m_goalTSR,
                                                                                                seedConstraint,
                                                                                                ik,
                                                                                                crrtParameters.maxNumTrials);
  // Create goal testable
  auto goalTestable =
      std::make_shared<aikido::constraint::dart::FrameTestable>(m_stateSpace, m_skeleton, m_bn, m_goalTSR);
  // Create constraint sampleable
  auto constraintSampleable = std::make_shared<aikido::constraint::dart::InverseKinematicsSampleable>(m_stateSpace,
                                                                                                      m_skeleton,
                                                                                                      m_constraintTSR,
                                                                                                      seedConstraint,
                                                                                                      ik,
                                                                                                      crrtParameters.maxNumTrials);
  // Create constraint projectable
  auto frameDiff =
      std::make_shared<aikido::constraint::dart::FrameDifferentiable>(m_stateSpace, m_skeleton, m_bn, m_constraintTSR);
  std::vector<double> projectionToleranceVec(
      frameDiff->getConstraintDimension(), crrtParameters.projectionTolerance);
  auto constraintProjectable = std::make_shared<aikido::constraint::NewtonsMethodProjectable>(frameDiff,
                                                                                              projectionToleranceVec,
                                                                                              crrtParameters.projectionMaxIteration);
  auto robot = m_skeleton->getBodyNode(0)->getSkeleton();
  std::unique_lock<std::mutex> lock(robot->getMutex());
  // Current state
  auto startState = m_stateSpace->getScopedStateFromMetaSkeleton(m_skeleton.get());

  auto collisionConstraint
      = ada->getFullCollisionConstraint(m_stateSpace, m_skeleton, m_collisionFree);

  // we will do our own planner
  auto interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(m_stateSpace);
  auto distanceMetric = aikido::distance::createDistanceMetric(m_stateSpace);
  auto boundsConstraint = aikido::constraint::dart::createTestableBounds(m_stateSpace);
  auto boundsProjector = aikido::constraint::dart::createProjectableBounds(m_stateSpace);
  auto si = aikido::planner::ompl::getSpaceInformation(m_stateSpace,
                                                       interpolator,
                                                       std::move(distanceMetric),
                                                       constraintSampleable,
                                                       collisionConstraint,
                                                       std::move(boundsConstraint),
                                                       std::move(boundsProjector),
                                                       crrtParameters.maxDistanceBtwProjections);
  // Set the start and goal
  auto pdef = aikido::planner::ompl::ompl_make_shared<::ompl::base::ProblemDefinition>(si);
  auto sspace =
      aikido::planner::ompl::ompl_static_pointer_cast<aikido::planner::ompl::GeometricStateSpace>(si->getStateSpace());
  auto start = sspace->allocState(startState.clone());
  pdef->addStartState(start);
  sspace->freeState(start);
  auto goalRegion = aikido::planner::ompl::getGoalRegion(si, goalTestable, goalSampleable);
  pdef->setGoal(goalRegion);
  auto planner = aikido::planner::ompl::ompl_make_shared<aikido::planner::ompl::CRRTConnect>(si);
  planner->setPathConstraint(constraintProjectable);
  planner->setRange(crrtParameters.maxExtensionDistance);
  planner->setProjectionResolution(crrtParameters.maxDistanceBtwProjections);
  planner->setConnectionRadius(crrtParameters.minTreeConnectionDistance);
  planner->setMinStateDifference(crrtParameters.minStepSize);

  auto traj = planOMPL(planner, pdef, std::move(m_stateSpace), std::move(interpolator), 100);

  if (traj) {
    ROS_INFO("Found valid trajectory!");
    lock.unlock();
    auto future = ada->executeTrajectory(traj);
    future.wait();
  }
}