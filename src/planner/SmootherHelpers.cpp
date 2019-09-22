//
// Created by hejia on 9/16/19.
//
#include "SmootherUtil.h"
#include "wecook/planner/SmootherHelpers.h"

namespace wecook {
namespace planner {
/*
 * Simple short cutting
 */
aikido::trajectory::InterpolatedPtr simpleSmoothPath(const std::shared_ptr<ada::Ada> &ada,
                                                     const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Trajectory *path,
                                                     const aikido::constraint::TestablePtr &constraint) {
  Eigen::VectorXd velocityLimits = ada->getVelocityLimits(metaSkeletonPtr);
  Eigen::VectorXd accelerationLimits = ada->getAccelerationLimits(metaSkeletonPtr);
  auto interpolated = dynamic_cast<const aikido::trajectory::Interpolated *>(path);
  if (!interpolated) {
    auto spline = dynamic_cast<const aikido::trajectory::Spline *>(path);
    if (!spline) {
      throw std::invalid_argument("Path should be either spline or Interpolated.");
    }
    interpolated = detail::SplineToInterpolated(*spline);
  }
  return simpleDoShortcut(metaStateSpace,
                          metaSkeletonPtr,
                          *interpolated,
                          *(ada->cloneRNG().get()),
                          constraint,
                          velocityLimits,
                          accelerationLimits);
}

aikido::trajectory::InterpolatedPtr simpleDoShortcut(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Interpolated &inputInterpolated,
                                                     aikido::common::RNG &rng,
                                                     const aikido::constraint::TestablePtr &collisionTestable,
                                                     const Eigen::VectorXd &maxVelocity,
                                                     const Eigen::VectorXd &maxAcceleration,
                                                     double timeLimit,
                                                     double checkResolution,
                                                     double tolerance) {
  auto stateSpace = inputInterpolated.getStateSpace();

  auto dynamicPath =
      detail::InterpolatedToSimpleDynamicPath(inputInterpolated,
                                              maxVelocity,
                                              maxAcceleration,
                                              metaStateSpace,
                                              metaSkeletonPtr);

  detail::simpleDoShortcut(*dynamicPath, collisionTestable, timeLimit, checkResolution, tolerance, rng);

  auto outputTraj = detail::SimpleDynamicPathToInterpolated(*dynamicPath,
                                                            stateSpace);

  return outputTraj;
}

/*
 * Smoothing with Kris Hauser's parabolic smoother
 */
aikido::trajectory::UniqueSplinePtr hauserSmoothPath(const std::shared_ptr<ada::Ada> &ada,
                                                     const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Trajectory *path,
                                                     const aikido::constraint::TestablePtr &constraint) {
  Eigen::VectorXd velocityLimits = ada->getVelocityLimits(metaSkeletonPtr);
  Eigen::VectorXd accelerationLimits = ada->getAccelerationLimits(metaSkeletonPtr);
  auto interpolated = dynamic_cast<const aikido::trajectory::Interpolated *>(path);
  if (!interpolated) {
    auto spline = dynamic_cast<const aikido::trajectory::Spline *>(path);
    if (!spline) {
      throw std::invalid_argument("Path should be either spline or Interpolated.");
    }
    interpolated = detail::SplineToInterpolated(*spline);
  }
  return hauserDoShortcut(metaStateSpace,
                          metaSkeletonPtr,
                          *interpolated,
                          *(ada->cloneRNG().get()),
                          constraint,
                          velocityLimits,
                          accelerationLimits);
}

aikido::trajectory::UniqueSplinePtr hauserDoShortcut(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Interpolated &inputInterpolated,
                                                     aikido::common::RNG &rng,
                                                     const aikido::constraint::TestablePtr &collisionTestable,
                                                     const Eigen::VectorXd &maxVelocity,
                                                     const Eigen::VectorXd &maxAcceleration,
                                                     double timeLimit,
                                                     double checkResolution,
                                                     double tolerance) {
  auto stateSpace = inputInterpolated.getStateSpace();

  double startTime = inputInterpolated.getStartTime();

  auto dynamicPath =
      detail::InterpolatedToHauserDynamicPath(inputInterpolated,
                                              maxVelocity,
                                              maxAcceleration,
                                              metaStateSpace,
                                              metaSkeletonPtr);

  detail::hauserDoShortcut(*dynamicPath, collisionTestable, timeLimit, checkResolution, tolerance, rng);

  auto outputTraj = detail::HauserDynamicPathToSpline(*dynamicPath,
                                                      startTime,
                                                      stateSpace);

  return outputTraj;
}

aikido::trajectory::InterpolatedPtr hauserSmoothPathInterpolated(const std::shared_ptr<ada::Ada> &ada,
                                                                 const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                 const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                 const aikido::trajectory::Trajectory *path,
                                                                 const aikido::constraint::TestablePtr &constraint) {
  Eigen::VectorXd velocityLimits = ada->getVelocityLimits(metaSkeletonPtr);
  Eigen::VectorXd accelerationLimits = ada->getAccelerationLimits(metaSkeletonPtr);
  auto interpolated = dynamic_cast<const aikido::trajectory::Interpolated *>(path);
  if (!interpolated) {
    auto spline = dynamic_cast<const aikido::trajectory::Spline *>(path);
    if (!spline) {
      throw std::invalid_argument("Path should be either spline or Interpolated.");
    }
    interpolated = detail::SplineToInterpolated(*spline);
  }
  return hauserDoShortcutInterpolated(metaStateSpace,
                                      metaSkeletonPtr,
                                      *interpolated,
                                      *(ada->cloneRNG().get()),
                                      constraint,
                                      velocityLimits,
                                      accelerationLimits);
}

aikido::trajectory::InterpolatedPtr hauserDoShortcutInterpolated(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                 const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                 const aikido::trajectory::Interpolated &inputInterpolated,
                                                                 aikido::common::RNG &rng,
                                                                 const aikido::constraint::TestablePtr &collisionTestable,
                                                                 const Eigen::VectorXd &maxVelocity,
                                                                 const Eigen::VectorXd &maxAcceleration,
                                                                 double timeLimit,
                                                                 double checkResolution,
                                                                 double tolerance) {
  auto stateSpace = inputInterpolated.getStateSpace();

  double startTime = inputInterpolated.getStartTime();

  auto dynamicPath =
      detail::InterpolatedToHauserDynamicPath(inputInterpolated,
                                              maxVelocity,
                                              maxAcceleration,
                                              metaStateSpace,
                                              metaSkeletonPtr);

  detail::hauserDoShortcut(*dynamicPath, collisionTestable, timeLimit, checkResolution, tolerance, rng);

  auto outputTraj = detail::HauserDynamicPathToInterpolated(*dynamicPath,
                                                            startTime,
                                                            stateSpace);

  return outputTraj;
}
}
}
