//
// Created by hejia on 9/16/19.
//
#include <aikido/statespace/dart/SO2Joint.hpp>
#include "SmootherUtil.h"
#include "ai/planner/smoother/SmootherHelpers.h"

namespace wecook {
namespace planner {
/*
 * Dangerous
 */
void unroll(Eigen::MatrixXd &traj) {
  enum FLIP_TYPE {
    /// -pi to pi
    NEGATIVE,
    /// pi to -pi
    POSITIVE,
  };

  std::size_t cols = traj.cols();
  std::size_t rows = traj.rows();

  for (std::size_t col = 0; col < cols; ++col) {
    std::vector<std::pair<std::size_t, FLIP_TYPE >> flips{};
    Eigen::VectorXd colVec = traj.col(col);
    auto last = colVec(0);
    for (std::size_t row = 1; row < rows; ++row) {
      if (colVec(row) * last < 0) {
        // Check if it's a flip, i.e., from -pi to pi or pi to -pi
        if (last < 0) {
          if (fabs(colVec(row)) + fabs(last) > fabs(colVec(row) - M_PI) + fabs(last + M_PI)) {
            flips.emplace_back(std::pair<std::size_t, FLIP_TYPE >{row, NEGATIVE});
          }
        } else {
          if (fabs(colVec(row)) + fabs(last) > fabs(colVec(row) + M_PI) + fabs(last - M_PI)) {
            flips.emplace_back(std::pair<std::size_t, FLIP_TYPE >{row, POSITIVE});
          }
        }
      }
      last = colVec(row);
    }
    // Unpate joint values based on flips
    for (const auto &flip : flips) {
      auto startIndex = flip.first;
      Eigen::VectorXd addVec(rows - startIndex);
      if (flip.second == NEGATIVE) {
        addVec.setConstant(-2 * M_PI);
      } else {
        addVec.setConstant(2 * M_PI);
      }
      // Update
      traj.col(col).segment(startIndex, rows - startIndex) += addVec;
    }
  }
}

/*
 * Dangerous
 */
aikido::trajectory::InterpolatedPtr unroll(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                                           const aikido::trajectory::Trajectory * originalTraj) {
  auto originalInterpolated = dynamic_cast<const aikido::trajectory::Interpolated *>(originalTraj);

  if (!originalInterpolated) {
    std::__throw_invalid_argument(
        "[SmootherHelpers::unroll]: The trajectory to be preprocessed should be interpolated!");
  }

  std::shared_ptr<aikido::statespace::GeodesicInterpolator>
      interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);
  auto newInterpolated = std::make_shared<aikido::trajectory::Interpolated>(stateSpace, interpolator);

  std::size_t numWaypoints = originalInterpolated->getNumWaypoints();
  Eigen::MatrixXd traj(numWaypoints, stateSpace->getDimension());
  for (std::size_t i = 0; i < numWaypoints; ++i) {
    Eigen::VectorXd temp;
    stateSpace->logMap(originalInterpolated->getWaypoint(i), temp);
    traj.row(i) = temp;
  }

  unroll(traj);

  const auto n = stateSpace->getNumSubspaces();
  for (std::size_t i = 0; i < numWaypoints; ++i) {
    auto newState = stateSpace->createState();
    size_t index = 0;
    for (std::size_t j = 0; j < n; ++j) {
      auto subSpace = stateSpace->getSubspace<aikido::statespace::dart::JointStateSpace>(j);
      auto dim = subSpace->getDimension();
      auto newSubState = stateSpace->getSubState<>(newState, j);
      if (dynamic_cast<const aikido::statespace::dart::SO2Joint *>(subSpace.get())) {
        dynamic_cast<const aikido::statespace::dart::SO2Joint *>(subSpace.get())->expMap(traj.row(i).segment(index, dim), newSubState);
      } else {
        subSpace->expMap(traj.row(i).segment(index, dim), newSubState);
      }
      index += dim;
    }
    newInterpolated->addWaypoint(i, newState);
  }

  return newInterpolated;
}

/*
 * Simple short cutting
 */
aikido::trajectory::InterpolatedPtr simpleSmoothPath(const std::shared_ptr<ada::Ada> &ada,
                                                     const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Trajectory *path,
                                                     const aikido::constraint::TestablePtr &constraint) {
  Eigen::VectorXd velocityLimits = ada->getVelocityLimits();
  Eigen::VectorXd accelerationLimits = ada->getAccelerationLimits();
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
  Eigen::VectorXd velocityLimits = ada->getVelocityLimits();
  Eigen::VectorXd accelerationLimits = ada->getAccelerationLimits();
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
  Eigen::VectorXd velocityLimits = ada->getVelocityLimits();
  Eigen::VectorXd accelerationLimits = ada->getAccelerationLimits();
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

std::unique_ptr<ai::trajectory::HauserPath> hauserSmoothPathHauserPath(const std::shared_ptr<ada::Ada> &ada,
                                                                       const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                       const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                       const aikido::trajectory::Trajectory *path,
                                                                       const aikido::constraint::TestablePtr &constraint) {
  Eigen::VectorXd velocityLimits = ada->getVelocityLimits();
  Eigen::VectorXd accelerationLimits = ada->getAccelerationLimits();
//  auto interpolated = dynamic_cast<const aikido::trajectory::Interpolated *>(path);
//  if (!interpolated) {
//    auto spline = dynamic_cast<const aikido::trajectory::Spline *>(path);
//    if (!spline) {
//      throw std::invalid_argument("Path should be either spline or Interpolated.");
//    }
//    interpolated = detail::SplineToInterpolated(*spline);
//  }
  // before smoothing, we will add pi to each SO2 joint, so there will be no wield rotation.
  auto processedTraj = unroll(metaStateSpace, path);
  return hauserDoShortcutHauserPath(metaStateSpace,
                                    metaSkeletonPtr,
                                    *processedTraj,
                                    *(ada->cloneRNG().get()),
                                    constraint,
                                    velocityLimits,
                                    accelerationLimits);
}

std::unique_ptr<ai::trajectory::HauserPath> hauserDoShortcutHauserPath(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
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
      detail::InterpolatedToHauserDynamicPath(inputInterpolated,
                                              maxVelocity,
                                              maxAcceleration,
                                              metaStateSpace,
                                              metaSkeletonPtr);

  detail::hauserDoShortcut(*dynamicPath, collisionTestable, timeLimit, checkResolution, tolerance, rng);

  auto outputTraj = std::make_unique<ai::trajectory::HauserPath>(dynamicPath, stateSpace);

  return outputTraj;
}
}
}
