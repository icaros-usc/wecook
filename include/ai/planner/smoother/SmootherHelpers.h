//
// Created by hejia on 9/16/19.
//

#ifndef WECOOK_SMOOTHERHELPERS_H
#define WECOOK_SMOOTHERHELPERS_H

#include <libada/Ada.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <ai/trajectory/HauserPath.h>

namespace wecook {
namespace planner {
/*
 * Simple short cutting
 */
aikido::trajectory::InterpolatedPtr simpleSmoothPath(const std::shared_ptr<ada::Ada> &ada,
                                                     const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Trajectory *path,
                                                     const aikido::constraint::TestablePtr &constraint);

aikido::trajectory::InterpolatedPtr simpleDoShortcut(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Interpolated &inputInterpolated,
                                                     aikido::common::RNG &rng,
                                                     const aikido::constraint::TestablePtr &collisionTestable,
                                                     const Eigen::VectorXd &maxVelocity,
                                                     const Eigen::VectorXd &maxAcceleration,
                                                     double timeLimit = 6.0,
                                                     double checkResolution = 5e-4,
                                                     double tolerance = 1e-3);

/*
 * Smoothing with Kris Hauser's parabolic smoother
 */
std::unique_ptr<ai::trajectory::HauserPath> hauserSmoothPathHauserPath(const std::shared_ptr<ada::Ada> &ada,
                                                                       const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                       const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                       const aikido::trajectory::Trajectory *path,
                                                                       const aikido::constraint::TestablePtr &constraint);

std::unique_ptr<ai::trajectory::HauserPath> hauserDoShortcutHauserPath(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                       const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                       const aikido::trajectory::Interpolated &inputInterpolated,
                                                                       aikido::common::RNG &rng,
                                                                       const aikido::constraint::TestablePtr &collisionTestable,
                                                                       const Eigen::VectorXd &maxVelocity,
                                                                       const Eigen::VectorXd &maxAcceleration,
                                                                       double timeLimit = 6.0,
                                                                       double checkResolution = 5e-4,
                                                                       double tolerance = 1e-3);

aikido::trajectory::UniqueSplinePtr hauserSmoothPath(const std::shared_ptr<ada::Ada> &ada,
                                                     const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Trajectory *path,
                                                     const aikido::constraint::TestablePtr &constraint);

aikido::trajectory::UniqueSplinePtr hauserDoShortcut(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                     const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                     const aikido::trajectory::Interpolated &inputInterpolated,
                                                     aikido::common::RNG &rng,
                                                     const aikido::constraint::TestablePtr &collisionTestable,
                                                     const Eigen::VectorXd &maxVelocity,
                                                     const Eigen::VectorXd &maxAcceleration,
                                                     double timeLimit = 3.0,
                                                     double checkResolution = 5e-4,
                                                     double tolerance = 1e-3);

aikido::trajectory::InterpolatedPtr hauserSmoothPathInterpolated(const std::shared_ptr<ada::Ada> &ada,
                                                                 const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                 const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                 const aikido::trajectory::Trajectory *path,
                                                                 const aikido::constraint::TestablePtr &constraint);

aikido::trajectory::InterpolatedPtr hauserDoShortcutInterpolated(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                 const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                 const aikido::trajectory::Interpolated &inputInterpolated,
                                                                 aikido::common::RNG &rng,
                                                                 const aikido::constraint::TestablePtr &collisionTestable,
                                                                 const Eigen::VectorXd &maxVelocity,
                                                                 const Eigen::VectorXd &maxAcceleration,
                                                                 double timeLimit = 8.0,
                                                                 double checkResolution = 5e-4,
                                                                 double tolerance = 1e-3);
}
}

#endif //WECOOK_SMOOTHERHELPERS_H
