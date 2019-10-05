//
// Created by hejia on 9/17/19.
//

#ifndef WECOOK_SMOOTHERUTIL_H
#define WECOOK_SMOOTHERUTIL_H

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/constraint/Testable.hpp>
#include <ai/external/hauser_parabolic_smoother/DynamicPath.h>

#include "SimpleDynamicPath.h"

namespace wecook {
namespace planner {
namespace detail {

std::unique_ptr<SimpleDynamicPath> InterpolatedToSimpleDynamicPath(const aikido::trajectory::Interpolated &inputInterpolated,
                                                                   const Eigen::VectorXd &maxVelocity,
                                                                   const Eigen::VectorXd &maxAcceleration,
                                                                   const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                   const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                   bool preserveWaypointVelocity = false);

std::shared_ptr<ParabolicRamp::DynamicPath> InterpolatedToHauserDynamicPath(const aikido::trajectory::Interpolated &inputInterpolated,
                                                                            const Eigen::VectorXd &maxVelocity,
                                                                            const Eigen::VectorXd &maxAcceleration,
                                                                            const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                            const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                            bool preserveWaypointVelocity = false);

std::unique_ptr<aikido::trajectory::Spline> HauserDynamicPathToSpline(const ParabolicRamp::DynamicPath &_inputPath,
                                                                      double _startTime,
                                                                      const aikido::statespace::ConstStateSpacePtr &_stateSpace);

aikido::trajectory::InterpolatedPtr HauserDynamicPathToInterpolated(const ParabolicRamp::DynamicPath &_inputPath,
                                                                    double _startTime,
                                                                    const aikido::statespace::ConstStateSpacePtr &_stateSpace);

aikido::trajectory::InterpolatedPtr SimpleDynamicPathToInterpolated(const SimpleDynamicPath &_inputPath,
                                                                    const aikido::statespace::ConstStateSpacePtr &_stateSpace);

aikido::trajectory::Interpolated *SplineToInterpolated(const aikido::trajectory::Spline &inputSpline);

//=====================================================================================================================

/*
 * simple shortcutting
 */
void simpleDoShortcut(SimpleDynamicPath &dynamicPath,
                      aikido::constraint::TestablePtr testable,
                      double timelimit,
                      double checkResolution,
                      double tolerance,
                      aikido::common::RNG &rng);

/*
 * smoothing with Kris Hauser's smoother
 */
void hauserDoShortcut(ParabolicRamp::DynamicPath &hauserDynamicPath,
                      aikido::constraint::TestablePtr testable,
                      double timelimit,
                      double checkResolution,
                      double tolerancce,
                      aikido::common::RNG &rng);
}
}
}

#endif //WECOOK_SMOOTHERUTIL_H
