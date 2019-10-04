//
// Created by hejia on 9/22/19.
//

#ifndef WECOOK_HAUSERUTIL_H
#define WECOOK_HAUSERUTIL_H

#include <Eigen/Eigen>

#include "DynamicPath.h"

namespace wecook {
namespace ai {
namespace external {
namespace hauser_parabolic_smoother {
/*
 * Utility functions
 */
/// Convert a Eigen vector to a ParabolicRamp vector
/// \param _x an Eigen vector
/// \return a ParabolicRamp vector
ParabolicRamp::Vector toVector(const Eigen::VectorXd &_x);

/// Convert a ParabolicRamp vector to a Eigen vector
/// \param _x a ParabolicRamp vector
/// \return an Eigen vector
Eigen::VectorXd toEigen(const ParabolicRamp::Vector &_x);

/// Evaluate the position and the velocity of a dynamic path
/// given time t
/// \param _path a dynamic path
/// \param _t time
/// \param[out] position at time \c _t
/// \param[out] velocity at time \c _t
void evaluateAtTime(const ParabolicRamp::DynamicPath &_path, double _t,
                    Eigen::VectorXd &_position, Eigen::VectorXd &_velocity);
}
}
}
}

#endif //WECOOK_HAUSERUTIL_H
