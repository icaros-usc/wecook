//
// Created by hejia on 9/22/19.
//
#include "ai/external/hauser_parabolic_smoother/HauserUtil.h"
#include "ai/external/hauser_parabolic_smoother/DynamicPath.h"

namespace wecook {
namespace ai {
namespace external {
namespace hauser_parabolic_smoother {
//==============================================================================
ParabolicRamp::Vector toVector(const Eigen::VectorXd &_x) {
  ParabolicRamp::Vector output(_x.size());

  for (int i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

//==============================================================================
Eigen::VectorXd toEigen(const ParabolicRamp::Vector &_x) {
  Eigen::VectorXd output(_x.size());

  for (std::size_t i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

//==============================================================================
void evaluateAtTime(const ParabolicRamp::DynamicPath &_path,
                    double _t,
                    Eigen::VectorXd &_position,
                    Eigen::VectorXd &_velocity) {
  ParabolicRamp::Vector positionVector;
  _path.Evaluate(_t, positionVector);
  _position = toEigen(positionVector);

  ParabolicRamp::Vector velocityVector;
  _path.Derivative(_t, velocityVector);
  _velocity = toEigen(velocityVector);
}
}
}
}
}
