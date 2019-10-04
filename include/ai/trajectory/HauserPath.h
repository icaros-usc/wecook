//
// Created by hejia on 9/22/19.
//

#ifndef WECOOK_HAUSERPATH_H
#define WECOOK_HAUSERPATH_H

#include <aikido/trajectory/Trajectory.hpp>
#include <ai/external/hauser_parabolic_smoother/DynamicPath.h>

namespace wecook {
namespace ai {
namespace trajectory {

/// Wrapper for hauser's dynamic path
class HauserPath : public aikido::trajectory::Trajectory {
 public:
  HauserPath(ParabolicRamp::DynamicPath *dynamicPath, aikido::statespace::ConstStateSpacePtr stateSpace);
  virtual ~HauserPath();

  // Documentation inherited
  aikido::statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited
  std::size_t getNumDerivatives() const override;

  // Documentation inherited
  double getDuration() const override;

  // Documentation inherited
  double getStartTime() const override;

  // Documentation inherited
  double getEndTime() const override;

  // Documentation inherited
  void evaluate(double t, aikido::statespace::StateSpace::State *state) const override;

  // Documentation inherited
  void evaluateDerivative(double t, int derivative, Eigen::VectorXd &tangentVector) const override;

 private:
  std::unique_ptr<ParabolicRamp::DynamicPath> m_dynamicPath;
  aikido::statespace::ConstStateSpacePtr m_stateSpace;

};

}
}
}

#endif //WECOOK_HAUSERPATH_H
