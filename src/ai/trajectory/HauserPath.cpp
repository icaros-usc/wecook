//
// Created by hejia on 9/22/19.
//
#include <iostream>

#include "ai/external/hauser_parabolic_smoother/HauserUtil.h"
#include "ai/trajectory/HauserPath.h"

using namespace wecook::ai::trajectory;

HauserPath::HauserPath(std::shared_ptr<ParabolicRamp::DynamicPath> dynamicPath, aikido::statespace::ConstStateSpacePtr stateSpace)
    : m_dynamicPath(dynamicPath), m_stateSpace(stateSpace) {

}

HauserPath::~HauserPath() {

}

aikido::statespace::ConstStateSpacePtr HauserPath::getStateSpace() const {
  return m_stateSpace;
}

std::size_t HauserPath::getNumDerivatives() const {
  return m_dynamicPath->GetNumDerivatives();
}

double HauserPath::getDuration() const {
  return m_dynamicPath->GetTotalTime();
}

double HauserPath::getStartTime() const {
  return m_dynamicPath->GetStartTime();
}

double HauserPath::getEndTime() const {
  return m_dynamicPath->GetEndTime();
}

void HauserPath::evaluate(double t, aikido::statespace::StateSpace::State *state) const {
  ParabolicRamp::Vector positionVector;
  m_dynamicPath->Evaluate(t, positionVector);
  Eigen::VectorXd eigA;
  eigA = ::wecook::ai::external::hauser_parabolic_smoother::toEigen(positionVector);
  m_stateSpace->expMap(eigA, state);
}

void HauserPath::evaluateDerivative(double t, int derivative, Eigen::VectorXd &tangentVector) const {
  if (derivative != 1) {
    std::__throw_invalid_argument("HauserPath::evaluateDerivative: derivative could only be 1!");
  }
  ParabolicRamp::Vector velocityVector;
  m_dynamicPath->Derivative(t, velocityVector);
  tangentVector = ::wecook::ai::external::hauser_parabolic_smoother::toEigen(velocityVector);
}



