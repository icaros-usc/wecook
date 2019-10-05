//
// Created by hejia on 9/17/19.
//
#include <set>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/common/VanDerCorput.hpp>
#include <ros/duration.h>

#include "aikido/common/Spline.hpp"
#include "dart/dynamics/MetaSkeleton.hpp"

#include "SmootherUtil.h"
#include "ai/external/hauser_parabolic_smoother/HauserUtil.h"

using CubicSplineProblem = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;
using SegmentSplineProblem = aikido::common::SplineProblem<double, int, 2, Eigen::Dynamic, Eigen::Dynamic>;

namespace wecook {
namespace planner {
namespace detail {

class SimpleFeasibilityCheckerBase {
 public:
  virtual ~SimpleFeasibilityCheckerBase() {}
  virtual bool ConfigFeasible(const Eigen::VectorXd &x) = 0;
  virtual bool SegmentFeasible(const Eigen::VectorXd &a, const Eigen::VectorXd &b) = 0;
};

class SimpleSmootherFeasibilityCheckerBase
    : public SimpleFeasibilityCheckerBase {
 public:
  SimpleSmootherFeasibilityCheckerBase(
      aikido::constraint::TestablePtr testable, double checkResolution)
      : mTestable(std::move(testable)),
        mCheckResolution(checkResolution),
        mStateSpace(mTestable->getStateSpace()),
        mInterpolator(mStateSpace) {
    // Do nothing
  }

  bool ConfigFeasible(const Eigen::VectorXd &eigX) override {
    auto state = mStateSpace->createState();
    mStateSpace->expMap(eigX, state);
    return mTestable->isSatisfied(state);
  }

  bool SegmentFeasible(const Eigen::VectorXd &eigA, const Eigen::VectorXd &eigB) override {
    auto testState = mStateSpace->createState();
    auto startState = mStateSpace->createState();
    auto goalState = mStateSpace->createState();
    mStateSpace->expMap(eigA, startState);
    mStateSpace->expMap(eigB, goalState);

    auto dist = (eigA - eigB).norm();
    auto checkRes = 0.005 / dist;
    aikido::common::VanDerCorput vdc{1, false, false, checkRes};

    for (const auto alpha : vdc) {
      mInterpolator.interpolate(startState, goalState, alpha, testState);
      if (!mTestable->isSatisfied(testState)) {
        Eigen::VectorXd testV;
        mStateSpace->logMap(testState, testV);
        return false;
      }
    }
    return true;
  }

 private:
  aikido::constraint::TestablePtr mTestable;
  double mCheckResolution;
  aikido::statespace::ConstStateSpacePtr mStateSpace;
  aikido::statespace::GeodesicInterpolator mInterpolator;
};

class SimpleFeasibilityChecker {
 public:
  SimpleFeasibilityChecker(SimpleFeasibilityCheckerBase *feas, double tol) : feas(feas), tol(tol), maxiters(0) {

  }
  bool Check(const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
    return feas->SegmentFeasible(a, b);
  }

  SimpleFeasibilityCheckerBase *feas;
  double tol;
  int maxiters;
};

class HauserSmootherFeasibilityCheckerBase
    : public ParabolicRamp::FeasibilityCheckerBase {
 public:
  HauserSmootherFeasibilityCheckerBase(aikido::constraint::TestablePtr testable, double checkResolution)
      : mTestable(std::move(testable)),
        mCheckResolution(checkResolution),
        mStateSpace(mTestable->getStateSpace()),
        mInterpolator(mStateSpace) {
    // Do nothing
  }

  bool ConfigFeasible(const ParabolicRamp::Vector& x) override {
    Eigen::VectorXd eigX = ::wecook::ai::external::hauser_parabolic_smoother::toEigen(x);
    auto state = mStateSpace->createState();
    mStateSpace->expMap(eigX, state);
    return mTestable->isSatisfied(state);
  }

  bool SegmentFeasible(const ParabolicRamp::Vector& a, const ParabolicRamp::Vector& b) override {
    Eigen::VectorXd eigA = ::wecook::ai::external::hauser_parabolic_smoother::toEigen(a);
    Eigen::VectorXd eigB = ::wecook::ai::external::hauser_parabolic_smoother::toEigen(b);

    auto testState = mStateSpace->createState();
    auto startState = mStateSpace->createState();
    auto goalState = mStateSpace->createState();
    mStateSpace->expMap(eigA, startState);
    mStateSpace->expMap(eigB, goalState);

    auto dist = (eigA - eigB).norm();
    auto checkRes = 0.05 / dist;

    // both ends of the segment have already been checked by calling
    // ConfigFeasible(),
    // thus it is no longer needed to check in SegmentFeasible()
    aikido::common::VanDerCorput vdc{1, false, false, checkRes};

    for (const auto alpha : vdc)
    {
      mInterpolator.interpolate(startState, goalState, alpha, testState);
      if (!mTestable->isSatisfied(testState))
      {
        return false;
      }
    }
    return true;
  }

 private:
  aikido::constraint::TestablePtr mTestable;
  double mCheckResolution;
  aikido::statespace::ConstStateSpacePtr mStateSpace;
  aikido::statespace::GeodesicInterpolator mInterpolator;
};

/*
 * Utility functions
 */

std::unique_ptr<SimpleDynamicPath> InterpolatedToSimpleDynamicPath(const aikido::trajectory::Interpolated &inputInterpolated,
                                                                   const Eigen::VectorXd &maxVelocity,
                                                                   const Eigen::VectorXd &maxAcceleration,
                                                                   const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                   const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                   bool preserveWaypointVelocity) {
  auto stateSpace = inputInterpolated.getStateSpace();
  const auto numWaypoints = inputInterpolated.getNumWaypoints();

  std::vector<Eigen::VectorXd> milestones;
  std::vector<Eigen::VectorXd> velocities;
  milestones.reserve(numWaypoints);
  velocities.reserve(numWaypoints);

  Eigen::VectorXd tangentVector, currVec;

  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint) {
    auto currentState = inputInterpolated.getWaypoint(iwaypoint);

    stateSpace->logMap(currentState, currVec);
    milestones.emplace_back(currVec);

    // TODO check later,
    // for now we are not using velocity
//    inputInterpolated.getWaypointDerivative(iwaypoint, 1, tangentVector);
//    velocities.emplace_back(tangentVector);
  }

  auto outputPath = std::make_unique<SimpleDynamicPath>();
  outputPath->Init(maxVelocity, maxAcceleration);
  if (preserveWaypointVelocity) {
    outputPath->SetMilestones(milestones, velocities);
  } else {
    outputPath->SetMilestones(milestones);
  }
//  if (!outputPath->IsValid())
//    throw std::runtime_error("Converted DynamicPath is not valid");
  return outputPath;
}

std::shared_ptr<ParabolicRamp::DynamicPath> InterpolatedToHauserDynamicPath(const aikido::trajectory::Interpolated &inputInterpolated,
                                                                            const Eigen::VectorXd &maxVelocity,
                                                                            const Eigen::VectorXd &maxAcceleration,
                                                                            const aikido::statespace::dart::MetaSkeletonStateSpacePtr &metaStateSpace,
                                                                            const dart::dynamics::MetaSkeletonPtr &metaSkeletonPtr,
                                                                            bool preserveWaypointVelocity) {
  auto stateSpace = inputInterpolated.getStateSpace();
  const auto numWaypoints = inputInterpolated.getNumWaypoints();

  std::vector<ParabolicRamp::Vector> milestones;
  std::vector<ParabolicRamp::Vector> velocities;
  milestones.reserve(numWaypoints);
  velocities.reserve(numWaypoints);

  Eigen::VectorXd tangentVector, currVec;

  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint) {
//    auto currentState = stateSpace->createState();
    auto currentState = inputInterpolated.getWaypoint(iwaypoint);

    stateSpace->logMap(currentState, currVec);
    milestones.emplace_back(::wecook::ai::external::hauser_parabolic_smoother::toVector(currVec));

//    inputInterpolated.getWaypointDerivative(iwaypoint, 1, tangentVector);
//    velocities.emplace_back(tangentVector);
  }

  auto outputPath = std::make_shared<ParabolicRamp::DynamicPath>();
  outputPath->Init(::wecook::ai::external::hauser_parabolic_smoother::toVector(maxVelocity), ::wecook::ai::external::hauser_parabolic_smoother::toVector(maxAcceleration));
  if (preserveWaypointVelocity) {
    outputPath->SetMilestones(milestones, velocities);
  } else {
    outputPath->SetMilestones(milestones);
  }
//  if (!outputPath->IsValid())
//    throw std::runtime_error("Converted DynamicPath is not valid");
  return outputPath;
}

std::unique_ptr<aikido::trajectory::Spline> HauserDynamicPathToSpline(const ParabolicRamp::DynamicPath &_inputPath,
                                                                      double _startTime,
                                                                      const aikido::statespace::ConstStateSpacePtr &_stateSpace) {
  std::cout << "HauserDynamicPathToSpline: " << std::endl;
  const auto dimension = _stateSpace->getDimension();

  // Construct a list of all ramp transition points.
  double t = 0.;
  std::set<double> transitionTimes;
  transitionTimes.insert(t);

  for (const auto &rampNd : _inputPath.ramps) {
    for (const auto &ramp1d : rampNd.ramps) {
      transitionTimes.insert(t + ramp1d.tswitch1);
      transitionTimes.insert(t + ramp1d.tswitch2);
    }

    t += rampNd.endTime;
    transitionTimes.insert(t);
  }

  // Convert the output to a spline with a knot at each transition time.
  assert(!transitionTimes.empty());
  const auto startIt = std::begin(transitionTimes);
  double timePrev = *startIt;
  transitionTimes.erase(startIt);

  Eigen::VectorXd positionPrev, velocityPrev;
  ::wecook::ai::external::hauser_parabolic_smoother::evaluateAtTime(_inputPath, timePrev, positionPrev, velocityPrev);

  auto _outputTrajectory = std::make_unique<aikido::trajectory::Spline>(_stateSpace, timePrev + _startTime);
  auto segmentStartState = _stateSpace->createState();

  for (const auto timeCurr : transitionTimes) {
    Eigen::VectorXd positionCurr, velocityCurr;
    ::wecook::ai::external::hauser_parabolic_smoother::evaluateAtTime(_inputPath, timeCurr, positionCurr, velocityCurr);

    SegmentSplineProblem problem(Eigen::Vector2d(0, timeCurr - timePrev), 2, dimension);
//    CubicSplineProblem problem(Eigen::Vector2d(0, timeCurr - timePrev), 4, dimension);
    problem.addConstantConstraint(0, 0, Eigen::VectorXd::Zero(dimension));
//    problem.addConstantConstraint(0, 1, velocityPrev);
    problem.addConstantConstraint(1, 0, positionCurr - positionPrev);
//    problem.addConstantConstraint(1, 1, velocityCurr);
    const auto spline = problem.fit();

    _stateSpace->expMap(positionPrev, segmentStartState);

    // Add the ramp to the output trajectory
    assert(spline.getCoefficients().size() == 1);
    const auto &coefficients = spline.getCoefficients().front();
    _outputTrajectory->addSegment(
        coefficients, timeCurr - timePrev, segmentStartState);

    timePrev = timeCurr;
    positionPrev = positionCurr;
    velocityPrev = velocityCurr;
  }
  return _outputTrajectory;
}

aikido::trajectory::InterpolatedPtr HauserDynamicPathToInterpolated(const ParabolicRamp::DynamicPath &_inputPath,
                                                                    double _startTime,
                                                                    const aikido::statespace::ConstStateSpacePtr &_stateSpace) {
  /*
   * Used for debugging
   */
  std::shared_ptr<aikido::statespace::GeodesicInterpolator>
      interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(_stateSpace);
  auto traj = std::make_shared<aikido::trajectory::Interpolated>(_stateSpace, interpolator);

  // Construct a list of all ramp transition points.
  double t = 0.;
  std::set<double> transitionTimes;
  transitionTimes.insert(t);

  for (const auto &rampNd : _inputPath.ramps) {
    for (const auto &ramp1d : rampNd.ramps) {
      transitionTimes.insert(t + ramp1d.tswitch1);
      transitionTimes.insert(t + ramp1d.tswitch2);
    }

    t += rampNd.endTime;
    transitionTimes.insert(t);
  }

  // Convert the output to a spline with a knot at each transition time.
  assert(!transitionTimes.empty());

  for (const auto timeCurr : transitionTimes) {
    Eigen::VectorXd positionCurr, velocityCurr;
    ::wecook::ai::external::hauser_parabolic_smoother::evaluateAtTime(_inputPath, timeCurr, positionCurr, velocityCurr);
    auto currState = _stateSpace->createState();
    _stateSpace->expMap(positionCurr, currState);
    traj->addWaypoint(timeCurr, currState);
  }
  return traj;
}

aikido::trajectory::InterpolatedPtr SimpleDynamicPathToInterpolated(const SimpleDynamicPath &_inputPath,
                                                                    const aikido::statespace::ConstStateSpacePtr &_stateSpace) {
  std::shared_ptr<aikido::statespace::GeodesicInterpolator>
      interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(_stateSpace);
  auto traj = std::make_shared<aikido::trajectory::Interpolated>(_stateSpace, interpolator);

  auto waypoints = _inputPath.getWaypoints();

  // Convert the output to a spline with a knot at each transition time.
  assert(!waypoints.empty());

  auto segmentStartState = _stateSpace->createState();

  double timeCurr = 0.;
  double delta = 0.2;
  for (const auto waypoint : waypoints) {
    _stateSpace->expMap(waypoint, segmentStartState);

    traj->addWaypoint(timeCurr, segmentStartState);

    timeCurr += delta;
  }

  return traj;
}

aikido::trajectory::Interpolated *SplineToInterpolated(const aikido::trajectory::Spline &inputSpline) {
  auto stateSpace = inputSpline.getStateSpace();
  const auto numWaypoints = inputSpline.getNumWaypoints();

  std::shared_ptr<aikido::statespace::GeodesicInterpolator>
      interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);
  auto traj = std::make_shared<aikido::trajectory::Interpolated>(stateSpace, interpolator);

  double timeCurr = 0.;
  double delta = 0.2;
  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint) {
    auto currentState = stateSpace->createState();
    inputSpline.getWaypoint(iwaypoint, currentState);
    traj->addWaypoint(timeCurr, currentState);
    timeCurr += delta;
  }

  return traj.get();
}

void hauserDoShortcut(ParabolicRamp::DynamicPath &hauserDynamicPath,
                      aikido::constraint::TestablePtr testable,
                      double timelimit,
                      double checkResolution,
                      double tolerance,
                      aikido::common::RNG &rng) {
  if (timelimit < 0.0)
    throw std::invalid_argument("Timelimit should be non-negative");
  if (checkResolution <= 0.0)
    throw std::invalid_argument("Check resolution should be positive");
  if (tolerance < 0.0)
    throw std::invalid_argument("Tolerance should be non-negative");

  HauserSmootherFeasibilityCheckerBase base(testable, checkResolution);
  ParabolicRamp::RampFeasibilityChecker feasibilityChecker(&base, tolerance);

  std::chrono::time_point<std::chrono::system_clock> startTime
      = std::chrono::system_clock::now();
  double elapsedTime = 0;

  while (elapsedTime < timelimit && hauserDynamicPath.ramps.size() > 3) {
    std::uniform_real_distribution<> dist(0.0, hauserDynamicPath.GetTotalTime());
    double t1 = dist(rng);
    double t2 = dist(rng);
    if (hauserDynamicPath.TryShortcut(t1, t2, feasibilityChecker)) {
    }

    elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::system_clock::now() - startTime)
        .count();
  }
}

void simpleDoShortcut(SimpleDynamicPath &dynamicPath,
                      aikido::constraint::TestablePtr testable,
                      double timelimit,
                      double checkResolution,
                      double tolerance,
                      aikido::common::RNG &rng) {
  if (timelimit < 0.0)
    throw std::invalid_argument("Timelimit should be non-negative");
  if (checkResolution <= 0.0)
    throw std::invalid_argument("Check resolution should be positive");
  if (tolerance < 0.0)
    throw std::invalid_argument("Tolerance should be non-negative");

  SimpleSmootherFeasibilityCheckerBase base(testable, checkResolution);
  SimpleFeasibilityChecker feasibilityChecker(&base, tolerance);

  std::chrono::time_point<std::chrono::system_clock> startTime
      = std::chrono::system_clock::now();
  double elapsedTime = 0;

  while (elapsedTime < timelimit && dynamicPath.getWaypoints().size() > 3) {
    // closed interval
    std::uniform_int_distribution<> dist(0, dynamicPath.getWaypoints().size() - 1);
    int i1 = dist(rng);
    int i2 = dist(rng);

    if (i1 > i2) {
      auto tmp = i1;
      i1 = i2;
      i2 = tmp;
    }

    if (i1 == i2 || i1 - i2 == -1) {
      continue;
    }

    if (feasibilityChecker.Check(dynamicPath.getWaypoint(i1), dynamicPath.getWaypoint(i2))) {
      dynamicPath.Shortcut(i1, i2);
    }

    elapsedTime =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - startTime).count();
  }
}
}
}
}