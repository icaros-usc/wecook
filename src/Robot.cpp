//
// Created by hejia on 7/30/19.
//

#include <iostream>

#include "wecook/Robot.h"

using namespace wecook;

int Robot::execute(Action &action) {
  m_action.emplace_back(action);
}

void Robot::stop() {
  m_isEnd = true;
  m_thread.join();
}

void Robot::run() {
  while (!m_isEnd) {
    if (!m_action.empty()) {
      m_isFree = false;
      Action action = m_action[0];

      // action execution
//      std::cout << action.get_pid() << " "
//                << action.get_verb() << " "
//                << action.get_tool() << " "
//                << action.get_location() << " "
//                << action.get_ingredients()[0] << std::endl;
      m_action.pop_back();
    }
    m_isFree = true;
  }
}

aikido::trajectory::TrajectoryPtr Robot::planToTSR(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space,
                                                   const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
                                                   const dart::dynamics::BodyNodePtr &bn,
                                                   const aikido::constraint::dart::TSRPtr &tsr,
                                                   const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                                                   double timelimit,
                                                   size_t maxNumTrails) {
  m_ada->planToTSR(space,
                   metaSkeleton,
                   bn,
                   tsr,
                   collisionFree,
                   timelimit,
                   maxNumTrails);
}

void Robot::closeHand() {
  auto handSkeleton = m_ada->getHand()->getMetaSkeleton();
  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());

  auto goalState = handSpace->createState();

  Eigen::Vector2d preshape;
  preshape << 0.75, 0.75;
  handSpace->convertPositionsToState(preshape, goalState);

  auto trajectory = m_ada->planToConfiguration(handSpace,
                                               handSkeleton,
                                               goalState,
                                               nullptr,
                                               1.0);
  auto future = m_ada->executeTrajectory(trajectory);
  future.wait();
}

