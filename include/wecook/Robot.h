//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_ROBOT_H
#define WECOOK_ROBOT_H

#include <boost/thread.hpp>
#include <libada/Ada.hpp>
#include <Eigen/Dense>

#include "Action.h"

namespace wecook {
class Robot {
 public:
  Robot(const Eigen::Isometry3d &transform,
        const std::string &name) : m_thread(&Robot::run, this), m_transform(transform), m_name(name) {

  }

  int execute(Action &action);

  void stop();

  void closeHand();

  inline bool isFree() {
    return m_isFree;
  }

  inline bool isEnd() {
    return m_isEnd;
  }

  inline aikido::planner::WorldPtr getWorld() {
    return m_ada->getWorld();
  }

  inline auto getArm() {
    return m_ada->getArm();
  }

  inline auto getHand() {
    return m_ada->getHand();
  }

  inline auto getStateSpace() {
    return m_ada->getStateSpace();
  }

  inline auto getMetaSkeleton() {
    return m_ada->getMetaSkeleton();
  }

  inline auto executeTrajectory(aikido::trajectory::TrajectoryPtr traj) {
    return m_ada->executeTrajectory(traj);
  }

  inline auto retimePath(const dart::dynamics::MetaSkeletonPtr &armSkeleton, aikido::trajectory::Trajectory *traj) {
    return m_ada->retimePath(armSkeleton, traj);
  }

  void createAda(const aikido::planner::WorldPtr &env) {
    m_ada = std::make_shared<ada::Ada>(env, true, m_name, m_transform);
//    auto robotSkeleton = m_ada->getMetaSkeleton();
//    dynamic_cast<dart::dynamics::FreeJoint *>(robotSkeleton->getJoint(0))->setTransform(m_transform);
  }

  aikido::trajectory::TrajectoryPtr planToTSR(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &bn,
      const aikido::constraint::dart::TSRPtr &tsr,
      const aikido::constraint::dart::CollisionFreePtr &collisionFree,
      double timelimit,
      size_t maxNumTrails);

  std::shared_ptr<ada::Ada> m_ada = nullptr;

 private:
  void run();

  std::string m_name;

  boost::thread m_thread;
  bool m_isFree = true;
  bool m_isEnd = false;
  std::vector<Action> m_action;
  Eigen::Isometry3d m_transform;
};
}
#endif //WECOOK_ROBOT_H
