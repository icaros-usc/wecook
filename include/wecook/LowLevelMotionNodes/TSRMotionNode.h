//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_TSRMOTIONNODE_H
#define WECOOK_TSRMOTIONNODE_H

#include "MotionNode.h"

namespace wecook {

class TSRMotionNode : public MotionNode {
 public:
  TSRMotionNode(const aikido::constraint::dart::TSRPtr &goalTSR,
                dart::dynamics::BodyNode *bn,
                const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                const dart::dynamics::MetaSkeletonPtr &skeleton,
                bool debug = false) : MotionNode(stateSpace, skeleton),
                                      m_goalTSR(goalTSR),
                                      m_collisionFree(collisionFree),
                                      m_bn(bn),
                                      m_debug(debug) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result = nullptr);

  void setTimeStep(double timeStep) {
    m_retimeTimeStep = timeStep;
  }

 private:
  void computeIK(std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState> &configurations,
                 const aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState &startState);

  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const std::shared_ptr<ada::Ada> &ada,
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const aikido::statespace::StateSpace::State *startState,
      const aikido::statespace::StateSpace::State *goalState,
      const aikido::constraint::dart::CollisionFreePtr &collisionFree,
      aikido::planner::Planner::Result *pResult);

  void validate();

  aikido::trajectory::TrajectoryPtr smoothTrajectory(aikido::trajectory::TrajectoryPtr untimedTrajectory,
                                                     const std::shared_ptr<ada::Ada> &ada);

  void executeTrajectory(const aikido::trajectory::TrajectoryPtr &timedTrajectory,
                         const std::shared_ptr<ada::Ada> &ada,
                         const std::shared_ptr<ada::Ada> &adaImg,
                         const aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState &startState);

 private:
  aikido::constraint::dart::CollisionFreePtr m_collisionFree = nullptr;
  dart::dynamics::BodyNode *m_bn = nullptr;
  aikido::constraint::dart::TSRPtr m_goalTSR;
  bool m_debug;
  double m_retimeTimeStep = 0.02;
};

}

#endif //WECOOK_TSRMOTIONNODE_H
