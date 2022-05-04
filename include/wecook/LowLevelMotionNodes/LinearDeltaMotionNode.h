//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_LINEARDELTAMOTIONNODE_H
#define WECOOK_LINEARDELTAMOTIONNODE_H

#include <dart/dynamics/Frame.hpp>

#include "MotionNode.h"

namespace wecook {

class LinearDeltaMotionNode : public MotionNode {
 public:
  LinearDeltaMotionNode(dart::dynamics::BodyNode *bn,
                        const Eigen::VectorXd &delta_x,
                        dart::dynamics::Frame *incoordinatesOf,
                        double repeat_time,
                        const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                        const dart::dynamics::MetaSkeletonPtr &skeleton,
                        const aikido::constraint::dart::CollisionFreePtr &collisionFree) : MotionNode(stateSpace,
                                                                                                      skeleton),
                                                                                           m_bn(bn),
                                                                                           m_delta_x(delta_x),
                                                                                           m_incoordinatesOf(
                                                                                               incoordinatesOf),
                                                                                           m_repeat_time(repeat_time),
                                                                                           m_collisionFree(collisionFree) {

  }

  void plan(const std::shared_ptr<ada::Ada> &adaPlan, const std::shared_ptr<ada::Ada> &adaExec, Result *result = nullptr);

 private:
  Eigen::VectorXd m_delta_x;
  double m_repeat_time;
  dart::dynamics::BodyNode *m_bn = nullptr;
  dart::dynamics::Frame *m_incoordinatesOf = nullptr;
  aikido::constraint::dart::CollisionFreePtr m_collisionFree = nullptr;
};

}

#endif //WECOOK_LINEARDELTAMOTIONNODE_H
