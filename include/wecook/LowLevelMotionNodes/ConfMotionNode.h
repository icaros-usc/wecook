//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_CONFMOTIONNODE_H
#define WECOOK_CONFMOTIONNODE_H

#include "MotionNode.h"

namespace wecook {

class ConfMotionNode : public MotionNode {
 public:
  ConfMotionNode(const Eigen::VectorXd &goalConf,
                 const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                 const dart::dynamics::MetaSkeletonPtr &skeleton) : MotionNode(stateSpace,
                                                                               skeleton),
                                                                    m_goalConf(goalConf) {
  }

  void plan(const std::shared_ptr<ada::Ada> &ada, const std::shared_ptr<ada::Ada> &adaImg, Result *result = nullptr);

 private:
  Eigen::VectorXd m_goalConf;
};
}

#endif //WECOOK_CONFMOTIONNODE_H
