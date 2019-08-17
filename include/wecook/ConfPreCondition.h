//
// Created by hejia on 8/13/19.
//

#ifndef WECOOK_CONFPRECONDITION_H
#define WECOOK_CONFPRECONDITION_H

#include <Eigen/Core>
#include <dart/dynamics/MetaSkeleton.hpp>

#include "PreCondition.h"



class ConfPreCondition : public PreCondition {
 public:
  ConfPreCondition(const Eigen::VectorXd &conf, const dart::dynamics::MetaSkeletonPtr &skeleton) :
      m_goalConf(conf), m_skeleton(skeleton) {

  }

  bool isSatisfied() {
    return m_goalConf == m_skeleton->getPositions();
  }

 private:
  Eigen::VectorXd m_goalConf;
  dart::dynamics::MetaSkeletonPtr m_skeleton;
};

#endif //WECOOK_CONFPRECONDITION_H
