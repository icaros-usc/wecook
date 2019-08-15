//
// Created by hejia on 8/13/19.
//

#ifndef WECOOK_TSRPRECONDITION_H
#define WECOOK_TSRPRECONDITION_H

#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <aikido/statespace/SE3.hpp>

#include "PreCondition.h"

class TSRPreCondition : public PreCondition {
 public:
  TSRPreCondition(const aikido::constraint::dart::TSRPtr &targetTSR,
                  const dart::dynamics::BodyNodePtr &bodyNode)
      : m_targetTSR(targetTSR), m_bodyNode(bodyNode) {

  }

  bool isSatisfied() {
    auto transform = m_bodyNode->getWorldTransform();
    auto stateSpace = aikido::statespace::SE3();
    auto state = stateSpace.createState();
    state.setIsometry(transform);
    auto satisfied = m_targetTSR->isSatisfied(state);
    if (satisfied) {
      // test again;
      ros::Duration(2.0).sleep();
      satisfied = m_targetTSR->isSatisfied(state);
    }
    return satisfied;
  }

 private:
  aikido::constraint::dart::TSRPtr m_targetTSR;
  dart::dynamics::BodyNodePtr m_bodyNode;

};

#endif //WECOOK_TSRPRECONDITION_H
