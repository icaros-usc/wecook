//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEMOVETONODE_H
#define WECOOK_PRIMITIVEMOVETONODE_H

#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include "PrimitiveActionNode.h"

namespace wecook {

class PrimitiveMoveToNode : public PrimitiveActionNode {
 public:
  PrimitiveMoveToNode(const aikido::constraint::dart::TSRPtr &targetPose,
                      const std::string &toMove,
                      dart::dynamics::BodyNode *refBodyNode,
                      dart::dynamics::BodyNode *toMoveBodyNode,
                      const std::string &pid,
                      const std::string &grabbingObj,
                      const std::string &placingObj,
                      bool ifHead = false,
                      bool ifTail = false) : PrimitiveActionNode(pid,
                                                                 "moveto",
                                                                 grabbingObj,
                                                                 placingObj,
                                                                 ifHead,
                                                                 ifTail),
                                             m_refBodyNode(refBodyNode),
                                             m_toMoveBodyNode(toMoveBodyNode),
                                             m_targetPose(targetPose),
                                             m_toMove(toMove) {
  }

  void execute();

 private:
  std::string m_toMove;
  aikido::constraint::dart::TSRPtr m_targetPose;
  dart::dynamics::BodyNode *m_refBodyNode;
  dart::dynamics::BodyNode *m_toMoveBodyNode;
};

}

#endif //WECOOK_PRIMITIVEMOVETONODE_H
