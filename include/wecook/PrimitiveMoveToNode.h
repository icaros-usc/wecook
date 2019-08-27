//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEMOVETONODE_H
#define WECOOK_PRIMITIVEMOVETONODE_H

#include <aikido/constraint/dart/TSR.hpp>

#include "PrimitiveActionNode.h"
#include "ContainingMap.h"

namespace wecook {

class PrimitiveMoveToNode : public PrimitiveActionNode {
 public:
  PrimitiveMoveToNode(const aikido::constraint::dart::TSRPtr &targetPose,
                      const std::string &toMove,
                      const std::string &refObject,
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
                                             m_refObject(refObject),
                                             m_targetPose(targetPose),
                                             m_toMove(toMove) {
  }

  void execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
               std::shared_ptr<ObjectMgr> &objMgr,
               std::shared_ptr<ContainingMap> &containingMap);

 private:
  std::string m_toMove;
  aikido::constraint::dart::TSRPtr m_targetPose;
  std::string m_refObject;
};

}

#endif //WECOOK_PRIMITIVEMOVETONODE_H
