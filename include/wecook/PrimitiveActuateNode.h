//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEACTUATENODE_H
#define WECOOK_PRIMITIVEACTUATENODE_H

#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include "PrimitiveActionNode.h"
#include "ContainingMap.h"

namespace wecook {

class PrimitiveActuateNode : public PrimitiveActionNode {
 public:
  PrimitiveActuateNode(const std::string &pid,
                       const std::string &manipulatedObj,
                       const std::string &motionType,
                       const std::string &grabbingObj,
                       const std::string &placingObj,
                       bool ifHead = false,
                       bool ifTail = false)
      : PrimitiveActionNode(pid,
                            "predefined",
                            grabbingObj,
                            placingObj,
                            ifHead,
                            ifTail), m_motionType(motionType), m_manipulatedObj(manipulatedObj) {

  }

  void execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
               std::shared_ptr<ObjectMgr> &objMgr,
               std::shared_ptr<ContainingMap> &containingMap);

 private:
  std::string m_motionType;
  std::string m_manipulatedObj;
};

}

#endif //WECOOK_PRIMITIVEACTUATENODE_H
