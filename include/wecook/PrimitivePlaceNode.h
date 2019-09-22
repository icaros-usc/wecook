//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEPLACENODE_H
#define WECOOK_PRIMITIVEPLACENODE_H

#include "PrimitiveActionNode.h"
#include "ContainingMap.h"

namespace wecook {

class PrimitivePlaceNode : public PrimitiveActionNode {
 public:
  PrimitivePlaceNode(const aikido::constraint::dart::TSRPtr &targetPose,
                     const std::string &toPlace,
                     const std::string &refObject,
                     const std::string &pid,
                     const std::string &grabbingObj,
                     const std::string &placingObj,
                     bool ifHead,
                     bool ifTail,
                     bool ifDebug = false)
      : PrimitiveActionNode(pid, "place", grabbingObj, placingObj, ifHead, ifTail),
        m_refObject(refObject),
        m_targetPose(targetPose),
        m_toPlace(toPlace),
        m_ifDebug(ifDebug) {

  }

  void execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
               std::shared_ptr<ObjectMgr> &objMgr,
               std::shared_ptr<ContainingMap> &containingMap);

 private:
  bool m_ifDebug;
  std::string m_toPlace;
  aikido::constraint::dart::TSRPtr m_targetPose;
  std::string m_refObject;
};

}

#endif //WECOOK_PRIMITIVEPLACENODE_H
