//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEGRABNODE_H
#define WECOOK_PRIMITIVEGRABNODE_H

#include <aikido/constraint/dart/TSR.hpp>

#include "PrimitiveActionNode.h"
#include "ContainingMap.h"

namespace wecook {

class PrimitiveGrabNode : public PrimitiveActionNode {
 public:
  PrimitiveGrabNode(const aikido::constraint::dart::TSRPtr &grabPose,
                    const std::string &toGrab,
                    const std::string &refObject,
                    const std::string &pid,
                    const std::string &grabbingObj,
                    const std::string &placingObj,
                    bool ifHead = false,
                    bool ifTail = false) : PrimitiveActionNode(pid,
                                                               "grab",
                                                               grabbingObj,
                                                               placingObj,
                                                               ifHead,
                                                               ifTail),
                                           m_toGrab(toGrab),
                                           m_grabPose(grabPose),
                                           m_refObject(refObject) {
  }

  void execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
               std::shared_ptr<ObjectMgr> &objMgr,
               std::shared_ptr<ContainingMap> &containingMap);

 private:
  std::string m_toGrab;
  aikido::constraint::dart::TSRPtr m_grabPose;
  std::string m_refObject;

};

}

#endif //WECOOK_PRIMITIVEGRABNODE_H
