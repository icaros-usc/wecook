//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEGRABNODE_H
#define WECOOK_PRIMITIVEGRABNODE_H

#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include "PrimitiveActionNode.h"

namespace wecook {

class PrimitiveGrabNode : public PrimitiveActionNode {
 public:
  PrimitiveGrabNode(const aikido::constraint::dart::TSRPtr &grabPose,
                    const std::string &toGrab,
                    dart::dynamics::BodyNode *refBodyNode,
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
                                           m_refBodyNode(refBodyNode) {
  }

  void execute();

 private:
  std::string m_toGrab;
  aikido::constraint::dart::TSRPtr m_grabPose;
  dart::dynamics::BodyNode *m_refBodyNode;

};

}

#endif //WECOOK_PRIMITIVEGRABNODE_H