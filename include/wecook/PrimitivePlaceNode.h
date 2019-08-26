//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEPLACENODE_H
#define WECOOK_PRIMITIVEPLACENODE_H

#include "PrimitiveActionNode.h"

namespace wecook {

class PrimitivePlaceNode : public PrimitiveActionNode {
 public:
  PrimitivePlaceNode(const aikido::constraint::dart::TSRPtr &targetPose,
                     const std::string &toPlace,
                     dart::dynamics::BodyNode *refBodyNode,
                     dart::dynamics::BodyNode *toPlaceBodyNode,
                     const std::string &pid,
                     const std::string &grabbingObj,
                     const std::string &placingObj,
                     bool ifHead = false,
                     bool ifTail = false)
      : PrimitiveActionNode(pid, "place", grabbingObj, placingObj, ifHead, ifTail),
        m_refBodyNode(refBodyNode),
        m_toPlaceBodyNode(toPlaceBodyNode),
        m_targetPose(targetPose),
        m_toPlace(toPlace) {

  }

  void execute();

 private:
  std::string m_toPlace;
  aikido::constraint::dart::TSRPtr m_targetPose;
  dart::dynamics::BodyNode *m_refBodyNode;
  dart::dynamics::BodyNode *m_toPlaceBodyNode;
};

}

#endif //WECOOK_PRIMITIVEPLACENODE_H
