//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEPLACENODE_H
#define WECOOK_PRIMITIVEPLACENODE_H

#include "Action.h"
#include "MotionNode.h"

namespace wecook {

class PrimitivePlaceNode : public PrimitiveActionNode {
 public:
  PrimitivePlaceNode(const aikido::constraint::dart::TSRPtr &targetPose,
                     const std::string &toPlace,
                     dart::dynamics::BodyNode *refBodyNode,
                     dart::dynamics::BodyNode *toPlaceBodyNode,
                     const std::string &pid, bool ifHead = false) : PrimitiveActionNode(pid, "place", ifHead),
                                                                    m_refBodyNode(refBodyNode),
                                                                    m_toPlaceBodyNode(toPlaceBodyNode),
                                                                    m_targetPose(targetPose),
                                                                    m_toPlace(toPlace) {

  }

 private:
  std::string m_toPlace;
  aikido::constraint::dart::TSRPtr m_targetPose;
  dart::dynamics::BodyNode *m_refBodyNode;
  dart::dynamics::BodyNode *m_toPlaceBodyNode;
};

}

#endif //WECOOK_PRIMITIVEPLACENODE_H
