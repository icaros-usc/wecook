//
// Created by hejia on 8/23/19.
//

#ifndef WECOOK_ACTIONNODE_H
#define WECOOK_ACTIONNODE_H

#include "Action.h"
#include "MotionNode.h"

namespace wecook {

class ActionNode {
 public:
  ActionNode(const Action& action, bool ifHead=false) : m_action(action), m_ifHead(ifHead) {}

  inline Action getAction() {
    return m_action;
  }

  inline std::vector<ActionNode *> getChildren() {
    return m_children;
  }

  inline void addChild(ActionNode *actionNode) {
    m_children.emplace_back(actionNode);
  }

  inline void addFather(ActionNode *actionNode) {
    m_fathers.emplace_back(actionNode);
  }

  inline void addMotionSeq(std::string &pid, std::vector<std::shared_ptr<MotionNode>> &motionSeq) {
    std::pair<std::string, std::vector<std::shared_ptr<MotionNode>>> motionSeqPair{pid, motionSeq};
    m_motionSeqMap.emplace(motionSeqPair);
  }

  inline std::map<std::string, std::vector<std::shared_ptr<MotionNode>>> getMotionSeqMap() {
    return m_motionSeqMap;
  }

  std::map<std::string, std::vector<std::shared_ptr<MotionNode>>> m_motionSeqMap;
 private:
  Action m_action;
  bool m_ifHead;
  std::vector<ActionNode *> m_children;
  std::vector<ActionNode *> m_fathers;
};

}

#endif //WECOOK_ACTIONNODE_H
