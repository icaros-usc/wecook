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
  ActionNode(const Action &action, const std::vector<std::string> &pids, bool ifHead = false)
      : m_action(action), m_pids(pids), m_ifHead(ifHead) {}

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

  inline std::vector<std::string> getPids() {
    return m_pids;
  }

  inline void addMotionSeq(std::string &pid, std::vector<std::shared_ptr<MotionNode>> &motionSeq) {
    std::pair<std::string, std::vector<std::shared_ptr<MotionNode>>> motionSeqPair{pid, motionSeq};
    m_motionSeqMap.emplace(motionSeqPair);
  }

  inline std::map<std::string, std::vector<std::shared_ptr<MotionNode>>> getMotionSeqMap() {
    return m_motionSeqMap;
  }

  inline void setPrimitiveTaskGraph(const PrimitiveTaskGraph &ptg) {
    m_primitiveTaskGraph = ptg;
  }

  std::map<std::string, std::vector<std::shared_ptr<MotionNode>>> m_motionSeqMap;

  PrimitiveTaskGraph m_primitiveTaskGraph;
 private:
  Action m_action;
  bool m_ifHead;
  std::vector<ActionNode *> m_children;
  std::vector<ActionNode *> m_fathers;
  std::vector<std::string> m_pids;
};

}

#endif //WECOOK_ACTIONNODE_H
