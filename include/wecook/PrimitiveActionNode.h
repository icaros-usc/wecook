//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEACTIONNODE_H
#define WECOOK_PRIMITIVEACTIONNODE_H

#include "Action.h"
#include "MotionNode.h"

namespace wecook {

class PrimitiveActionNode {
 public:
  PrimitiveActionNode(const std::string &pid, const std::string &type, bool ifHeand = false)
      : m_ifHead(ifHeand), m_pid(pid), m_type(type) {

  }

  inline void setMotionSeq(const std::vector<std::shared_ptr<MotionNode>> &motionSeq) {
    m_motionSeq = motionSeq;
  }

  inline void addChild(std::shared_ptr<PrimitiveActionNode> child) {
    m_children.emplace_back(child);
  }

  inline void addFather(std::shared_ptr<PrimitiveActionNode> father) {
    m_fathers.emplace_back(father);
  }

  inline bool ifHead() {
    return m_ifHead;
  }

  inline std::string getPid() {
    return m_pid;
  }

 private:
  bool m_ifHead;
  std::string m_pid;
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_children;
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_fathers;
  std::vector<std::shared_ptr<MotionNode>> m_motionSeq;
  std::string m_type;
  std::string m_grabbingObj;
};

}

#endif //WECOOK_PRIMITIVEACTIONNODE_H
