//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEACTIONNODE_H
#define WECOOK_PRIMITIVEACTIONNODE_H

#include "Agent.h"
#include "Action.h"
#include "MotionNode.h"

namespace wecook {

class PrimitiveActionNode {
 public:
  PrimitiveActionNode(const std::string &pid,
                      const std::string &type,
                      const std::string &grabbingObj = "",
                      const std::string &placingObj = "",
                      bool ifHeand = false,
                      bool ifTail = false)
      : m_ifHead(ifHeand),
        m_pid(pid),
        m_grabbingObj(grabbingObj),
        m_placingObj(placingObj),
        m_type(type),
        m_ifTail(ifTail) {

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

  inline std::string getGrabbingObj() {
    return m_grabbingObj;
  }

  inline std::string getPlacingObj() {
    return m_placingObj;
  }

  inline bool ifTail() {
    return m_ifTail;
  }

  inline std::string getType() {
    return m_type;
  }

  inline void setIfTail(bool ifTail) {
    m_ifTail = ifTail;
  }

  inline void setIfHead(bool ifHead) {
    m_ifHead = ifHead;
  }

  inline void setIfExecuted(bool ifExecuted) {
    m_ifExecuted = ifExecuted;
  }

  inline bool ifExecuted() {
    return m_ifExecuted;
  }

  inline std::vector<std::shared_ptr<PrimitiveActionNode>> getFathers() {
    return m_fathers;
  }

  inline std::vector<std::shared_ptr<PrimitiveActionNode>> getChildren() {
    return m_children;
  }

  virtual void execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                       std::shared_ptr<ObjectMgr> &objMgr,
                       std::shared_ptr<ContainingMap> &containingMap) = 0;

  void removeChild(std::shared_ptr<PrimitiveActionNode> &child);

  void removeFather(std::shared_ptr<PrimitiveActionNode> &father);

 protected:
  bool m_ifTail;
  bool m_ifHead;
  bool m_ifExecuted = false;
  std::string m_pid;
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_children;
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_fathers;
  std::vector<std::shared_ptr<MotionNode>> m_motionSeq;
  std::string m_type;
  std::string m_grabbingObj;
  std::string m_placingObj;
};

}

#endif //WECOOK_PRIMITIVEACTIONNODE_H
