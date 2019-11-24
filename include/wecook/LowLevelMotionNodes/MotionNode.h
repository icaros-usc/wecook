//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_MOTIONNODE_H
#define WECOOK_MOTIONNODE_H

#include <libada/Ada.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>

namespace wecook {

class MotionNode {
 public:
  class Result;

  MotionNode(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
             const dart::dynamics::MetaSkeletonPtr &skeleton)
      : m_stateSpace(stateSpace), m_skeleton(skeleton) {

  }

  virtual void plan(const std::shared_ptr<ada::Ada> &ada,
                    const std::shared_ptr<ada::Ada> &adaImg,
                    Result *result = nullptr) = 0;

 protected:
  aikido::statespace::dart::MetaSkeletonStateSpacePtr m_stateSpace = nullptr;
  dart::dynamics::MetaSkeletonPtr m_skeleton = nullptr;

};

/// Base class for executing result of various motion nodes.
class MotionNode::Result {
 public:
  enum StatusType {
    /// Uninitialized status
        UNKNOWN = 0,
    /// Invalid start state or no start state specified (only for motion planning node)
        INVALID_START,
    /// Invalid goal state (only for motion planning node)
        INVALID_GOAL,
    /// No valid IK solution (only for motion planning node)
        INVALID_IK,
    /// Succeeded
        SUCCEEDED,
    /// Failed
        FAILED
  };

  /// Returns message.
  const std::string &getMessage() const {
    return mMessage;
  }

  void setMessage(const std::string &message) {
    mMessage = message;
  }

  const StatusType &getStatus() const {
    return mStatus;
  }

  void setStatus(const StatusType &status) {
    mStatus = status;
  }

 protected:
  /// Message.
  std::string mMessage = "";
  StatusType mStatus = StatusType::UNKNOWN;
};
}

#endif //WECOOK_MOTIONNODE_H
