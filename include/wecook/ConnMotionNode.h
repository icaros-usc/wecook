//
// Created by hejia on 8/12/19.
//

#ifndef WECOOK_CONNMOTIONNODE_H
#define WECOOK_CONNMOTIONNODE_H

#include "MotionNode.h"
#include "ContainingMap.h"

namespace wecook {

class ConnMotionNode : public MotionNode {
 public:
  ConnMotionNode(const dart::dynamics::SkeletonPtr &bodyToConn,
                 const dart::dynamics::SkeletonPtr &bodyThatConns,
                 const std::string &containerName,
                 const std::string &objectName,
                 std::shared_ptr<ContainingMap> &containingMap,
                 bool conn,
                 const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                 const dart::dynamics::MetaSkeletonPtr &skeleton,
                 const std::shared_ptr<PreCondition> &condition = nullptr) : MotionNode(stateSpace,
                                                                                           skeleton,
                                                                                           condition),
                                                                                m_bodyToConn(bodyToConn),
                                                                                m_bodyThatConns(bodyThatConns),
                                                                                m_conn(conn),
                                                                                m_containingMap(containingMap),
                                                                                m_containerName(containerName),
                                                                                m_objectName(objectName) {

  }

  void plan(const std::shared_ptr<ada::Ada> &ada);

 private:
  dart::dynamics::SkeletonPtr m_bodyToConn;
  dart::dynamics::SkeletonPtr m_bodyThatConns;
  std::string m_containerName;
  std::string m_objectName;
  bool m_conn;
  std::shared_ptr<ContainingMap> m_containingMap;

};

}

#endif //WECOOK_CONNMOTIONNODE_H
