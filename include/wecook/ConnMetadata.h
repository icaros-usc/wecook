//
// Created by hejia on 8/12/19.
//

#ifndef WECOOK_CONNMETADATA_H
#define WECOOK_CONNMETADATA_H

#include <dart/dynamics/dynamics.hpp>

namespace wecook {
struct ConnMetadata {
  ConnMetadata(dart::dynamics::BodyNodePtr bodyNode,
               const std::string &oldName,
               dart::dynamics::SkeletonPtr parentSkeleton,
               const dart::dynamics::FreeJoint::Properties &jointProperties) : m_bodyNode(bodyNode),
                                                                               m_oldName(oldName),
                                                                               m_parentSkeleton(parentSkeleton),
                                                                               m_jointProperties(jointProperties) {

  }

  dart::dynamics::BodyNodePtr m_bodyNode;
  std::string m_oldName;
  dart::dynamics::SkeletonPtr m_parentSkeleton;
  dart::dynamics::FreeJoint::Properties m_jointProperties;
};
}

#endif //WECOOK_CONNMETADATA_H
