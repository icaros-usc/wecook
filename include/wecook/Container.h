//
// Created by hejia on 8/12/19.
//

#ifndef WECOOK_CONTAINER_H
#define WECOOK_CONTAINER_H

#include <dart/dart.hpp>
#include "Object.h"
#include "ConnMetadata.h"

namespace wecook {
class Container {
 public:
  Container(const std::string &containerName,
            const std::string &containedName,
            dart::dynamics::BodyNodePtr containerBodyNode) :
      m_containerName(containerName), m_containedName(containedName), m_containerBodyNode(containerBodyNode) {

  }

  void connect(const dart::dynamics::SkeletonPtr &bodyToConnect);

  void unconnect();

  inline bool ifSame(const std::string &name, const std::string &objectName) {
    if (name == m_containerName && objectName == m_containedName) {
      return true;
    } else {
      return false;
    }
  }

  inline bool contains(const std::string &objectName) const {
    if (m_containedName == objectName) {
      return true;
    } else {
      return false;
    }
  }

  inline Eigen::Isometry3d getContainedPose() const {
    return m_connMetadata->m_bodyNode->getWorldTransform();
  }

  inline dart::dynamics::BodyNode *getContainedBodyNode() const {
    return m_connMetadata->m_bodyNode;
  }

  inline std::string getContainerName() const {
    return m_containerName;
  }

  inline std::string getContainedName() const {
    return m_containedName;
  }

 private:
  std::string m_containerName;
  std::string m_containedName;
  std::shared_ptr<ConnMetadata> m_connMetadata = nullptr;
  dart::dynamics::BodyNodePtr m_containerBodyNode = nullptr;

//  void disablePairwiseSelfCollision(
//      const dart::dynamics::BodyNodePtr &singleNode,
//      const dart::dynamics::BodyNodePtr &rootNode,
//      const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &selfCollisionFilter);
};
}

#endif //WECOOK_CONTAINER_H
