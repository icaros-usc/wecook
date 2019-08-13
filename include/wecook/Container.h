//
// Created by hejia on 8/12/19.
//

#ifndef WECOOK_CONTAINER_H
#define WECOOK_CONTAINER_H

#include <dart/dart.hpp>
#include "Object.h"
#include "ConnMetadata.h"

namespace wecook {
class Container : public Object {
 public:
  Container(const std::string &name,
            const std::string &url,
            const std::vector<double> &pose,
            dart::dynamics::BodyNodePtr containerBodyNode) :
      Object(name, url, pose),
      m_containerBodyNode(containerBodyNode) {

  }

  void connect(const dart::dynamics::SkeletonPtr &bodyToConnect);

  void unconnect();

 private:
  std::shared_ptr<ConnMetadata> m_connMetadata = nullptr;
  dart::dynamics::BodyNodePtr m_containerBodyNode = nullptr;

//  void disablePairwiseSelfCollision(
//      const dart::dynamics::BodyNodePtr &singleNode,
//      const dart::dynamics::BodyNodePtr &rootNode,
//      const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &selfCollisionFilter);
};
}

#endif //WECOOK_CONTAINER_H
