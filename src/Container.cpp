//
// Created by hejia on 8/12/19.
//

#include "wecook/Container.h"
#include "ros/console.h"

using namespace wecook;

void Container::connect(const dart::dynamics::SkeletonPtr &bodyToConnect) {
  auto joint = bodyToConnect->getJoint(0);
  auto freeJoint = dynamic_cast<dart::dynamics::FreeJoint *>(joint);
  if (freeJoint == nullptr) {
    std::stringstream ss;
    ss << "[Container::connect] Only Skeletons with a root FreeJoint may be connected. "
       << "Skeleton '" << bodyToConnect->getName() << "' has a " << "root " << joint->getType()
       << std::endl;
    throw std::runtime_error(ss.str());
  }

  // Get fields for ConnMetadata
  auto bodyNode = freeJoint->getChildBodyNode();
  std::string bodyNodeName = bodyNode->getName();
  auto jointProperties = freeJoint->getFreeJointProperties();

  // Get relative transform between container and BodyNode
  auto containerToBodyTransform = bodyNode->getTransform(m_containerBodyNode);

  // Connect connected BodyNode to container
  dart::dynamics::WeldJoint::Properties weldJointProperties;
  weldJointProperties.mT_ParentBodyToJoint = containerToBodyTransform;
  bodyNode->moveTo<dart::dynamics::WeldJoint>(m_containerBodyNode, weldJointProperties);

  // Moving the connected object into the same skeleton as the container means that
  // it will be considered during self-collision checking. Therefore, we need to
  // disable self-collision checking between connected object and container.
  //TODO
  m_connMetadata = std::make_shared<ConnMetadata>(bodyNode, bodyNodeName, bodyToConnect, jointProperties);
}

void Container::unconnect() {
  // Get connected body node and its transform wrt the world
  auto connectedBodyNode = m_connMetadata->m_bodyNode;
  Eigen::Isometry3d connectedBodyTransform = connectedBodyNode->getTransform();

  // Re-enable self-collision checking
  //TODO

  // Move connected BodyNode to root of the old object skeleton
  dart::dynamics::SkeletonPtr skeleton = m_connMetadata->m_parentSkeleton;
  connectedBodyNode->moveTo<dart::dynamics::FreeJoint>(skeleton, nullptr, m_connMetadata->m_jointProperties);

  // Set transform of skeleton FreeJoint wrt world
  auto joint = skeleton->getJoint(0);
  auto freeJoint = dynamic_cast<dart::dynamics::FreeJoint*>(joint);
  freeJoint->setTransform(connectedBodyTransform);

  // Restore old name
  std::string oldName = m_connMetadata->m_oldName;
  std::string newName = connectedBodyNode->setName(oldName);
  m_connMetadata.reset();
}