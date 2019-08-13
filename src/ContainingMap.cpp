//
// Created by hejia on 8/12/19.
//

#include "wecook/ContainingMap.h"
#include "ros/console.h"

using namespace wecook;

ContainingMap::ContainingMap(const wecook::Task &task, const aikido::planner::WorldPtr &env) : m_env(env) {
  auto containingPairs = task.getContainingPairs();
  for (const auto &containingPair : containingPairs) {
    auto containerName = containingPair.first;
    auto objectName = containingPair.second;
    auto containerSkeleton = m_env->getSkeleton(containerName);
    auto objectSkeleton = m_env->getSkeleton(objectName);
    Container container(containerName, objectName, containerSkeleton->getBodyNode(0));
    container.connect(objectSkeleton);
    m_containers.emplace_back(container);
  }
}

void ContainingMap::unconnectAll() {
  for (auto &container : m_containers) {
    container.unconnect();
  }

  m_containers.clear();
}

void ContainingMap::connect(const dart::dynamics::SkeletonPtr &bodyToConn,
                            const dart::dynamics::SkeletonPtr &bodyThatConns,
                            const std::string &containerName,
                            const std::string &objectName) {
  Container container(containerName, objectName, bodyThatConns->getBodyNode(0));
  container.connect(bodyToConn);
  m_containers.emplace_back(container);
}

void ContainingMap::unconnect(const dart::dynamics::SkeletonPtr &bodyToConn,
                              const dart::dynamics::SkeletonPtr &bodyThatConns,
                              const std::string &containerName,
                              const std::string &objectName) {
  for (auto container = m_containers.begin(); container != m_containers.end(); ++container) {
    if (container->ifSame(containerName, objectName)) {
      ROS_INFO("Unconnect!");
      container->unconnect();
      m_containers.erase(container);
      break;
    }
  }
}