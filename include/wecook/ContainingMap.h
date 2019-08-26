//
// Created by hejia on 8/12/19.
//

#ifndef WECOOK_CONTAININGMAP_H
#define WECOOK_CONTAININGMAP_H

#include <aikido/planner/World.hpp>
#include "Task.h"
#include "Container.h"

namespace wecook {

class ContainingMap {
 public:
  ContainingMap(const Task &task,
                const aikido::planner::WorldPtr &env);

  void unconnectAll();

  void connect(const dart::dynamics::SkeletonPtr &bodyToConn,
               const dart::dynamics::SkeletonPtr &bodyThatConns,
               const std::string &containerName,
               const std::string &objectName);

  void unconnect(const dart::dynamics::SkeletonPtr &bodyToConn,
                 const dart::dynamics::SkeletonPtr &bodyThatConns,
                 const std::string &containerName,
                 const std::string &objectName);

  inline Eigen::Isometry3d getContainedPose(const std::string &containedName) {
    for (const auto &container : m_containers) {
      if (container.contains(containedName)) {
        return container.getContainedPose();
      }
    }
  }

  inline dart::dynamics::BodyNode *getContainedBodyNode(const std::string &containedName) {
    for (const auto &container : m_containers) {
      if (container.contains(containedName)) {
        return container.getContainedBodyNode();
      }
    }
    return nullptr;
  }

  inline bool hasTuple(const std::string &containerName, const std::string &containedName) {
    for (const auto &container : m_containers) {
      if (container.getContainerName() == containerName && container.getContainedName() == containedName) {
        return true;
      }
    }
    return false;
  }

 private:
  std::vector<Container> m_containers;
  aikido::planner::WorldPtr m_env;
};

}

#endif //WECOOK_CONTAININGMAP_H
