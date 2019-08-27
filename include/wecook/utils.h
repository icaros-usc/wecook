//
// Created by hejia on 7/31/19.
//

#ifndef WECOOK_UTILS_H
#define WECOOK_UTILS_H

#include <string>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <dart/dart.hpp>
#include <aikido/planner/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <ros/ros.h>
#include "ContainingMap.h"
#include <tf2/LinearMath/Matrix3x3.h>

namespace wecook {

inline void waitForUser(const std::string &msg) {
  ROS_INFO(msg.c_str());
  std::cin.get();
}

inline Eigen::Isometry3d vectorToIsometry(std::vector<double> &poseVector) {
  double *ptr = &poseVector[0];
  Eigen::Map<Eigen::VectorXd> p(ptr, 7);

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = p.head(3);
  // w, x, y, z
  Eigen::Quaterniond q(p[6], p[3], p[4], p[5]);
  pose.linear() = Eigen::Matrix3d(q);
  return pose;
}

std::shared_ptr<::dart::dynamics::Skeleton> addBodyFromURDF(aikido::planner::World *world,
                                                            const std::string &uri,
                                                            std::vector<double> objectPose,
                                                            const std::string &name) {
  auto transform = vectorToIsometry(objectPose);

  dart::utils::DartLoader urdfLoader;
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  const auto skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);
  skeleton->setName(name);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint *>(skeleton->getJoint(0))
      ->setTransform(transform);

  world->addSkeleton(skeleton);
  return skeleton;
}

inline Eigen::Isometry3d getObjectPose(const dart::dynamics::SkeletonPtr &skeleton,
                                       const std::shared_ptr<ContainingMap> &containingMap) {
  if (skeleton->getNumBodyNodes() > 0) {
    auto body = skeleton->getBodyNode(0);
    auto pose = body->getWorldTransform();
    return pose;
  } else {
    // need to find
    return containingMap->getContainedPose(skeleton->getName());
  }
}

inline Eigen::Isometry3d getObjectPose(const dart::dynamics::MetaSkeletonPtr &skeleton,
                                       const std::shared_ptr<ContainingMap> &containingMap) {
  if (skeleton->getNumBodyNodes() > 0) {
    auto body = skeleton->getBodyNode(0);
    auto pose = body->getWorldTransform();
    return pose;
  } else {
    // need to find
    return containingMap->getContainedPose(skeleton->getName());
  }
}

inline Eigen::Vector6d TransformMatrix2SpatialVector(const Eigen::Isometry3d &transform) {
  auto rotation = dart::math::matrixToEulerXYZ(transform.linear());
  auto translation = transform.translation();
  Eigen::Vector6d spatialVector = Eigen::Vector6d::Zero();
  spatialVector << rotation(0, 0), rotation(1, 0), rotation(2, 0), translation[0], translation[1], translation[2];
  return spatialVector;
}

inline dart::dynamics::BodyNode *getBodyNode(const std::string &name,
                                             const std::shared_ptr<ContainingMap> &containingMap,
                                             const aikido::planner::WorldPtr &world) {
  auto skeleton = world->getSkeleton(name);
  if (skeleton->getNumBodyNodes() > 0) {
    return skeleton->getBodyNode(0);
  } else {
    return containingMap->getContainedBodyNode(name);
  }
}

}

#endif //WECOOK_UTILS_H
