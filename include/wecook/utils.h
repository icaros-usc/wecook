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
  Eigen::Quaterniond q(p[3], p[4], p[5], p[6]);
  pose.linear() = Eigen::Matrix3d(q);
  return pose;
}

inline std::shared_ptr<::dart::dynamics::Skeleton> addBodyFromURDF(aikido::planner::World *world,
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

inline Eigen::Isometry3d getObjectPose(const dart::dynamics::SkeletonPtr &skeleton) {
  auto body = skeleton->getBodyNode(0);
  auto transform = body->getWorldTransform();
  return transform;
}

}

#endif //WECOOK_UTILS_H
