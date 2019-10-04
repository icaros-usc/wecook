//
// Created by hejia on 8/26/19.
//

#include <ros/console.h>

#include "wecook/ObjectMgr.h"

using namespace wecook;

void ObjectMgr::init(std::vector<Object> &objects, bool ifSim, aikido::planner::WorldPtr &env) {
  for (auto &object : objects) {
    addBodyFromURDF(env.get(), object.getUrl(), object.getPose(), object.getName());

    m_objects.emplace(std::pair<std::string, InternalObject>{object.getName(), InternalObject{ifSim, object, env}});
  }
}

void ObjectMgr::clear(std::vector<Object> &objects, bool ifSim, aikido::planner::WorldPtr &env) {
  for (auto &object : objects) {
    auto skeleton = env->getSkeleton(object.getName());
    env->removeSkeleton(skeleton);
  }
}

dart::collision::CollisionGroupPtr ObjectMgr::createCollisionGroupExceptFoodAndMovingObj(const std::string &toMove,
                                                                                         dart::collision::FCLCollisionDetectorPtr &collisionDetector,
                                                                                         dart::dynamics::BodyNode *blackNode) {
  auto envCollisionGroup = collisionDetector->createCollisionGroup();
  for (auto &object : m_objects) {
    if ((object.first.find("knife") != std::string::npos || object.first.find("fork") != std::string::npos
        || object.first.find("spoon") != std::string::npos)
        && (toMove.find("knife") != std::string::npos || toMove.find("fork") != std::string::npos
            || toMove.find("spoon") != std::string::npos) && toMove.back() != object.first.back()) {
      continue;
    }
    if (object.first == toMove || object.first.find("food") != std::string::npos)
      continue;
//    ROS_INFO_STREAM("Adding object for collision detect " << object.first);
    auto newNode = object.second.getBodyNode();
    if (blackNode && newNode == blackNode) {
      continue;
    }
    envCollisionGroup->addShapeFramesOf(newNode);
  }
  return envCollisionGroup;
}