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

void ObjectMgr::clear(std::vector<Object> &objects, aikido::planner::WorldPtr &env) {
  for (auto &object : objects) {
    auto skeleton = env->getSkeleton(object.getName());
    env->removeSkeleton(skeleton);
  }
}

dart::collision::CollisionGroupPtr ObjectMgr::createCollisionGroupExceptFoodAndToMoveObj(const std::string &toMove,
                                                                                         dart::collision::FCLCollisionDetectorPtr &collisionDetector) {
  auto envCollisionGroup = collisionDetector->createCollisionGroup();
  for (auto &object : m_objects) {
    if (object.first == toMove || object.first.find("food") != std::string::npos
        || object.first.find("mouth") != std::string::npos)
      continue;
    auto newNode = object.second.getBodyNode();
    std::cout << object.first << std::endl;
    envCollisionGroup->addShapeFramesOf(newNode);
  }
  return envCollisionGroup;
}