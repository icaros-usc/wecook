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

dart::collision::CollisionGroupPtr ObjectMgr::createCollisionGroupExceptFoodAndMovingObj(dart::collision::FCLCollisionDetectorPtr &collisionDetector) {
  // TODO add collision check between agents
  auto envCollisionGroup = collisionDetector->createCollisionGroup();
  for (auto &object : m_objects) {
    auto newNode = object.second.getBodyNode();
    envCollisionGroup->addShapeFramesOf(newNode);
  }
  return envCollisionGroup;
}