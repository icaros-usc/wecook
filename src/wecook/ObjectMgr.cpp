//
// Created by hejia on 8/26/19.
//

#include <ros/console.h>

#include "wecook/ObjectMgr.h"
#include "apriltags/AprilTagDetections.h"

using namespace wecook;

void ObjectMgr::init(std::vector<Object> &objects, std::vector<Tag> &tags, bool ifSim, aikido::planner::WorldPtr &env) {
  for (auto &object : objects) {
    addBodyFromURDF(env.get(), object.getUrl(), object.getPose(), object.getName());

    m_objects.emplace(std::pair<std::string, InternalObject>{object.getName(), InternalObject{ifSim, *this, object, env}});
  }
  for (auto &tag : tags) {
    m_objectToTagMap[tag.getObjectName()] = tag.getTagId();
    auto objPose = tag.getObjectPose();
    auto objTransform = vectorToIsometry(objPose);
    m_tags.emplace(std::pair<uint32_t, InternalTag>{tag.getTagId(), InternalTag{tag.getTagId(), objTransform}});
  }
  m_Listener = m_nh.subscribe("detections", 1000, &ObjectMgr::processTagMsg, this);
}

void ObjectMgr::clear(std::vector<Object> &objects, std::vector<Tag> &tags, aikido::planner::WorldPtr &env) {
  for (auto &object : objects) {
    auto skeleton = env->getSkeleton(object.getName());
    env->removeSkeleton(skeleton);
  }
  // PK-TODO: Do we really need to get tags as input to this method? Can we simple clear the tags and objectToTags dictionaries?
  for (auto &tag : tags) {
  }
  m_tags.clear();
  m_objectToTagMap.clear();
  m_Listener.shutdown(); //PK_TODO: Do I need to explicitly call this?
}

dart::collision::CollisionGroupPtr ObjectMgr::createCollisionGroupExceptFoodAndToMoveObj(const std::string &toMove,
                                                                                         dart::collision::FCLCollisionDetectorPtr &collisionDetector) {
  auto envCollisionGroup = collisionDetector->createCollisionGroup();
  for (auto &object : m_objects) {
    if (object.first == toMove || object.first.find("food") != std::string::npos
        || object.first.find("mouth") != std::string::npos)
      continue;
    auto newNode = object.second.getBodyNode();
    envCollisionGroup->addShapeFramesOf(newNode);
  }
  return envCollisionGroup;
}

void ObjectMgr::processTagMsg(const apriltags::AprilTagDetections::ConstPtr &msg) {
  for (auto &detection: msg->detections) {
  // PK-TODO: My lack of knowledge with C++ standard API - there must be a better way to construct the vector below!
    std::vector<double> vector;
    vector.push_back(detection.pose.position.x);
    vector.push_back(detection.pose.position.y);
    vector.push_back(detection.pose.position.z);
    vector.push_back(detection.pose.orientation.x);
    vector.push_back(detection.pose.orientation.y);
    vector.push_back(detection.pose.orientation.z);
    vector.push_back(detection.pose.orientation.w);
    auto camTransform = vectorToIsometry(vector);
    auto tag = m_tags.find(detection.id);
    if(tag != m_tags.end()) {
      tag->second.updateCameraTransform(camTransform);
    }
  }
}