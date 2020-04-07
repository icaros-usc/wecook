//
// Created by hejia on 8/26/19.
//

#include <ros/console.h>
#include <boost/thread.hpp>

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
  m_Listener = m_nh.subscribe("/apriltags/detections", 1000, &ObjectMgr::processTagMsg, this);
  m_objectPoseUpdateThread = new boost::thread(&ObjectMgr::updateObjectTransforms, this);
}

void ObjectMgr::clear(std::vector<Object> &objects, std::vector<Tag> &tags, aikido::planner::WorldPtr &env) {
  for (auto &object : objects) {
    auto skeleton = env->getSkeleton(object.getName());
    env->removeSkeleton(skeleton);
  }
  // PK-TODO: Do we really need to get tags as input to this method? Can we simple clear the tags and objectToTags dictionaries?
  for (auto &tag : tags) {
  }
  m_Listener.shutdown(); //PK_TODO: Do I need to explicitly call this?
  m_stopUpdatingFromTags = true;
  m_objectPoseUpdateThread->join();
  m_tags.clear();
  m_objectToTagMap.clear();
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
  if (m_tagMsgCounter == 0) {
      ROS_WARN_STREAM("processTagMsg is called first time");
  }
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
      if (m_tagMsgCounter == 0) {
          ROS_WARN_STREAM("processTagMsg: Updated the tag cam transform for tag id " << detection.id);
          ROS_WARN_STREAM("processTagMsg: tag for " << detection.id << " is updated, orientation w.r.t camera: " << std::endl << tag->second.m_T_tag_cam.linear());
          ROS_WARN_STREAM("processTagMsg: tag for " << detection.id << " is updated, position w.r.t camera: " << std::endl << tag->second.m_T_tag_cam.translation());
      }
    }
  }
  m_tagMsgCounter++;
}

Eigen::Isometry3d ObjectMgr::setObjTransform(const std::string &obj, Eigen::Isometry3d newTransform) {
  return m_objects.at(obj).setTransform(newTransform);
}

void ObjectMgr::updateObjectTransforms() {
    ROS_WARN_STREAM("updateObjectTransforms: Started");
    auto waitingTime = 1.0;
    while (!m_stopUpdatingFromTags) {
        ros::Duration(waitingTime).sleep();
        auto baseTagIdObjPair = m_tags.find(baseTagId);
        if (baseTagIdObjPair != m_tags.end()) {
            auto baseTag = baseTagIdObjPair->second;
            auto objectTagPair = m_objectToTagMap.begin();
            while (objectTagPair != m_objectToTagMap.end()) {
                auto tagId = objectTagPair->second;
                if (tagId != baseTagId) {
                    auto tagIdObjPair = m_tags.find(tagId);
                    if (tagIdObjPair != m_tags.end()) {
                        auto tag = tagIdObjPair->second;
                        auto newTransform = baseTag.m_T_tag_object * baseTag.m_T_tag_cam.inverse() * tag.m_T_tag_cam * tag.m_T_tag_object.inverse();
                        setObjTransform(objectTagPair->first, newTransform);
                    }
                }
                objectTagPair++;
            }
        }
    }
    ROS_WARN_STREAM("updateObjectTransforms: Finished");
}