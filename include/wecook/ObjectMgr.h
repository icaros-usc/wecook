//
// Created by hejia on 8/26/19.
//

#ifndef WECOOK_OBJECTMGR_H
#define WECOOK_OBJECTMGR_H

#include "utils.h"
#include "Object.h"
#include "Tag.h"
#include <Eigen/Dense>
#include "apriltags/AprilTagDetections.h"
#include "ObjectMgr.h"

namespace wecook {

class ObjectMgr {
  class InternalTag {
    public:
      InternalTag(uint32_t tagId, const Eigen::Isometry3d &T_object_tag): m_tagId(tagId), m_T_tag_object(T_object_tag) {

      }

      void updateCameraTransform(const Eigen::Isometry3d &T_tag_cam) {
        m_T_tag_cam = T_tag_cam;
      }

      uint32_t m_tagId;
      Eigen::Isometry3d m_T_tag_cam;
      Eigen::Isometry3d m_T_tag_object;
  };
  
  class InternalObject {
   public:
    InternalObject(bool ifSim, const ObjectMgr &mgr, const Object &object, aikido::planner::WorldPtr &env) : m_ifSim(ifSim), m_name(object.getName()), m_mgr(mgr){
      if (m_ifSim) {
        m_skeleton = env->getSkeleton(m_name);
        m_bn = m_skeleton->getBodyNode(0);
      } else {
        // TODO store another handle which could achieve transform of the object
        // Temporally using the same in the simulator
        m_skeleton = env->getSkeleton(m_name);
        m_bn = m_skeleton->getBodyNode(0);
      }
    }

    Eigen::Isometry3d getTransform() const {
      ROS_WARN_STREAM("ObjectMgr: getTransform  called for object " << m_name << ", sim value: " << m_ifSim);
      if (m_ifSim) {
           auto result = m_bn->getTransform();
           ROS_WARN_STREAM("ObjectMgr: Object " << m_name << " orientation w.r.t robot: " << std::endl << result.linear());
           ROS_WARN_STREAM("ObjectMgr: Object " << m_name << " position w.r.t robot: " << std::endl << result.translation());
           return result;
      } else {
          bool fallBackToSimulation = true;
          Eigen::Isometry3d result;
        auto objectTagPair = m_mgr.m_objectToTagMap.find(m_name);
        if (objectTagPair != m_mgr.m_objectToTagMap.end()) {
          auto tagIdObjPair = m_mgr.m_tags.find(objectTagPair->second);
          auto baseTagIdObjPair = m_mgr.m_tags.find(m_mgr.baseTagId);
          if (tagIdObjPair != m_mgr.m_tags.end() && baseTagIdObjPair != m_mgr.m_tags.end()) {
            auto tag = tagIdObjPair->second;
            auto baseTag = baseTagIdObjPair->second;
            result = baseTag.m_T_tag_object * baseTag.m_T_tag_cam.inverse() * tag.m_T_tag_cam * tag.m_T_tag_object.inverse();
            ROS_WARN_STREAM("ObjectMgr: base tag has orientation w.r.t camera: " << std::endl << baseTag.m_T_tag_cam.linear());
            ROS_WARN_STREAM("ObjectMgr: base tag has position w.r.t camera: " << std::endl << baseTag.m_T_tag_cam.translation());
            ROS_WARN_STREAM("ObjectMgr: object tag for " << m_name << " has orientation w.r.t camera: " << std::endl << tag.m_T_tag_cam.linear());
            ROS_WARN_STREAM("ObjectMgr: object tag for " << m_name << " has position w.r.t camera: " << std::endl << tag.m_T_tag_cam.translation());
            ROS_WARN_STREAM("ObjectMgr: Object " << m_name << " orientation w.r.t robot: " << std::endl << result.linear());
            ROS_WARN_STREAM("ObjectMgr: Object " << m_name << " position w.r.t robot: " << std::endl << result.translation());
            fallBackToSimulation = false;
          }
        }
        if (fallBackToSimulation) {
        // PK-TODO: Fallback to simulated value when there is no tag information, need to revisit.
            result = m_bn->getTransform();
            ROS_WARN_STREAM("ObjectMgr: Object " << m_name << " is using simulated transform in real demo likely because it doesn't have a tag attached to it");
            ROS_WARN_STREAM("ObjectMgr: Object " << m_name << " orientation w.r.t robot: " << std::endl << result.linear());
            ROS_WARN_STREAM("ObjectMgr: Object " << m_name << " position w.r.t robot: " << std::endl << result.translation());
        }
        return result;
      }
    }

    dart::dynamics::BodyNode *getBodyNode() const {
      return m_bn;
    }

    bool ifContained() const {
      return m_skeleton->getNumBodyNodes() == 0;
    }

   private:
    const wecook::ObjectMgr& m_mgr;
    bool m_ifSim;
    std::string m_name;
    dart::dynamics::BodyNodePtr m_bn = nullptr;
    dart::dynamics::SkeletonPtr m_skeleton = nullptr;

  };

 public:
  ObjectMgr(const ros::NodeHandle &nh): m_nh(nh){
  };

  ObjectMgr(const ObjectMgr &other) {
    m_nh = other.m_nh;
    m_objects = other.m_objects;
  }

  void init(std::vector<Object> &objects, std::vector<Tag> &tags, bool ifSim, aikido::planner::WorldPtr &env);

  void clear(std::vector<Object> &objects, std::vector<Tag> &tags, aikido::planner::WorldPtr &env);

  dart::dynamics::BodyNode *getObjBodyNode(const std::string &obj) const {
    return m_objects.at(obj).getBodyNode();
  }

  Eigen::Isometry3d getObjTransform(const std::string &obj) const {
    return m_objects.at(obj).getTransform();
  }

  bool ifContained(const std::string &obj) const {
    // could be grabbed by hands or connected with other objects
    return m_objects.at(obj).ifContained();
  }

  dart::collision::CollisionGroupPtr createCollisionGroupExceptFoodAndToMoveObj(const std::string &toMove, dart::collision::FCLCollisionDetectorPtr &collisionDetector);

 private:
  std::map<std::string, InternalObject> m_objects;
  std::map<std::string, uint32_t> m_objectToTagMap;
  std::map<uint32_t, InternalTag > m_tags;
  const uint32_t baseTagId = 0; //PK_TODO: Update this value as per our usage
  ros::NodeHandle m_nh;
  ros::Subscriber m_Listener;
  void processTagMsg(const apriltags::AprilTagDetections::ConstPtr &msg);
  uint32_t m_tagMsgCounter = 0; // For debugging and understanding purpose
};

}

#endif //WECOOK_OBJECTMGR_H
