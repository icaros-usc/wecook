//
// Created by hejia on 8/26/19.
//

#ifndef WECOOK_OBJECTMGR_H
#define WECOOK_OBJECTMGR_H

#include "utils.h"
#include "Object.h"

namespace wecook {

class ObjectMgr {
  class InternalObject {
   public:
    InternalObject(bool ifSim, const Object &object, aikido::planner::WorldPtr &env) : m_ifSim(ifSim) {
      if (m_ifSim) {
        m_skeleton = env->getSkeleton(object.getName());
        m_bn = m_skeleton->getBodyNode(0);
      } else {
        // TODO store another handle which could achieve transform of the object
      }
    }

    Eigen::Isometry3d getTransform() {
      if (m_ifSim) {
        return m_bn->getTransform();
      } else {
        // TODO achieve from apriltag?
      }
    }

    dart::dynamics::BodyNode *getBodyNode() {
      return m_bn;
    }

    bool ifContained() {
      return m_skeleton->getNumBodyNodes() == 0
    }

   private:
    bool m_ifSim;
    dart::dynamics::BodyNodePtr m_bn = nullptr;
    dart::dynamics::SkeletonPtr m_skeleton = nullptr;

  };

 public:
  ObjectMgr() = default;

  void init(std::vector<Object> &objects, bool ifSim, aikido::planner::WorldPtr &env);

  void clear(std::vector<Object> &objects, bool ifSim, aikido::planner::WorldPtr &env);

  inline dart::dynamics::BodyNode *getObjBodyNode(const std::string &obj) const {
    return m_objects[obj].getBodyNode();
  }

  inline Eigen::Isometry3d getObjTransform(const std::string &obj) const {
    return m_objects[obj].getTransform();
  }

  inline bool ifContained(const std::string &obj) const {
    // could be grabbed by hands or connected with other objects
    return m_objects[obj].ifContained();
  }

 private:
  std::map<std::string, InternalObject> m_objects;
};

}

#endif //WECOOK_OBJECTMGR_H
