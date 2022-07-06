//
// Created by hejia on 8/26/19.
//

#ifndef WECOOK_OBJECTMGR_H
#define WECOOK_OBJECTMGR_H

#include <boost/thread.hpp>

#include "utils.h"
#include "Object.h"
#include "Tag.h"
#include <Eigen/Dense>
#include "pyapriltags/AprilTagDetections.h"
#include "ObjectMgr.h"

namespace wecook {

    class ObjectMgr {
        class InternalTag {
        public:
            InternalTag(uint32_t tagId, const Eigen::Isometry3d &T_object_tag) : m_tagId(tagId),
                                                                                 m_T_tag_object(T_object_tag) {

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
            InternalObject(bool ifSim, const ObjectMgr &mgr, const Object &object, aikido::planner::WorldPtr &env)
                    : m_ifSim(ifSim), m_name(object.getName()), m_mgr(mgr) {
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

            Eigen::Isometry3d setTransform(Eigen::Isometry3d newTransform) const {
                dynamic_cast<dart::dynamics::FreeJoint *>(m_skeleton->getJoint(0))->setTransform(newTransform);
            }

            Eigen::Isometry3d getTransform() const {
                return m_bn->getTransform();
            }

            dart::dynamics::BodyNode *getBodyNode() const {
                return m_bn;
            }

            bool ifContained() const {
                return m_skeleton->getNumBodyNodes() == 0;
            }

        private:
            const wecook::ObjectMgr &m_mgr;
            bool m_ifSim;
            std::string m_name;
            dart::dynamics::BodyNodePtr m_bn = nullptr;
            dart::dynamics::SkeletonPtr m_skeleton = nullptr;

        };

    public:
        ObjectMgr(const ros::NodeHandle &nh) : m_nh(nh) {
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

        dart::collision::CollisionGroupPtr createCollisionGroupExceptFoodAndToMoveObj(const std::string &toMove,
                                                                                      dart::collision::FCLCollisionDetectorPtr &collisionDetector);

    private:
        std::map<std::string, InternalObject> m_objects;
        std::map<std::string, uint32_t> m_objectToTagMap;
        std::map<uint32_t, InternalTag> m_tags;
        const uint32_t baseTagId = 0;
        ros::NodeHandle m_nh;
        ros::Subscriber m_Listener;

        void processTagMsg(const pyapriltags::AprilTagDetections::ConstPtr &msg);

        void updateObjectTransforms();

        Eigen::Isometry3d setObjTransform(const std::string &obj, Eigen::Isometry3d newTransform);

        uint32_t m_tagMsgCounter = 0; // For debugging and understanding purpose
        bool m_stopUpdatingFromTags = false;
        boost::thread *m_objectPoseUpdateThread = nullptr;
    };

}

#endif //WECOOK_OBJECTMGR_H
