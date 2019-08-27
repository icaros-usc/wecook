//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveGrabNode.h"
#include "wecook/utils.h"
#include "wecook/Agent.h"
#include "wecook/Robot.h"
#include "wecook/ConnMotionNode.h"

using namespace wecook;

void PrimitiveGrabNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                std::shared_ptr<ObjectMgr> &objMgr,
                                std::shared_ptr<ContainingMap> &containingMap) {
  auto agent = agents[m_pid];

  // since every primitive action node only involves one agent
  if (agent->getType() == "human") {
    // TODO human execute
    waitForUser("Please grab " + m_toGrab + "...");

    m_ifExecuted = true;
  } else if (agent->getType() == "robot") {
    auto robot = std::dynamic_pointer_cast<Robot, Agent>(agent);
    auto robotArm = robot->getArm();
    auto robotHand = robot->getHand();
    auto armSkeleton = robotArm->getMetaSkeleton();
    auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
    auto handSkeleton = robotHand->getMetaSkeleton();
    auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
    auto world = robot->getWorld();

    m_grabPose->mT0_w = objMgr->getObjTransform(m_toGrab);

    auto motion1 = std::make_shared<TSRMotionNode>(m_grabPose,
                                                   robotHand->getEndEffectorBodyNode(),
                                                   nullptr,
                                                   armSpace,
                                                   armSkeleton,
                                                   nullptr,
                                                   false);
    motion1->plan(robot->m_ada);

    auto conf = Eigen::Vector2d();
    conf << 0.75, 0.75;
    auto motion2 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
    ROS_INFO("Grabbing...Closing gripper");
    motion2->plan(robot->m_ada);

    // before do grab, we need to check if to grab object is connected with other object
    if (objMgr->ifContained(m_toGrab)) {
      if (containingMap->hasObject(m_toGrab)) {
        auto container = containingMap->getContainer(m_toGrab);
        auto containerSkeleton = world->getSkeleton(container.getContainerName());
        auto containedSkeleton = world->getSkeleton(container.getContainedName());
        // build an unconnect node
        auto motion = std::make_shared<ConnMotionNode>(containedSkeleton,
                                                       containerSkeleton,
                                                       container.getContainerName(),
                                                       container.getContainedName(),
                                                       containingMap,
                                                       false,
                                                       armSpace,
                                                       armSkeleton);
        motion->plan(robot->m_ada);
      } else {
        for (auto &other : agents) {
          if (other.first != m_pid && other.second->getType() == "robot") {
            auto robot2 = std::dynamic_pointer_cast<Robot, Agent>(other.second);
            auto robotArm2 = robot2->getArm();
            auto armSkeleton2 = robotArm2->getMetaSkeleton();
            auto armSpace2 = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton2.get());
            if (robot2->getHand()->isGrabbing(m_toGrab) == 0) {
              ROS_INFO("Need to let the other robot ungrabs this object");
              auto
                  motion = std::make_shared<GrabMotionNode>(world->getSkeleton(m_toGrab), false, armSpace2, armSkeleton2);
              motion->plan(robot2->m_ada);
            } else {
              break;
            }
          }
        }
      }
    }
    auto motion3 = std::make_shared<GrabMotionNode>(world->getSkeleton(m_toGrab), true, armSpace, armSkeleton);
    motion3->plan(robot->m_ada);
    ROS_INFO("Grabbing...Connecting");

    m_ifExecuted = true;
  }
}
