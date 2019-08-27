//
// Created by hejia on 8/26/19.
//

#include "wecook/Robot.h"
#include "wecook/PrimitivePlaceNode.h"

using namespace wecook;

void PrimitivePlaceNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                 std::shared_ptr<ObjectMgr> &objMgr,
                                 std::shared_ptr<ContainingMap> &containingMap) {
  auto agent = agents[m_pid];

  if (agent->getType() == "human") {
    waitForUser("Please place object " + m_toPlace);

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

    m_targetPose->mT0_w = objMgr->getObjTransform(m_refObject);
    auto placeBn = objMgr->getObjBodyNode(m_toPlace);
    auto motion1 = std::make_shared<TSRMotionNode>(m_targetPose,
                                                   placeBn,
                                                   nullptr,
                                                   armSpace,
                                                   armSkeleton,
                                                   nullptr,
                                                   false);
    motion1->plan(robot->m_ada);

    if (robot->getHand()->isGrabbing(m_toPlace) == 2) {
      auto motion2 = std::make_shared<GrabMotionNode>(world->getSkeleton(m_toPlace), false, armSpace, armSkeleton);
      motion2->plan(robot->m_ada);
    }

    auto conf = Eigen::Vector2d();
    conf << 0., 0.;
    auto motion3 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
    motion3->plan(robot->m_ada);

    m_ifExecuted = true;
  }
}