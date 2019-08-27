//
// Created by hejia on 8/26/19.
//

#include "wecook/Robot.h"
#include "wecook/PrimitiveMoveToNode.h"

using namespace wecook;

void PrimitiveMoveToNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                  std::shared_ptr<ObjectMgr> &objMgr,
                                  std::shared_ptr<ContainingMap> &containingMap) {
  auto agent = agents[m_pid];

  if (agent->getType() == "human") {
    waitForUser("Please move object to...");

    m_ifExecuted = true;
  } else if (agent->getType() == "robot") {
    ROS_INFO("Moving...");
    auto robot = std::dynamic_pointer_cast<Robot, Agent>(agent);
    auto robotArm = robot->getArm();
    auto robotHand = robot->getHand();
    auto armSkeleton = robotArm->getMetaSkeleton();
    auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
    auto handSkeleton = robotHand->getMetaSkeleton();
    auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
    auto world = robot->getWorld();

    m_targetPose->mT0_w = objMgr->getObjTransform(m_refObject);
    auto moveBn = objMgr->getObjBodyNode(m_toMove);
    auto motion1 = std::make_shared<TSRMotionNode>(m_targetPose,
                                                   moveBn,
                                                   nullptr,
                                                   armSpace,
                                                   armSkeleton,
                                                   nullptr,
                                                   false);
    motion1->plan(robot->m_ada);

    m_ifExecuted = true;
  }
}