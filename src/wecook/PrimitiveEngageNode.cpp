//
// Created by hejia on 8/26/19.
//

#include "wecook/Robot.h"
#include "wecook/PrimitiveEngageNode.h"

using namespace wecook;

void PrimitiveEngageNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
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

    // setup collisionDetector
    auto moveBn = objMgr->getObjBodyNode(m_toMove);
    auto collisionDetector = dart::collision::FCLCollisionDetector::create();
    auto containedBodyNodes = containingMap->getContainedBodyNodes(m_toMove);
    std::shared_ptr<dart::collision::CollisionGroup>
        armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), moveBn, handSkeleton.get());
    for (const auto &containedBodyNode : containedBodyNodes) {
      armCollisionGroup->addShapeFramesOf(containedBodyNode);
    }
    auto envCollisionGroup = objMgr->createCollisionGroupExceptFoodAndMovingObj(m_toMove, collisionDetector);
    std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
        std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
    collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);

    m_targetPose->mT0_w = objMgr->getObjTransform(m_refObject);

    auto motion1 = std::make_shared<TSRMotionNode>(m_targetPose,
                                                   moveBn,
                                                   collisionFreeConstraint,
                                                   armSpace,
                                                   armSkeleton,
                                                   nullptr,
                                                   false);
    motion1->setTimeStep(m_timeStep);
    motion1->plan(robot->m_ada, robot->m_adaImg);

    m_ifExecuted = true;
  }
}