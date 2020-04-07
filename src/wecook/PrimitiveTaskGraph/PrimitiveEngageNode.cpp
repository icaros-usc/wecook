//
// Created by hejia on 8/26/19.
//

#include "wecook/Agents/Robot.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveEngageNode.h"

using namespace wecook;

void PrimitiveEngageNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                  std::shared_ptr<ObjectMgr> &objMgr,
                                  std::shared_ptr<ContainingMap> &containingMap,
                                  Result *result) {
  auto agent = agents[m_pid];

  if (agent->getType() == "human") {
//    waitForUser("Please move object to...");

    ros::Duration(1.0).sleep();

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
    ROS_INFO("Created ARM and Hand meta skeletons");

    // setup collisionDetector
    auto moveBn = objMgr->getObjBodyNode(m_toMove);
    auto collisionDetector = dart::collision::FCLCollisionDetector::create();
    auto containedBodyNodes = containingMap->getContainedBodyNodes(m_toMove);
    std::shared_ptr<dart::collision::CollisionGroup>
        armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), moveBn, handSkeleton.get());
    for (const auto &containedBodyNode : containedBodyNodes) {
      armCollisionGroup->addShapeFramesOf(containedBodyNode);
    }
    auto envCollisionGroup = objMgr->createCollisionGroupExceptFoodAndToMoveObj(m_toMove, collisionDetector);
    // We also need to add skeletons of other agents
    for (const auto &itr : agents) {
      if (itr.first == m_pid) {
        continue;
      } else if (itr.second->getType() == "robot") {
        ROS_INFO("Creating other ARM and Hand meta skeletons");
        auto otherArmSkeleton = std::dynamic_pointer_cast<Robot, Agent>(itr.second)->getArm()->getMetaSkeleton();
        auto otherHandSkeleton = std::dynamic_pointer_cast<Robot, Agent>(itr.second)->getHand()->getMetaSkeleton();
        envCollisionGroup->addShapeFramesOf(otherArmSkeleton.get(), otherHandSkeleton.get());
        ROS_INFO("Created other ARM and Hand meta skeletons");
      }
    }
    std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
        std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
    collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);

    m_targetPose->mT0_w = objMgr->getObjTransform(m_refObject);

    auto motion1 = std::make_shared<TSRMotionNode>(m_targetPose,
                                                   moveBn,
                                                   collisionFreeConstraint,
                                                   armSpace,
                                                   armSkeleton);
    motion1->setTimeStep(m_timeStep);
    MotionNode::Result motionNodeResult{};
    ROS_INFO("About to execute plan for primitive engage node");
    motion1->plan(robot->m_ada, robot->m_adaImg, &motionNodeResult);
    ROS_INFO("Should have executed plan for primitive engage node");
    if (motionNodeResult.getStatus() == MotionNode::Result::StatusType::INVALID_GOAL) {
      if (result) {
        result->setStatus(PrimitiveActionNode::Result::StatusType::INVALID_MOTION_GOAL);
        return;
      }
    } else if (motionNodeResult.getStatus() == MotionNode::Result::StatusType::INVALID_IK
        || motionNodeResult.getStatus() == MotionNode::Result::StatusType::INVALID_START
        || motionNodeResult.getStatus() == MotionNode::Result::StatusType::INVALID_START
        || motionNodeResult.getStatus() == MotionNode::Result::StatusType::FAILED) {
      if (result) {
        result->setStatus(PrimitiveActionNode::Result::StatusType::INVALID_MOTION_FAIL);
      }
    } else if (result) {
      result->setStatus(PrimitiveActionNode::Result::StatusType::SUCCEEDED);
    }

    m_ifExecuted = true;
  }
}