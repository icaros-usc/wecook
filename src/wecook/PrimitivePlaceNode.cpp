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
  auto theOtherPid = m_pid;
  if (theOtherPid == "p1") {
    theOtherPid = "p2";
  } else {
    theOtherPid = "p1";
  }
  auto theOther = agents[theOtherPid];
  if (agent->getType() == "human") {
    waitForUser("Please place object " + m_toPlace);

    m_ifExecuted = true;
  } else if (agent->getType() == "robot") {
    auto theOtherRobot = std::dynamic_pointer_cast<Robot, Agent>(theOther);
    auto robot = std::dynamic_pointer_cast<Robot, Agent>(agent);
    auto robotArm = robot->getArm();
    auto robotHand = robot->getHand();
    auto armSkeleton = robotArm->getMetaSkeleton();
    auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
    auto handSkeleton = robotHand->getMetaSkeleton();
    auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
    auto world = robot->getWorld();

    if (robot->getHand()->isGrabbing(m_toPlace) == 0) {
      m_targetPose->mT0_w = objMgr->getObjTransform(m_refObject);
      auto grabbedBodyNode = theOtherRobot->getHand()->getGrabbedBodyNode();
      auto placeBn = objMgr->getObjBodyNode(m_toPlace);
      auto collisionDetector = dart::collision::FCLCollisionDetector::create();
      std::shared_ptr<dart::collision::CollisionGroup>
          armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), placeBn, handSkeleton.get());
      auto envCollisionGroup =
          objMgr->createCollisionGroupExceptFoodAndMovingObj(m_toPlace, collisionDetector, grabbedBodyNode);
      std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
          std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
      collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);

      auto motion1 = std::make_shared<TSRMotionNode>(m_targetPose,
                                                     placeBn,
                                                     collisionFreeConstraint,
                                                     armSpace,
                                                     armSkeleton,
                                                     nullptr,
                                                     m_ifDebug);
      motion1->plan(robot->m_ada);

      auto motion2 = std::make_shared<GrabMotionNode>(world->getSkeleton(m_toPlace), false, armSpace, armSkeleton);
      motion2->plan(robot->m_ada);
    }

    auto conf = Eigen::Vector2d();
    conf << 0., 0.;
    auto motion3 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
    motion3->plan(robot->m_ada);

    Eigen::Vector3d delta_x(0., 0., 0.005);
    auto motion4 =
        std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
                                                delta_x,
                                                dart::dynamics::Frame::World(),
                                                20,
                                                armSpace,
                                                armSkeleton);
    motion4->plan(robot->m_ada);

    m_ifExecuted = true;
  }
}