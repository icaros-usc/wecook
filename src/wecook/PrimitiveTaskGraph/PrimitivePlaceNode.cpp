//
// Created by hejia on 8/26/19.
//

#include "wecook/Agents/Robot.h"
#include "wecook/PrimitiveTaskGraph/PrimitivePlaceNode.h"

using namespace wecook;

void PrimitivePlaceNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                 std::shared_ptr<ObjectMgr> &objMgr,
                                 std::shared_ptr<ContainingMap> &containingMap,
                                 Result *result) {
    auto agent = agents[m_pid];
    if (agent->getType() == "human") {
//        waitForUser("Please place object " + m_toPlace);
        ros::Duration(1.0).sleep();
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

        if (robot->getHand()->isGrabbing(m_toPlace) == 0) {
            m_targetPose->mT0_w = objMgr->getObjTransform(m_refObject);
            auto placeBn = objMgr->getObjBodyNode(m_toPlace);
            auto collisionDetector = dart::collision::FCLCollisionDetector::create();
            std::shared_ptr<dart::collision::CollisionGroup>
                    armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), placeBn,
                                                                                handSkeleton.get());
            auto envCollisionGroup =
                    objMgr->createCollisionGroupExceptFoodAndToMoveObj(m_toPlace, collisionDetector);
            // We also need to add skeletons of other agents
            for (const auto &itr : agents) {
                if (itr.first == m_pid) {
                    continue;
                } else if (itr.second->getType() == "robot") {
                    auto otherArmSkeleton = std::dynamic_pointer_cast<Robot, Agent>(
                            itr.second)->getArm()->getMetaSkeleton();
                    auto otherHandSkeleton = std::dynamic_pointer_cast<Robot, Agent>(
                            itr.second)->getHand()->getMetaSkeleton();
                    envCollisionGroup->addShapeFramesOf(otherArmSkeleton.get(), otherHandSkeleton.get());
                }
            }
            std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
                    std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
            collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);

            auto motion1 = std::make_shared<TSRMotionNode>(m_targetPose,
                                                           placeBn,
                                                           collisionFreeConstraint,
                                                           armSpace,
                                                           armSkeleton,
                                                           m_ifDebug);
            MotionNode::Result motionNodeResult{};
            motion1->plan(robot->m_adaPlan, robot->m_adaExec, &motionNodeResult);
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

            auto motion2 = std::make_shared<GrabMotionNode>(world->getSkeleton(m_toPlace), false, armSpace,
                                                            armSkeleton);
            motion2->plan(robot->m_adaPlan, robot->m_adaExec);
        }

        auto adaPlanConf = Eigen::Vector2d();
        adaPlanConf << 0., 0.;
        auto adaExecConf = Eigen::Vector2d();
        adaExecConf << 0., 0.;
        auto motion3 = std::make_shared<ConfMotionNode>(adaPlanConf, adaExecConf, handSpace, handSkeleton);
        motion3->plan(robot->m_adaPlan, robot->m_adaExec);

        if (agent->ifSim()) {
            Eigen::Vector3d delta_x(0., 0., 0.005);
            auto motion4 =
                    std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
                                                            delta_x,
                                                            dart::dynamics::Frame::World(),
                                                            20,
                                                            armSpace,
                                                            armSkeleton,
                                                            nullptr);
            motion4->plan(robot->m_adaPlan, robot->m_adaExec);
        } else {
            Eigen::Vector3d delta_x(0., 0., 0.05);
            auto motion4 =
                    std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
                                                            delta_x,
                                                            dart::dynamics::Frame::World(),
                                                            1,
                                                            armSpace,
                                                            armSkeleton,
                                                            nullptr);
            motion4->plan(robot->m_adaPlan, robot->m_adaExec);
        }

        m_ifExecuted = true;
    }
}