//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveTaskGraph/PrimitiveGraspNode.h"
#include "wecook/utils.h"
#include "wecook/Agents/Agent.h"
#include "wecook/Agents/Robot.h"
#include "wecook/LowLevelMotionNodes/ConnMotionNode.h"

using namespace wecook;

void PrimitiveGraspNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                 std::shared_ptr<ObjectMgr> &objMgr,
                                 std::shared_ptr<ContainingMap> &containingMap,
                                 Result *result) {
    auto agent = agents[m_pid];

    // since every primitive action node only involves one agent
    if (agent->getType() == "human") {
        // TODO human execute
//        waitForUser("Please grab " + m_toGrab + "...");
        ros::Duration(1.0).sleep();
        m_ifExecuted = true;
    } else if (agent->getType() == "robot") {
        ROS_INFO("PrimitiveGraspNode::execute: Starting grasping...");
        auto robot = std::dynamic_pointer_cast<Robot, Agent>(agent);
        auto robotArm = robot->getArm();
        auto robotHand = robot->getHand();
        auto armSkeleton = robotArm->getMetaSkeleton();
        auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
        auto handSkeleton = robotHand->getMetaSkeleton();
        auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
        auto world = robot->getWorld();

        m_grabPose->mT0_w = objMgr->getObjTransform(m_toGrab);

        // setup collisionDetector
        auto collisionDetector = dart::collision::FCLCollisionDetector::create();
        std::shared_ptr<dart::collision::CollisionGroup>
                armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), handSkeleton.get());
        auto envCollisionGroup = objMgr->createCollisionGroupExceptFoodAndToMoveObj("hand", collisionDetector);
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


        if (agent->ifSim()) {
            // If the robot is simulated we don't need to set an up pose
            auto motion1 = std::make_shared<TSRMotionNode>(m_grabPose,
                                                           robotHand->getEndEffectorBodyNode(),
                                                           collisionFreeConstraint,
                                                           armSpace,
                                                           armSkeleton);
            // TSRMotionNode
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
        } else {
            if (m_toGrab.find('spoon') != std::string::npos) {
                auto upPose = std::make_shared<aikido::constraint::dart::TSR>();
                upPose->mT0_w = m_grabPose->mT0_w;
                upPose->mBw = m_grabPose->mBw;
                upPose->mTw_e = m_grabPose->mTw_e;
                upPose->mTw_e.translation() = upPose->mTw_e.translation() + Eigen::Vector3d{0, 0, 0.15};
                auto motion2 = std::make_shared<TSRMotionNode>(upPose,
                                                               robotHand->getEndEffectorBodyNode(),
                                                               collisionFreeConstraint,
                                                               armSpace,
                                                               armSkeleton);
                // TSRMotionNode
                MotionNode::Result motion2NodeResult{};
                motion2->plan(robot->m_adaPlan, robot->m_adaExec, &motion2NodeResult);
            }

            // If the robot is simulated we don't need to set an up pose
            auto motion1 = std::make_shared<TSRMotionNode>(m_grabPose,
                                                           robotHand->getEndEffectorBodyNode(),
                                                           collisionFreeConstraint,
                                                           armSpace,
                                                           armSkeleton);
            // TSRMotionNode
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
        }


        auto adaPlanConf = Eigen::Vector2d();
        adaPlanConf << 0.5, 0.5;
        auto adaExecConf = Eigen::Vector2d();
        adaExecConf << 1.6, 1.6;
        auto motion2 = std::make_shared<ConfMotionNode>(adaPlanConf, adaExecConf, handSpace, handSkeleton);
        ROS_INFO("Grabbing...Closing gripper");
        // ConfMotionNode
        motion2->plan(robot->m_adaPlan, robot->m_adaExec);

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
                motion->plan(robot->m_adaPlan, robot->m_adaExec);
            } else {
                for (auto &other : agents) {
                    if (other.first != m_pid && other.second->getType() == "robot") {
                        auto robot2 = std::dynamic_pointer_cast<Robot, Agent>(other.second);
                        auto robotArm2 = robot2->getArm();
                        auto armSkeleton2 = robotArm2->getMetaSkeleton();
                        auto armSpace2 = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
                                armSkeleton2.get());
                        if (robot2->getHand()->isGrabbing(m_toGrab) == 0) {
                            ROS_INFO("Need to let the other robot ungrabs this object");
                            auto
                                    motion =
                                    std::make_shared<GrabMotionNode>(world->getSkeleton(m_toGrab), false, armSpace2,
                                                                     armSkeleton2);
                            motion->plan(robot2->m_adaPlan, robot->m_adaExec);
                        } else {
                            break;
                        }
                    }
                }
            }
        }
        auto motion3 = std::make_shared<GrabMotionNode>(world->getSkeleton(m_toGrab), true, armSpace, armSkeleton);
        ROS_INFO("Grabbing...About to start");
        motion3->plan(robot->m_adaPlan, robot->m_adaExec);
        ROS_INFO("Grabbing...Should have connected");

        m_ifExecuted = true;
    }
}
