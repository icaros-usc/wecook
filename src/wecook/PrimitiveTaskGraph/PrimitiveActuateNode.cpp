//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveTaskGraph/PrimitiveActuateNode.h"
#include "wecook/Agents/Robot.h"
#include "wecook/LowLevelMotionNodes/RelativeIKMotionNode.h"
#include "wecook/LowLevelMotionNodes/ConnMotionNode.h"

using namespace wecook;

void PrimitiveActuateNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                   std::shared_ptr<ObjectMgr> &objMgr,
                                   std::shared_ptr<ContainingMap> &containingMap,
                                   Result *result) {
  auto agent = agents[m_pid];

  if (agent->getType() == "human") {
    // TODO human execute
    waitForUser("Please do " + m_motionType + "...");

    m_ifExecuted = true;
  } else if (agent->getType() == "robot") {
    auto robot = std::dynamic_pointer_cast<Robot, Agent>(agent);
    auto robotArm = robot->getArm();
    auto robotHand = robot->getHand();
    auto armSkeleton = robotArm->getMetaSkeleton();
    auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
    auto handSkeleton = robotHand->getMetaSkeleton();
    auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
    dart::dynamics::BodyNode *bn = nullptr;
    if (m_manipulatedObj == "end-effector") {
      bn = robotHand->getEndEffectorBodyNode();
    } else {
      bn = objMgr->getObjBodyNode(m_manipulatedObj);
    }

    // Create the collision free constraints
    auto collisionDetector = dart::collision::FCLCollisionDetector::create();
    std::shared_ptr<dart::collision::CollisionGroup>
        armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), handSkeleton.get(), bn);
    auto envCollisionGroup = objMgr->createCollisionGroupExceptFoodAndToMoveObj(m_manipulatedObj, collisionDetector);
    // We also need to add skeletons of other agents
    for (const auto &itr : agents) {
      if (itr.first == m_pid) {
        continue;
      } else if (itr.second->getType() == "robot") {
        auto otherArmSkeleton = std::dynamic_pointer_cast<Robot, Agent>(itr.second)->getArm()->getMetaSkeleton();
        auto otherHandSkeleton = std::dynamic_pointer_cast<Robot, Agent>(itr.second)->getHand()->getMetaSkeleton();
        envCollisionGroup->addShapeFramesOf(otherArmSkeleton.get(), otherHandSkeleton.get());
      }
    }
    std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
        std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
    collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);

    if (m_motionType == "cut") {
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(0., 0., -0.003);
        auto motion4 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion4->plan(robot->m_ada, robot->m_adaImg);

        delta_x << 0., 0., 0.003;
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion5->plan(robot->m_ada, robot->m_adaImg);
      }
    } else if (m_motionType == "stir") {
      Eigen::Vector3d delta_x(-0.003, 0., -0.);
      delta_x << +0.003, 0., 0.00;
      auto motion6 =
          std::make_shared<LinearDeltaMotionNode>(bn,
                                                  delta_x,
                                                  dart::dynamics::Frame::World(),
                                                  10,
                                                  armSpace,
                                                  armSkeleton,
                                                  collisionFreeConstraint);
      motion6->plan(robot->m_ada, robot->m_adaImg);
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(-0.003, 0., -0.);
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    20,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion5->plan(robot->m_ada, robot->m_adaImg);

        delta_x << +0.003, 0., 0.00;
        auto motion6 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    20,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion6->plan(robot->m_ada, robot->m_adaImg);
      }
      delta_x << -0.003, 0., 0.00;
      motion6 =
          std::make_shared<LinearDeltaMotionNode>(bn,
                                                  delta_x,
                                                  dart::dynamics::Frame::World(),
                                                  10,
                                                  armSpace,
                                                  armSkeleton,
                                                  collisionFreeConstraint);
      motion6->plan(robot->m_ada, robot->m_adaImg);
    } else if (m_motionType == "transfer1") {
      // This motion is used to do transfer by container
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear() << 0.5000, 0.500000, 0.7071, 0.5000, 0.5000, -0.7071, -0.7071, 0.7071, 0.0000;

      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                           rotatePose,
                                                           dart::dynamics::Frame::World(),
                                                           armSpace,
                                                           armSkeleton,
                                                           collisionFreeConstraint);
      motion->plan(robot->m_ada, robot->m_adaImg);
    } else if (m_motionType == "transfer2") {
      // this action is used to grasp some food with the tool
      auto action = m_metaActuateInfo.getAction();
      auto ingredientName = action.get_ingredients()[0];
      auto oldLocationName = action.get_locations()[0];
      auto toolName = action.get_tool();

      auto world = robot->getWorld();
      auto ingredientNode = world->getSkeleton(ingredientName);
      auto oldLocation = world->getSkeleton(oldLocationName);
      auto toolNode = world->getSkeleton(toolName);
      // ungrab for connect
      robotHand->ungrab();
      auto motionunconnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                              oldLocation,
                                                              oldLocationName,
                                                              ingredientName,
                                                              containingMap,
                                                              false,
                                                              armSpace,
                                                              armSkeleton);
      motionunconnect->plan(robot->m_ada, robot->m_adaImg);

      auto motionConnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                            toolNode,
                                                            toolName,
                                                            ingredientName,
                                                            containingMap,
                                                            true,
                                                            armSpace,
                                                            armSkeleton);
      motionConnect->plan(robot->m_ada, robot->m_adaImg);
      // grab again
      robotHand->grab(toolNode);
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear()
          <<
          1., 0., 0.,
          0., 0.50000, -0.866,
          0., 0.8666, 0.5;

      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                           rotatePose,
                                                           dart::dynamics::Frame::World(),
                                                           armSpace,
                                                           armSkeleton,
                                                           collisionFreeConstraint);
      motion->plan(robot->m_ada, robot->m_adaImg);
      Eigen::Vector3d delta_x(0., 0., 0.005);
      auto motion2 = std::make_shared<LinearDeltaMotionNode>(bn,
                                                             delta_x,
                                                             dart::dynamics::Frame::World(),
                                                             10,
                                                             armSpace,
                                                             armSkeleton,
                                                             collisionFreeConstraint);
      motion2->plan(robot->m_ada, robot->m_adaImg);
    } else if (m_motionType == "transfer3") {
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear() <<
                          1., 0., 0.,
          0., 0.7071055, 0.7071081,
          0., -0.7071081, 0.7071055;

      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                           rotatePose,
                                                           dart::dynamics::Frame::World(),
                                                           armSpace,
                                                           armSkeleton,
                                                           collisionFreeConstraint);
      motion->plan(robot->m_ada, robot->m_adaImg);

      // this action is used to release some food from the tool
      auto action = m_metaActuateInfo.getAction();
      auto ingredientName = action.get_ingredients()[0];
      auto newLocationName = action.get_locations()[1];
      auto toolName = action.get_tool();

      auto world = robot->getWorld();
      auto ingredientNode = world->getSkeleton(ingredientName);
      auto newLocation = world->getSkeleton(newLocationName);
      auto toolNode = world->getSkeleton(toolName);
      // ungrab for unconnect
      robotHand->ungrab();
      // unconnect food from the tool
      auto motionunconnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                              toolNode,
                                                              toolName,
                                                              ingredientName,
                                                              containingMap,
                                                              false,
                                                              armSpace,
                                                              armSkeleton);
      motionunconnect->plan(robot->m_ada, robot->m_adaImg);
      // connect food to the tool
      auto motionConnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                            newLocation,
                                                            newLocationName,
                                                            ingredientName,
                                                            containingMap,
                                                            true,
                                                            armSpace,
                                                            armSkeleton);
      motionConnect->plan(robot->m_ada, robot->m_adaImg);
      // grab the tool again
      robotHand->grab(toolNode);
    } else if (m_motionType == "close") {
      auto conf = Eigen::Vector2d();
      conf << 1., 1.;
      auto motion = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
      ROS_INFO("Fully closing gripper");
      motion->plan(robot->m_ada, robot->m_adaImg);
    } else if (m_motionType == "roll") {
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(-0.003, 0., -0.);
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion5->plan(robot->m_ada, robot->m_adaImg);

        delta_x << +0.003, 0., 0.00;
        auto motion6 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion6->plan(robot->m_ada, robot->m_adaImg);
      }
    } else if (m_motionType == "heat") {
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(-0.003, 0., -0.);
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion5->plan(robot->m_ada, robot->m_adaImg);

        delta_x << +0.003, 0., 0.00;
        auto motion6 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion6->plan(robot->m_ada, robot->m_adaImg);
      }
    } else if (m_motionType == "feeding1") {
      // In feeding1, we do scoop up

      // 4) rotate

      // this action is used to grasp some food with the tool
      auto action = m_metaActuateInfo.getAction();
      auto ingredientName = action.get_ingredients()[0];
      auto oldLocationName = action.get_locations()[0];
      auto toolName = action.get_tool();
      auto world = robot->getWorld();
      auto ingredientNode = world->getSkeleton(ingredientName);
      auto oldLocation = world->getSkeleton(oldLocationName);
      auto toolNode = world->getSkeleton(toolName);

      // 1) First go down
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(0., 0., -0.003);
        auto motion4 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion4->plan(robot->m_ada, robot->m_adaImg);
      }

      // 2) rotate
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear()
          <<
          1., 0., 0.,
          0., 0.50000, -0.866,
          0., 0.8666, 0.5;

      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                           rotatePose,
                                                           dart::dynamics::Frame::World(),
                                                           armSpace,
                                                           armSkeleton,
                                                           collisionFreeConstraint);
      motion->plan(robot->m_ada, robot->m_adaImg);

      // 3) move horizontally to get more food
      for (int j = 0; j < 1; j++) {
        Eigen::Vector3d delta_x(0., 0.003, 0.);
        auto motion4 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton,
                                                    collisionFreeConstraint);
        motion4->plan(robot->m_ada, robot->m_adaImg);
      }

      // 4) rotate again to be more lean
      auto rotatePose2 = Eigen::Isometry3d::Identity();
      rotatePose2.linear()
          <<
          1., 0., 0.,
          0., 0.707, -0.707,
          0., 0.707, 0.707;

      auto motion3 = std::make_shared<RelativeIKMotionNode>(bn,
                                                            rotatePose2,
                                                            dart::dynamics::Frame::World(),
                                                            armSpace,
                                                            armSkeleton,
                                                            collisionFreeConstraint);
      motion3->plan(robot->m_ada, robot->m_adaImg);

      // 5) grab
      // ungrab for connect
      robotHand->ungrab();
      auto motionunconnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                              oldLocation,
                                                              oldLocationName,
                                                              ingredientName,
                                                              containingMap,
                                                              false,
                                                              armSpace,
                                                              armSkeleton);
      motionunconnect->plan(robot->m_ada, robot->m_adaImg);

      auto motionConnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                            toolNode,
                                                            toolName,
                                                            ingredientName,
                                                            containingMap,
                                                            true,
                                                            armSpace,
                                                            armSkeleton);
      motionConnect->plan(robot->m_ada, robot->m_adaImg);
      // grab again
      robotHand->grab(toolNode);

      // 6) go up
      Eigen::Vector3d delta_x(0., 0., 0.005);
      auto motion2 = std::make_shared<LinearDeltaMotionNode>(bn,
                                                             delta_x,
                                                             dart::dynamics::Frame::World(),
                                                             15,
                                                             armSpace,
                                                             armSkeleton,
                                                             collisionFreeConstraint);
      motion2->plan(robot->m_ada, robot->m_adaImg);

    } else if (m_motionType == "feeding2") {
      // When we do feeding, we just hang around the mouth
      // TODO tracking mouth
      ROS_INFO("Feeding...");
      ros::Duration(10).sleep();

      // this action is used to release some food from the tool
      auto action = m_metaActuateInfo.getAction();
      auto ingredientName = action.get_ingredients()[0];
      auto newLocationName = action.get_locations()[1];
      auto toolName = action.get_tool();

      auto world = robot->getWorld();
      auto ingredientNode = world->getSkeleton(ingredientName);
      auto newLocation = world->getSkeleton(newLocationName);
      auto toolNode = world->getSkeleton(toolName);
      // ungrab for connect
      robotHand->ungrab();
      auto motionunconnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                              toolNode,
                                                              toolName,
                                                              ingredientName,
                                                              containingMap,
                                                              false,
                                                              armSpace,
                                                              armSkeleton);
      motionunconnect->plan(robot->m_ada, robot->m_adaImg);

      auto motionConnect = std::make_shared<ConnMotionNode>(ingredientNode,
                                                            newLocation,
                                                            newLocationName,
                                                            ingredientName,
                                                            containingMap,
                                                            true,
                                                            armSpace,
                                                            armSkeleton);
      motionConnect->plan(robot->m_ada, robot->m_adaImg);
      // grab again
      robotHand->grab(toolNode);
    }
    m_ifExecuted = true;
  }

}