//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveActuateNode.h"
#include "wecook/Robot.h"
#include "wecook/RelativeIKMotionNode.h"
#include "wecook/ConnMotionNode.h"

using namespace wecook;

void PrimitiveActuateNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                   std::shared_ptr<ObjectMgr> &objMgr,
                                   std::shared_ptr<ContainingMap> &containingMap) {
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

    if (m_motionType == "cut") {
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(0., 0., -0.003);
        auto motion4 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton);
        motion4->plan(robot->m_ada, robot->m_adaImg);

        delta_x << 0., 0., 0.003;
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton);
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
                                                  armSkeleton);
      motion6->plan(robot->m_ada, robot->m_adaImg);
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(-0.003, 0., -0.);
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    20,
                                                    armSpace,
                                                    armSkeleton);
        motion5->plan(robot->m_ada, robot->m_adaImg);

        delta_x << +0.003, 0., 0.00;
        auto motion6 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    20,
                                                    armSpace,
                                                    armSkeleton);
        motion6->plan(robot->m_ada, robot->m_adaImg);
      }
      delta_x << -0.003, 0., 0.00;
      motion6 =
          std::make_shared<LinearDeltaMotionNode>(bn,
                                                  delta_x,
                                                  dart::dynamics::Frame::World(),
                                                  10,
                                                  armSpace,
                                                  armSkeleton);
      motion6->plan(robot->m_ada, robot->m_adaImg);
    } else if (m_motionType == "transfer1") {
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear() << 0.5000, 0.500000, 0.7071, 0.5000, 0.5000, -0.7071, -0.7071, 0.7071, 0.0000;
      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                           rotatePose,
                                                           dart::dynamics::Frame::World(),
                                                           armSpace,
                                                           armSkeleton);
      motion->plan(robot->m_ada, robot->m_adaImg);
    } else if (m_motionType == "transfer2") {
      // hacking
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
                                                           armSkeleton);
      motion->plan(robot->m_ada, robot->m_adaImg);
      Eigen::Vector3d delta_x(0., 0., 0.005);
      auto motion2 = std::make_shared<LinearDeltaMotionNode>(bn,
                                                             delta_x,
                                                             dart::dynamics::Frame::World(),
                                                             10,
                                                             armSpace,
                                                             armSkeleton);
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
                                                           armSkeleton);
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
                                                    armSkeleton);
        motion5->plan(robot->m_ada, robot->m_adaImg);

        delta_x << +0.003, 0., 0.00;
        auto motion6 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton);
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
                                                    armSkeleton);
        motion5->plan(robot->m_ada, robot->m_adaImg);

        delta_x << +0.003, 0., 0.00;
        auto motion6 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    10,
                                                    armSpace,
                                                    armSkeleton);
        motion6->plan(robot->m_ada, robot->m_adaImg);
      }
    } else if (m_motionType == "feeding") {
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