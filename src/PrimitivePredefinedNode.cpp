//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitivePredefinedNode.h"
#include "wecook/Robot.h"
#include "wecook/RelativeIKMotionNode.h"

using namespace wecook;

void PrimitivePredefinedNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
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
    dart::dynamics::BodyNode *bn = nullptr;
    if (m_manipulatedObj == "end-effector") {
      bn = robotHand->getEndEffectorBodyNode();
    } else {
      bn = objMgr->getObjBodyNode(m_manipulatedObj);
    }

    if (m_motionType == "cut") {
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(0., 0., -0.001);
        auto motion4 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    30,
                                                    armSpace,
                                                    armSkeleton);
        motion4->plan(robot->m_ada);

        delta_x << 0., 0., 0.001;
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    30,
                                                    armSpace,
                                                    armSkeleton);
        motion5->plan(robot->m_ada);
      }
    } else if (m_motionType == "stir") {
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d delta_x(-0.001, 0., -0.);
        auto motion5 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    30,
                                                    armSpace,
                                                    armSkeleton);
        motion5->plan(robot->m_ada);

        delta_x << +0.001, 0., 0.00;
        auto motion6 =
            std::make_shared<LinearDeltaMotionNode>(bn,
                                                    delta_x,
                                                    dart::dynamics::Frame::World(),
                                                    30,
                                                    armSpace,
                                                    armSkeleton);
        motion6->plan(robot->m_ada);
      }
    } else if (m_motionType == "transfer1") {
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear() << 0.5000, 0.500000, 0.7071, 0.5000, 0.5000, -0.7071, -0.7071, 0.7071, 0.0000;
      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                           rotatePose,
                                                           dart::dynamics::Frame::World(),
                                                           armSpace,
                                                           armSkeleton);
      motion->plan(robot->m_ada);
    } else if (m_motionType == "transfer2") {
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear()
          <<
          1., 0., 0.,
          0., 0.7071068, -0.7071068,
          0., 0.7071068, 0.7071068;
      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                           rotatePose,
                                                           dart::dynamics::Frame::World(),
                                                           armSpace,
                                                           armSkeleton);
      motion->plan(robot->m_ada);
      Eigen::Vector3d delta_x(0., 0., 0.001);
      auto motion2 = std::make_shared<LinearDeltaMotionNode>(bn,
                                                             delta_x,
                                                             dart::dynamics::Frame::World(),
                                                             200,
                                                             armSpace,
                                                             armSkeleton);
      motion2->plan(robot->m_ada);
    } else if (m_motionType == "transfer3") {
      auto rotatePose = Eigen::Isometry3d::Identity();
      rotatePose.linear() <<
                          0., -1., 0.,
                          1., 0., 0.,
                          0., 0, 1.;
      auto motion = std::make_shared<RelativeIKMotionNode>(bn,
                                                            rotatePose,
                                                            dart::dynamics::Frame::World(),
                                                            armSpace,
                                                            armSkeleton);
      motion->plan(robot->m_ada);
    }
    m_ifExecuted = true;
  }

}