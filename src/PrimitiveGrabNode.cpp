//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveGrabNode.h"
#include "wecook/utils.h"
#include "wecook/Agent.h"
#include "wecook/Robot.h"

using namespace wecook;

void PrimitiveGrabNode::execute(std::map<std::string, std::shared_ptr<Agent>> &agents,
                                std::shared_ptr<ObjectMgr> &objMgr,
                                std::shared_ptr<ContainingMap> &containingMap) {
  auto agent = agents[m_pid];

  // since every primitive action node only involves one agent
  if (agent->getType() == "human") {
    // TODO human execute
  } else if (agent->getType() == "robot") {
    auto robot = std::dynamic_pointer_cast<Robot, Agent>(agent);
    auto robotArm = robot->getArm();
    auto robotHand = robot->getHand();
    auto armSkeleton = robotArm->getMetaSkeleton();
    auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
    std::vector<std::shared_ptr<MotionNode>> motionSeq;



  }
}
