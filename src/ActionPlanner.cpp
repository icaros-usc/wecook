//
// Created by hejia on 8/2/19.
//
#include <aikido/common/PseudoInverse.hpp>

#include "wecook/tsr/knife.h"
#include "wecook/ActionPlanner.h"
#include "wecook/strutils.h"
#include "wecook/ConfPreCondition.h"
#include "wecook/ConnPreCondition.h"
#include "wecook/PrimitiveGraspNode.h"
#include "wecook/PrimitiveEngageNode.h"
#include "wecook/PrimitiveActuateNode.h"
#include "wecook/PrimitivePlaceNode.h"
#include "wecook/Robot.h"

using namespace wecook;

void ActionPlanner::compile(std::shared_ptr<TaskGraph> &taskGraph,
                            std::map<std::string, std::shared_ptr<Agent>> &agents,
                            std::shared_ptr<ContainingMap> &containingMap,
                            std::shared_ptr<ObjectMgr> &objectMgr) {
  // for each actionNode we first plan primitive task graph for each of them
  auto actionNodes = taskGraph->getNodes();
  for (auto &actionNode : actionNodes) {
    plan(actionNode, agents, containingMap, objectMgr);
  }
  taskGraph->merge();
}

void ActionPlanner::plan(ActionNode *actionNode,
                         std::map<std::string, std::shared_ptr<Agent>> &agents,
                         std::shared_ptr<ContainingMap> &containingMap,
                         std::shared_ptr<ObjectMgr> &objectMgr) {
  // TODO add more skills
  if (actionNode->getAction().get_verb() == "cut") {
    planCut(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb() == "stir") {
    planStir(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb() == "handover") {
    planHandover(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb() == "transfer") {
    planTransfer(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb().find('_') != std::string::npos) {
    planHolding(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb() == "roll") {
    planRoll(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb() == "heat") {
    planHeat(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb() == "sprinkle") {
    planSprinkle(actionNode, agents, containingMap, objectMgr);
  } else if (actionNode->getAction().get_verb() == "squeeze") {
    planSqueeze(actionNode, agents, containingMap, objectMgr);
  }
}

void ActionPlanner::planRoll(ActionNode *actionNode,
                             std::map<std::string, std::shared_ptr<Agent>> &agents,
                             std::shared_ptr<ContainingMap> &containingMap,
                             std::shared_ptr<ObjectMgr> &objectMgr) {
  // To do rolling, we have 4 steps: grab tool, move tool to start position,
  // do predefined rolling motion, place tool back
  // 1) create grab node
  // create grab node
  auto grabPose = std::make_shared<aikido::constraint::dart::TSR>();
  grabPose->mTw_e.translation() = Eigen::Vector3d(0., 0.01, 0.08);
  Eigen::Matrix3d rot;
  rot <<
      1., 0., 0.,
      0., -1., 0.,
      0., 0., -1;
  Eigen::Matrix3d rot2;
  rot2 <<
       0., 0., -1.,
      0., 1., 0.,
      1., 0., 0.;
  grabPose->mTw_e.linear() = rot2 * rot;
  auto epsilon = 0.01;
  grabPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto toolName = actionNode->getAction().get_tool();
  auto pid = actionNode->getAction().get_pids()[0];
  auto agent = agents[pid];

  auto grabNode = std::make_shared<PrimitiveGraspNode>(grabPose, toolName, toolName, pid, toolName, "", true, false);

  // 2) create move to node
  // create start pose
  auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
  targetPose->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.03);
  targetPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto toRollName = actionNode->getAction().get_ingredients()[0];
  auto moveToNode =
      std::make_shared<PrimitiveEngageNode>(targetPose, toolName, toRollName, pid, toolName, "", false, false);

  // 3) create predefined rolling node
  auto predefinedNode =
      std::make_shared<PrimitiveActuateNode>(pid, "end-effector", "roll", toolName, "", false, false);

  // 4) create place back node
  auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
  auto translation = objectMgr->getObjTransform(toolName).translation();
  placePose->mTw_e.translation() = translation - Eigen::Vector3d(0.5, 0.0, 0.);
  placePose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto
      placeNode =
      std::make_shared<PrimitivePlaceNode>(placePose,
                                           toolName,
                                           "table0",
                                           pid,
                                           "",
                                           toolName,
                                           false,
                                           true);

  // connect these body nodes
  grabNode->addChild(moveToNode);
  moveToNode->addFather(grabNode);
  moveToNode->addChild(predefinedNode);
  predefinedNode->addFather(moveToNode);
  predefinedNode->addChild(placeNode);
  placeNode->addFather(predefinedNode);

  // build sub primitive task graph for this action node
  PrimitiveTaskGraph ptg{};
  ptg.addNode(grabNode);
  ptg.addNode(moveToNode);
  ptg.addNode(predefinedNode);
  ptg.addNode(placeNode);

  actionNode->setPrimitiveTaskGraph(ptg);
}

void ActionPlanner::planHeat(ActionNode *actionNode,
                             std::map<std::string, std::shared_ptr<Agent>> &agents,
                             std::shared_ptr<ContainingMap> &containingMap,
                             std::shared_ptr<ObjectMgr> &objectMgr) {
  // TO do heating, we assume food is already in some container that can be heated.
  // The action heating is used to make sure the food is not overheated.
  // 1) create grab node
  // create grab pose
  auto grabPose = std::make_shared<aikido::constraint::dart::TSR>();
  grabPose->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.10);
  Eigen::Matrix3d rot;
  rot <<
      1., 0., 0.,
      0., -1., 0.,
      0., 0., -1;
  grabPose->mTw_e.linear() = rot;
  auto epsilon = 0.02;
  grabPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto toolName = actionNode->getAction().get_tool();
  auto pid = actionNode->getAction().get_pids()[0];
  auto agent = agents[pid];

  auto grabNode = std::make_shared<PrimitiveGraspNode>(grabPose, toolName, toolName, pid, toolName, "", true, false);

  // 2) create move to node
  // create start pose
  auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
  targetPose->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.03);
  targetPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto toHeatName = actionNode->getAction().get_ingredients()[0];
  auto moveToNode =
      std::make_shared<PrimitiveEngageNode>(targetPose, toolName, toHeatName, pid, toolName, "", false, false);

  // 3) create predefined heating node
  auto predefinedNode =
      std::make_shared<PrimitiveActuateNode>(pid, "end-effector", "heat", toolName, "", false, false);

  // 4) create place back node
  auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
  auto translation = objectMgr->getObjTransform(toolName).translation();
  placePose->mTw_e.translation() = translation - Eigen::Vector3d(0.5, 0.0, 0.);
  placePose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto
      placeNode =
      std::make_shared<PrimitivePlaceNode>(placePose,
                                           toolName,
                                           "table0",
                                           pid,
                                           "",
                                           toolName,
                                           false,
                                           true);

  // connect these body nodes
  grabNode->addChild(moveToNode);
  moveToNode->addFather(grabNode);
  moveToNode->addChild(predefinedNode);
  predefinedNode->addFather(moveToNode);
  predefinedNode->addChild(placeNode);
  placeNode->addFather(predefinedNode);

  // build sub primitive task graph for this action node
  PrimitiveTaskGraph ptg{};
  ptg.addNode(grabNode);
  ptg.addNode(moveToNode);
  ptg.addNode(predefinedNode);
  ptg.addNode(placeNode);

  actionNode->setPrimitiveTaskGraph(ptg);

}

void ActionPlanner::planCut(ActionNode *actionNode,
                            std::map<std::string, std::shared_ptr<Agent>> &agents,
                            std::shared_ptr<ContainingMap> &containingMap,
                            std::shared_ptr<ObjectMgr> &objectMgr) {
  // To do cutting, we have 4 steps: grab tool, move tool to start position,
  // do predefined cutting motion, place tool back
  // 1) create grab node
  // create grab pose
  auto grabPose = std::make_shared<aikido::constraint::dart::TSR>();
  grabPose->mTw_e.translation() = Eigen::Vector3d(0.04, 0, 0.02);
  Eigen::Matrix3d rot;
  rot <<
      1., 0., 0.,
      0., -1., 0.,
      0., 0., -1;
  grabPose->mTw_e.linear() = rot;
  auto epsilon = 0.02;
  grabPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto toolName = actionNode->getAction().get_tool();
  auto pid = actionNode->getAction().get_pids()[0];
  auto agent = agents[pid];

  auto grabNode =
      std::make_shared<PrimitiveGraspNode>(grabPose, toolName, toolName, pid, toolName, "", true, false);

  // 2) create move to node
  // create start pose
  auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
  targetPose->mTw_e.translation() = Eigen::Vector3d(-0.12, 0., 0.06);
  targetPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto toCutName = actionNode->getAction().get_ingredients()[0];
  auto moveToNode =
      std::make_shared<PrimitiveEngageNode>(targetPose,
                                            toolName,
                                            toCutName,
                                            pid,
                                            toolName,
                                            "",
                                            false,
                                            false);

  // 3) create predefined cutting node
  auto predefinedNode =
      std::make_shared<PrimitiveActuateNode>(pid, "end-effector", "cut", toolName, "", false, false);

  // 4) create place back node
  auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
  // get the original place of the tool
  auto translation = objectMgr->getObjTransform(toolName).translation();
  epsilon = 0.01;
  placePose->mTw_e.translation() = translation - Eigen::Vector3d(0.5, 0.0, 0.);
  placePose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto
      placeNode =
      std::make_shared<PrimitivePlaceNode>(placePose,
                                           toolName,
                                           "table0",
                                           pid,
                                           "",
                                           toolName,
                                           false,
                                           true);

  // connect these body nodes
  grabNode->addChild(moveToNode);
  moveToNode->addFather(grabNode);
  moveToNode->addChild(predefinedNode);
  predefinedNode->addFather(moveToNode);
  predefinedNode->addChild(placeNode);
  placeNode->addFather(predefinedNode);

  // build sub primitive task graph for this action node
  PrimitiveTaskGraph ptg{};
  ptg.addNode(grabNode);
  ptg.addNode(moveToNode);
  ptg.addNode(predefinedNode);
  ptg.addNode(placeNode);

  actionNode->setPrimitiveTaskGraph(ptg);
}

void ActionPlanner::planStir(ActionNode *actionNode,
                             std::map<std::string, std::shared_ptr<Agent>> &agents,
                             std::shared_ptr<ContainingMap> &containingMap,
                             std::shared_ptr<ObjectMgr> &objectMgr) {
  // To do stiring, we have 4 steps: grab tool, move tool to start position,
  // do predefined stiring motion, place tool back
  // 1) create grab node
  // create grab pose
  auto grabPose = std::make_shared<aikido::constraint::dart::TSR>();
  grabPose->mTw_e.translation() = Eigen::Vector3d(0.0, 0., 0.15);
  Eigen::Matrix3d rot;
  rot <<
      1., 0., 0.,
      0., -1., 0.,
      0., 0., -1;
  grabPose->mTw_e.linear() = rot;
  auto epsilon = 0.01;
  grabPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto toolName = actionNode->getAction().get_tool();
  auto pid = actionNode->getAction().get_pids()[0];
  auto grabNode =
      std::make_shared<PrimitiveGraspNode>(grabPose, toolName, toolName, pid, toolName, "", true, false);

  // 2) create move to node
  // create start pose
  auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
  targetPose->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.06);
  targetPose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI, M_PI;
  auto toStirName = actionNode->getAction().get_location()[0];
  auto moveToNode =
      std::make_shared<PrimitiveEngageNode>(targetPose,
                                            toolName,
                                            toStirName,
                                            pid,
                                            toolName,
                                            "",
                                            false,
                                            false);

  // 3) create predefined stirring node
  auto predefinedNode =
      std::make_shared<PrimitiveActuateNode>(pid, "end-effector", "stir", toolName, "", false, false);

  // 4) create place back node
  auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
  auto translation = objectMgr->getObjTransform(toolName).translation();
  placePose->mTw_e.translation() = translation - Eigen::Vector3d(0.5, 0.0, 0.);
  placePose->mTw_e.linear() = objectMgr->getObjTransform(toolName).linear();
  placePose->mBw
      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  auto
      placeNode =
      std::make_shared<PrimitivePlaceNode>(placePose,
                                           toolName,
                                           "table0",
                                           pid,
                                           "",
                                           toolName,
                                           false,
                                           true);

  // connect these nodes
  grabNode->addChild(moveToNode);
  moveToNode->addFather(grabNode);
  moveToNode->addChild(predefinedNode);
  predefinedNode->addFather(moveToNode);
  predefinedNode->addChild(placeNode);
  placeNode->addFather(predefinedNode);

  // build sub primitive task graph for this action node
  PrimitiveTaskGraph ptg{};
  ptg.addNode(grabNode);
  ptg.addNode(moveToNode);
  ptg.addNode(predefinedNode);
  ptg.addNode(placeNode);

  actionNode->setPrimitiveTaskGraph(ptg);
}

void ActionPlanner::planTransfer(ActionNode *actionNode,
                                 std::map<std::string,
                                          std::shared_ptr<Agent>> &agents,
                                 std::shared_ptr<ContainingMap> &containingMap,
                                 std::shared_ptr<ObjectMgr> &objectMgr) {
  if (actionNode->getAction().get_tool() == "hand") {
    // To do transfer object by hand, we have 2 steps: grab object, place object to new position
    // 1) create grab node
    // create grab pose
    auto grabPose = std::make_shared<aikido::constraint::dart::TSR>();
    Eigen::Matrix3d rot;
    rot <<
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;;
    grabPose->mTw_e.linear() = rot;
    grabPose->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.10);
    auto epsilon = 0.02;
    grabPose->mBw
        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI, M_PI;
    auto objectName = actionNode->getAction().get_ingredients()[0];
    auto pid = actionNode->getAction().get_pids()[0];
    auto grabNode =
        std::make_shared<PrimitiveGraspNode>(grabPose, objectName, objectName, pid, objectName, "", true, false);

    // 2) create place node
    auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
    auto newLocationName = actionNode->getAction().get_location()[1];
    placePose->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.05);
    placePose->mBw
        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI / 8, M_PI / 8, -M_PI / 8, M_PI
        / 8, -M_PI, M_PI;
    auto placeNode =
        std::make_shared<PrimitivePlaceNode>(placePose,
                                             objectName,
                                             newLocationName,
                                             pid,
                                             "",
                                             objectName,
                                             false,
                                             true);

    // connect these nodes
    grabNode->addChild(placeNode);
    placeNode->addFather(grabNode);

    // build sub primitive task graph for this action node
    PrimitiveTaskGraph ptg{};
    ptg.addNode(grabNode);
    ptg.addNode(placeNode);

    actionNode->setPrimitiveTaskGraph(ptg);

    return;
  } else if (actionNode->getAction().get_tool() == actionNode->getAction().get_location()[0]) {
    // To do transfer object by old container, we have 2 steps: grab old container, move container to start position,
    // do predefined motion
    // 1) create grab node
    // create grab pose
    auto grabPose = std::make_shared<aikido::constraint::dart::TSR>();
    Eigen::Matrix3d rot;
    rot <<
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    Eigen::Matrix3d rot2;
    rot2 <<
         0.7071072, 0.7071063, 0.0000000,
        -0.7071063, 0.7071072, 0.0000000,
        0.0000000, 0.0000000, 1.0000000;
    grabPose->mTw_e.linear() = rot2 * rot;
    grabPose->mTw_e.translation() = Eigen::Vector3d(0.0, 0.06, 0.08);
    grabPose->mBw << -0.01, 0.01, 0., 0., -0.01, 0.01, -M_PI / 8, M_PI / 8, -M_PI / 8, M_PI / 8, -0.02, 0.02;
    auto objectName = actionNode->getAction().get_location()[0];
    auto pid = actionNode->getAction().get_pids()[0];
    auto grabNode =
        std::make_shared<PrimitiveGraspNode>(grabPose, objectName, objectName, pid, objectName, "", true, false);

    // 2) create move to node
    auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
    targetPose->mTw_e.translation() = Eigen::Vector3d(-0.0, 0.10, 0.2);
    auto epsilon = 0.02;
    targetPose->mBw << -epsilon / 2, epsilon / 2, -epsilon / 2, epsilon
        / 2, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto newLocationName = actionNode->getAction().get_location()[1];
    auto moveToNode =
        std::make_shared<PrimitiveEngageNode>(targetPose,
                                              objectName,
                                              newLocationName,
                                              pid,
                                              objectName,
                                              "",
                                              false,
                                              false);

    // 3) create predefined pouring node
    auto predefinedNode =
        std::make_shared<PrimitiveActuateNode>(pid, objectName, "transfer1", objectName, "", false, false);

    // 4) create place back node
    auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
    placePose->mTw_e.translation() = Eigen::Vector3d(-0.1, -0.75, 0.73);
    placePose->mBw
        << -0.02, 0.02, -0.02, 0.02, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto placeNode =
        std::make_shared<PrimitivePlaceNode>(placePose,
                                             objectName,
                                             "table0",
                                             pid,
                                             "",
                                             objectName,
                                             false,
                                             true);

    // connect these nodes
    grabNode->addChild(moveToNode);
    moveToNode->addFather(grabNode);
    moveToNode->addChild(predefinedNode);
    predefinedNode->addFather(moveToNode);
    predefinedNode->addChild(placeNode);
    placeNode->addFather(predefinedNode);

    // build sub primitive task graph for this action node
    PrimitiveTaskGraph ptg{};
    ptg.addNode(grabNode);
    ptg.addNode(moveToNode);
    ptg.addNode(predefinedNode);
    ptg.addNode(placeNode);

    actionNode->setPrimitiveTaskGraph(ptg);

    return;
  } else {
    // To do transfer object by tool, we have 6 steps: grab tool, move tool to start position,
    // do predefined motion, move tool to new location, do predefined motion, place back tool
    // 1) create grab node
    // create grab pose
    auto grabPose = std::make_shared<aikido::constraint::dart::TSR>();
    grabPose->mTw_e.translation() = Eigen::Vector3d(-0.0, 0., 0.12);
    Eigen::Matrix3d rot;
    rot <<
        1., 0., 0.,
        0., -1., 0.,
        0., 0., -1;
    grabPose->mTw_e.linear() = rot;
    auto epsilon = 0.01;
    grabPose->mBw
        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto toolName = actionNode->getAction().get_tool();
    auto pid = actionNode->getAction().get_pids()[0];
    auto
        grabNode =
        std::make_shared<PrimitiveGraspNode>(grabPose, toolName, toolName, pid, toolName, "", true, false);

    // 2) create move to node
    // create start pose
    auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
    targetPose->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.03);
    targetPose->mBw
        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto oldLocationName = actionNode->getAction().get_location()[0];
    auto ingredientName = actionNode->getAction().get_ingredients()[0];
    auto moveToNode =
        std::make_shared<PrimitiveEngageNode>(targetPose,
                                              toolName,
                                              ingredientName,
                                              pid,
                                              toolName,
                                              "",
                                              false,
                                              false);

    // 3) create predefined motion node
    auto predefinedNode =
        std::make_shared<PrimitiveActuateNode>(pid, toolName, "transfer2", toolName, "", false, false);

    // 4) move food to new location
    auto targetPose2 = std::make_shared<aikido::constraint::dart::TSR>();
    Eigen::Matrix3d rot2;
    rot2 <<
        1., 0., 0.,
        0., 0.7071055, -0.7071081,
        0., 0.7071081, 0.7071055;
    targetPose2->mTw_e.linear() = rot2;
    targetPose2->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.06);
    targetPose2->mBw << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI / 8, M_PI / 8, -M_PI / 8, M_PI
        / 8, -M_PI / 8, M_PI / 8;
    auto newLocationName = actionNode->getAction().get_location()[1];
    auto moveToNode2 =
        std::make_shared<PrimitiveEngageNode>(targetPose2,
                                              toolName,
                                              newLocationName,
                                              pid,
                                              toolName,
                                              "",
                                              false,
                                              false);


    // 5) create predefined pouring node
    auto predefinedNode2 =
        std::make_shared<PrimitiveActuateNode>(pid, toolName, "transfer3", toolName, "", false, false);

    // 6) create place back node
    auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
    auto translation = objectMgr->getObjTransform(toolName).translation();
    placePose->mTw_e.translation() = translation - Eigen::Vector3d(0.5, 0.0, 0.);
    placePose->mBw
        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto
        placeNode =
        std::make_shared<PrimitivePlaceNode>(placePose,
                                             toolName,
                                             "table0",
                                             pid,
                                             "",
                                             toolName,
                                             false,
                                             true);

    // connect these nodes
    grabNode->addChild(moveToNode);
    moveToNode->addFather(grabNode);
    moveToNode->addChild(predefinedNode);
    predefinedNode->addFather(moveToNode);
    predefinedNode->addChild(moveToNode2);
    moveToNode2->addFather(predefinedNode);
    moveToNode2->addChild(predefinedNode2);
    predefinedNode2->addFather(moveToNode2);
    predefinedNode2->addChild(placeNode);
    placeNode->addFather(predefinedNode2);

    // build sub primitive task graph for this action node
    PrimitiveTaskGraph ptg{};
    ptg.addNode(grabNode);
    ptg.addNode(moveToNode);
    ptg.addNode(predefinedNode);
    ptg.addNode(moveToNode2);
    ptg.addNode(predefinedNode2);
    ptg.addNode(placeNode);

    actionNode->setPrimitiveTaskGraph(ptg);

    return;
  }
}

void ActionPlanner::planHandover(ActionNode *actionNode,
                                 std::map<std::string,
                                          std::shared_ptr<Agent>> &agents,
                                 std::shared_ptr<ContainingMap> &containingMap,
                                 std::shared_ptr<ObjectMgr> &objectMgr) {
  // To do handover, we have 4 steps to do: agent M grab->agentM move to->agentS grab->agent M place
  // 1) create agent M grab node
  auto grabPoseM = std::make_shared<aikido::constraint::dart::TSR>();
  Eigen::Matrix3d rot;
  rot <<
      1, 0, 0,
      0, -1, 0,
      0, 0, -1;;

  grabPoseM->mTw_e.linear() = rot;
  grabPoseM->mTw_e.translation() = Eigen::Vector3d(0.0, 0.0, 0.08);
  grabPoseM->mBw << -0.01, 0.01, 0., 0., -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
  auto objectName = actionNode->getAction().get_ingredients()[0];
  auto pidM = actionNode->getAction().get_pids()[0];
  auto agentM = agents[pidM];
  auto grabNodeM =
      std::make_shared<PrimitiveGraspNode>(grabPoseM, objectName, objectName, pidM, objectName, "", true, false);

  // 2) create move to node
  // create collaborative pose
  auto targetPoseM = std::make_shared<aikido::constraint::dart::TSR>();
  auto pidS = actionNode->getAction().get_pids()[1];
  auto agentS = agents[pidS];
  auto positionM = agentM->getPosition();
  auto positionS = agentS->getPosition();
  targetPoseM->mTw_e.translation() =
      Eigen::Vector3d(-0.2, (positionM[1] + positionS[1]) / 2, 0.85);
  targetPoseM->mBw << -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
  // TODO adding constraint
  auto moveToNodeM =
      std::make_shared<PrimitiveEngageNode>(targetPoseM,
                                            objectName,
                                            "table0",
                                            pidM,
                                            objectName,
                                            "",
                                            false,
                                            false);

  // 3) create agent S grab node
  auto grabPoseS = std::make_shared<aikido::constraint::dart::TSR>();
  Eigen::Matrix3d rot2;
  rot2 <<
       1, 0, 0,
      0, 0, -1.,
      0, 1, 0;
  Eigen::Matrix3d rot3;
  rot3 <<
       0, 0, -1,
      0, 1, 0.,
      1, 0, 0;
  grabPoseS->mTw_e.linear() = rot3 * rot2;
  grabPoseS->mTw_e.translation() = Eigen::Vector3d(-0.0, 0.08, 0.0);
  grabPoseS->mBw << -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
  auto grabNodeS =
      std::make_shared<PrimitiveGraspNode>(grabPoseS, objectName, objectName, pidS, objectName, "", true, true);

  // 4) create agent M place node
  auto placePoseM = std::make_shared<aikido::constraint::dart::TSR>();
  // TODO find a place to place object
  placePoseM->mBw << -0.005, 0.005, -0.005, 0.005, -0.005, 0.005, -0.005, 0.005, -0.005, 0.005, -0.005, 0.005;
  auto placeNodeM =
      std::make_shared<PrimitivePlaceNode>(placePoseM,
                                           objectName,
                                           objectName,
                                           pidM,
                                           "",
                                           objectName,
                                           false,
                                           true);

  // connect these body nodes
  grabNodeM->addChild(moveToNodeM);
  moveToNodeM->addFather(grabNodeM);
  moveToNodeM->addChild(grabNodeS);
  moveToNodeM->addChild(placeNodeM);
  grabNodeS->addFather(moveToNodeM);
  grabNodeS->addChild(placeNodeM);
  placeNodeM->addFather(moveToNodeM);
  placeNodeM->addFather(grabNodeS);

  // build sub primitive task graph for this action node
  PrimitiveTaskGraph ptg{};
  ptg.addNode(grabNodeM);
  ptg.addNode(moveToNodeM);
  ptg.addNode(grabNodeS);
  ptg.addNode(placeNodeM);

  actionNode->setPrimitiveTaskGraph(ptg);
}

void ActionPlanner::planHolding(wecook::ActionNode *actionNode,
                                std::map<std::string, std::shared_ptr<Agent>> &agents,
                                std::shared_ptr<ContainingMap> &containingMap,
                                std::shared_ptr<ObjectMgr> &objectMgr) {
  // first get collaborative action
  auto actionName = actionNode->getAction().get_verb();
  auto occurances = findAllOccurances(actionName, '_');
  auto collaborativeAction = actionName.substr(occurances.back() + 1);
  std::cout << collaborativeAction << std::endl;
  auto holdedObjectName = actionName.substr(occurances[0] + 1, occurances.back() - occurances[0] - 1);
  if (collaborativeAction == "transfer") {
    if (holdedObjectName == actionNode->getAction().get_location()[1]) {
      // To do collaborative transferring, we have 9 steps
      // 1) create robotM grab node
      auto grabPoseM = std::make_shared<aikido::constraint::dart::TSR>();
      Eigen::Matrix3d rot;
      rot <<
          1, 0, 0,
          0, -1, 0,
          0, 0, -1;
      Eigen::Matrix3d rot2;
      rot2 <<
          1., 0., 0.,
          0., 0.8678, 0.4969,
          0., -0.4969, 0.86781;

      grabPoseM->mTw_e.linear() = rot2 * rot;
      grabPoseM->mTw_e.translation() = Eigen::Vector3d(0.0, 0.14, 0.08);
      grabPoseM->mBw << -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
      auto pidM = actionNode->getAction().get_pids()[0];
      auto agentM = agents[pidM];
      auto grabNodeM = std::make_shared<PrimitiveGraspNode>(grabPoseM,
                                                            holdedObjectName,
                                                            holdedObjectName,
                                                            pidM,
                                                            holdedObjectName,
                                                            "",
                                                            true,
                                                            false);

      // 2) create move to node
      // create collaborative pose
      auto targetPoseM = std::make_shared<aikido::constraint::dart::TSR>();
      auto pidS = actionNode->getAction().get_pids()[1];
      auto agentS = agents[pidS];
      auto positionM = agentM->getPosition();
      auto positionS = agentS->getPosition();
      targetPoseM->mTw_e.translation() =
          Eigen::Vector3d(-0.2, (positionM[1] + positionS[1]) / 2, 0.85);
      targetPoseM->mBw << -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
      // TODO adding constraint
      auto moveToNodeM =
          std::make_shared<PrimitiveEngageNode>(targetPoseM,
                                                holdedObjectName,
                                                "table0",
                                                pidM,
                                                holdedObjectName,
                                                "",
                                                false,
                                                false);

      // 3) create agent M place node
      auto placePoseM = std::make_shared<aikido::constraint::dart::TSR>();
      placePoseM->mTw_e.translation() = Eigen::Vector3d(-0.1, 0.75, 0.75);
      auto epsilon = 0.02;
      placePoseM->mBw
          << -0.2, 0.2, -0.2, 0.2, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
      auto placeNodeM = std::make_shared<PrimitivePlaceNode>(placePoseM,
                                                             holdedObjectName,
                                                             "table0",
                                                             pidM,
                                                             "",
                                                             holdedObjectName,
                                                             false,
                                                             true);

      // build an new action node and plan it to get the other sub primitiveTaskGraph
      Action action{std::vector<std::string>{pidS}, actionNode->getAction().get_location(),
                    actionNode->getAction().get_ingredients(), "transfer", actionNode->getAction().get_tool()};
      auto collaborativeActionNode = new ActionNode(action, std::vector<std::string>{pidS});
      planTransfer(collaborativeActionNode, agents, containingMap, objectMgr);

      auto nodesS = collaborativeActionNode->m_primitiveTaskGraph.getNodes();
      auto grabNodeS = nodesS[0];
      auto moveToNodeS = nodesS[1];
      auto predefinedNodeS = nodesS[2];
      auto moveToNodeS2 = nodesS[3];
      auto predefinedNodeS2 = nodesS[4];
      auto placeNodeS = nodesS[5];

      // connect these nodes
      grabNodeM->addChild(moveToNodeM);
      moveToNodeM->addFather(grabNodeM);
      moveToNodeM->addChild(placeNodeM);
      placeNodeM->addFather(moveToNodeM);
      moveToNodeM->addChild(predefinedNodeS2);
      predefinedNodeS2->addFather(moveToNodeM);
      predefinedNodeS2->addChild(placeNodeM);
      placeNodeM->addFather(predefinedNodeS2);

      // build sub primitive task graph for this action node
      PrimitiveTaskGraph ptg{};
      ptg.addNode(grabNodeM);
      ptg.addNode(moveToNodeM);
      ptg.addNode(placeNodeM);
      ptg.addNode(grabNodeS);
      ptg.addNode(moveToNodeS);
      ptg.addNode(predefinedNodeS);
      ptg.addNode(moveToNodeS2);
      ptg.addNode(predefinedNodeS2);
      ptg.addNode(placeNodeS);

      actionNode->setPrimitiveTaskGraph(ptg);

      delete (collaborativeActionNode);

      return;
    } else if (holdedObjectName == actionNode->getAction().get_location()[0]) {
      // If the holder is holding the coontainer which is the source of transfer
      // 1) create robotM grab node
      auto grabPoseM = std::make_shared<aikido::constraint::dart::TSR>();
      Eigen::Matrix3d rot;
      rot <<
          1, 0, 0,
          0, -1, 0,
          0, 0, -1;
      Eigen::Matrix3d rot2;
      rot2 <<
           1., 0., 0.,
          0., 0.8678, 0.4969,
          0., -0.4969, 0.86781;
      grabPoseM->mTw_e.linear() = rot2 * rot;
      grabPoseM->mTw_e.translation() = Eigen::Vector3d(0.0, 0.125, 0.08);
      grabPoseM->mBw << -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
      auto pidM = actionNode->getAction().get_pids()[0];
      auto agentM = agents[pidM];
      auto grabNodeM = std::make_shared<PrimitiveGraspNode>(grabPoseM,
                                                            holdedObjectName,
                                                            holdedObjectName,
                                                            pidM,
                                                            holdedObjectName,
                                                            "",
                                                            true,
                                                            false);
      // 2) move grabbed object to collaborative pose
      // create move to node
      auto targetPoseM = std::make_shared<aikido::constraint::dart::TSR>();
      auto pidS = actionNode->getAction().get_pids()[1];
      auto agentS = agents[pidS];
      auto positionM = agentM->getPosition();
      auto positionS = agentS->getPosition();
      targetPoseM->mTw_e.translation() =
          Eigen::Vector3d(-0.3, (positionM[1] + positionS[1]) / 2, 0.90);
      targetPoseM->mBw << -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
      auto moveToNodeM =
          std::make_shared<PrimitiveEngageNode>(targetPoseM,
                                                holdedObjectName,
                                                "table0",
                                                pidM,
                                                holdedObjectName,
                                                "",
                                                false,
                                                false);

      // 3) after holding action finished, we place back the holded object
      auto placePoseM = std::make_shared<aikido::constraint::dart::TSR>();
      // TODO find feasible place automatically
      // now we will place it back to its original place
      auto translation = objectMgr->getObjTransform(holdedObjectName).translation();
      placePoseM->mTw_e.translation() = translation - Eigen::Vector3d(0.5, 0.0, 0.);
      auto epsilon = 0.02;
      placePoseM->mBw
          << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
      auto placeNodeM = std::make_shared<PrimitivePlaceNode>(placePoseM,
                                                             holdedObjectName,
                                                             "table0",
                                                             pidM,
                                                             "",
                                                             holdedObjectName,
                                                             false,
                                                             true);

      // build an new action node and plan it to get the other sub primitiveTaskGraph
      Action action{std::vector<std::string>{pidS}, actionNode->getAction().get_location(),
                    actionNode->getAction().get_ingredients(), "transfer", actionNode->getAction().get_tool()};
      auto collaborativeActionNode = new ActionNode(action, std::vector<std::string>{pidS});
      planTransfer(collaborativeActionNode, agents, containingMap, objectMgr);

      auto nodesS = collaborativeActionNode->m_primitiveTaskGraph.getNodes();
      auto grabNodeS = nodesS[0];
      auto moveToNodeS = nodesS[1];
      auto predefinedNodeS = nodesS[2];
      auto moveToNodeS2 = nodesS[3];
      auto predefinedNodeS2 = nodesS[4];
      auto placeNodeS = nodesS[5];

      // connect these nodes
      grabNodeM->addChild(moveToNodeM);
      moveToNodeM->addFather(grabNodeM);
      moveToNodeM->addChild(placeNodeM);
      placeNodeM->addFather(moveToNodeM);
      moveToNodeM->addChild(moveToNodeS);
      moveToNodeS->addFather(moveToNodeM);
      predefinedNodeS2->addChild(placeNodeM);
      placeNodeM->addFather(predefinedNodeS2);

      // build sub primitive task graph for this action node
      PrimitiveTaskGraph ptg{};
      ptg.addNode(grabNodeM);
      ptg.addNode(moveToNodeM);
      ptg.addNode(placeNodeM);
      ptg.addNode(grabNodeS);
      ptg.addNode(moveToNodeS);
      ptg.addNode(predefinedNodeS);
      ptg.addNode(moveToNodeS2);
      ptg.addNode(predefinedNodeS2);
      ptg.addNode(placeNodeS);

      std::cout << ptg.getTailNode(pidS)->getType() << std::endl;

      actionNode->setPrimitiveTaskGraph(ptg);

      delete (collaborativeActionNode);

      return;
    }
  } else if (collaborativeAction == "cut") {
    // to do holding cut
    // 1) create robotM grab node
    auto grabPoseM = std::make_shared<aikido::constraint::dart::TSR>();
    Eigen::Matrix3d rot;
    rot <<
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;

    Eigen::Matrix3d rot2;
    rot2 <<
         0.7073883, -0.707, 0.,
        0.707, 0.707, 0,
        -0., 0, 1;

    grabPoseM->mTw_e.linear() = rot2 * rot;
    grabPoseM->mTw_e.translation() = Eigen::Vector3d(0.0, -0.20, 0.08);
    grabPoseM->mBw << -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
    auto pidM = actionNode->getAction().get_pids()[0];
    auto agentM = agents[pidM];
    auto grabNodeM = std::make_shared<PrimitiveGraspNode>(grabPoseM,
                                                          holdedObjectName,
                                                          holdedObjectName,
                                                          pidM,
                                                          holdedObjectName,
                                                          "",
                                                          true,
                                                          false);

    // 2) create agentM place node
    auto placePoseM = std::make_shared<aikido::constraint::dart::TSR>();
    placePoseM->mTw_e.translation() = Eigen::Vector3d(-0.2, 0., 0.75);
    auto epsilon = 0.02;
    placePoseM->mBw
        << -0.2, 0.2, -0.2, 0.2, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto placeNodeM = std::make_shared<PrimitivePlaceNode>(placePoseM,
                                                           holdedObjectName,
                                                           "table0",
                                                           pidM,
                                                           "",
                                                           holdedObjectName,
                                                           false,
                                                           true);

    // 3) for the other agent, build a new action node
    auto pidS = actionNode->getAction().get_pids()[1];
    Action action{std::vector<std::string>{pidS}, actionNode->getAction().get_location(),
                  actionNode->getAction().get_ingredients(), "cut", actionNode->getAction().get_tool()};
    auto collaborativeActionNode = new ActionNode(action, std::vector<std::string>{pidS});
    planCut(collaborativeActionNode, agents, containingMap, objectMgr);
    auto nodesS = collaborativeActionNode->m_primitiveTaskGraph.getNodes();
    auto grabNodeS = nodesS[0];
    auto moveToNodeS = nodesS[1];
    auto predefinedNodeS = nodesS[2];
    auto placeNodeS = nodesS[3];

    // connect these nodes
    grabNodeM->addChild(placeNodeM);
    placeNodeM->addFather(grabNodeM);
    grabNodeM->addChild(predefinedNodeS);
    predefinedNodeS->addFather(grabNodeM);
    placeNodeM->addFather(placeNodeS);
    placeNodeS->addChild(placeNodeM);

    PrimitiveTaskGraph ptg{};
    ptg.addNode(grabNodeM);
    ptg.addNode(placeNodeM);
    ptg.addNode(grabNodeS);
    ptg.addNode(moveToNodeS);
    ptg.addNode(predefinedNodeS);
    ptg.addNode(placeNodeS);

    actionNode->setPrimitiveTaskGraph(ptg);

    delete (collaborativeActionNode);

    return;
  }
}

void ActionPlanner::planSprinkle(ActionNode *actionNode,
                                 std::map<std::string, std::shared_ptr<Agent>> &agents,
                                 std::shared_ptr<ContainingMap> &containingMap,
                                 std::shared_ptr<ObjectMgr> &objectMgr) {

}

void ActionPlanner::planSqueeze(ActionNode *actionNode,
                                std::map<std::string, std::shared_ptr<Agent>> &agents,
                                std::shared_ptr<ContainingMap> &containingMap,
                                std::shared_ptr<ObjectMgr> &objectMgr) {

}
//void ActionPlanner::planHandover(ActionNode *actionNode,
//                                 std::map<std::string,
//                                          std::shared_ptr<Agent>> &agents,
//                                 std::shared_ptr<ContainingMap> &containingMap) {
//  std::vector<std::shared_ptr<MotionNode>> motionSeqM;
//  // this actionNode will invlove two agents, robot master and robot slave
//  // robot master is the agent initiating the interaction
//  auto robotM = agents[actionNode->getAction().get_pids()[0]];
//  auto robotMPid = actionNode->getAction().get_pids()[0];
//  auto robotMHand = robotM->getHand();
//  auto robotMArm = robotM->getArm();
//  auto armMSkeleton = robotMArm->getMetaSkeleton();
//  auto armMSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armMSkeleton.get());
//  auto handMSkeleton = robotMHand->getMetaSkeleton();
//  auto handMSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handMSkeleton.get());
//
//  // first plan robot master to grasp the object
//  auto worldM = robotM->getWorld();
//  auto objectName = actionNode->getAction().get_ingredients()[0];
//  auto objectSkeleton = worldM->getSkeleton(objectName);
//  auto objectPose = getObjectPose(objectSkeleton, containingMap);
//  aikido::constraint::dart::TSR objectTSR = getDefaultBowlTSR();
//  Eigen::Matrix3d rot;
//  rot <<
//      -1, 0, 0,
//      0, 1, 0,
//      0, 0, -1;
//  Eigen::Matrix3d rot2;
//  rot2 <<
//       0.7071072, 0.7071063, 0.0000000,
//      -0.7071063, 0.7071072, 0.0000000,
//      0.0000000, 0.0000000, 1.0000000;
//  objectTSR.mTw_e.linear() = rot2 * rot;
//  objectTSR.mTw_e.translation() = Eigen::Vector3d(-0.05, -0.05, 0.08);
//  objectTSR.mBw = Eigen::Matrix<double, 6, 2>::Zero();
//  objectTSR.mBw(2, 0) = -0.01;
//  objectTSR.mBw(2, 1) = 0.01;
//  objectTSR.mBw(0, 0) = -0.01;
//  objectTSR.mBw(0, 1) = 0.01;
//  objectTSR.mBw(5, 0) = -M_PI;
//  objectTSR.mBw(5, 1) = M_PI;
//  objectTSR.mBw(4, 0) = -M_PI / 8;
//  objectTSR.mBw(4, 1) = M_PI / 8;
//  objectTSR.mBw(3, 0) = -M_PI / 8;
//  objectTSR.mBw(3, 1) = M_PI / 8;
//  objectTSR.mT0_w = objectPose * objectTSR.mT0_w;
//  auto goalTSRM1 = std::make_shared<aikido::constraint::dart::TSR>(objectTSR);
//  auto motionM1 = std::make_shared<TSRMotionNode>(goalTSRM1,
//                                                  robotMHand->getEndEffectorBodyNode(),
//                                                  nullptr,
//                                                  armMSpace,
//                                                  armMSkeleton,
//                                                  nullptr,
//                                                  false);
//  motionSeqM.emplace_back(motionM1);
//
//  // grab the bowl
//  auto conf = Eigen::Vector2d();
//  conf << 0.8, 0.8;
//  auto motionM2 = std::make_shared<ConfMotionNode>(conf, handMSpace, handMSkeleton);
//  motionSeqM.emplace_back(motionM2);
//  auto motionM3 =
//      std::make_shared<GrabMotionNode>(objectSkeleton, true, armMSpace, armMSkeleton);
//  motionSeqM.emplace_back(motionM3);
//
//  // move the bowl to the centor of two agents
//  // first get robot slave
//  auto robotS = agents[actionNode->getAction().get_pids()[1]];
//  auto robotSPid = actionNode->getAction().get_pids()[1];
//  auto robotSHand = robotS->getHand();
//  auto robotSArm = robotS->getArm();
//  auto armSSkeleton = robotSArm->getMetaSkeleton();
//  auto armSSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSSkeleton.get());
//  auto handSSkeleton = robotSHand->getMetaSkeleton();
//  auto handSSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSSkeleton.get());
//  // now get positions of two agents
//  auto robotMSkeleton = robotM->m_ada->getMetaSkeleton();
//  auto transformM = getObjectPose(robotMSkeleton, containingMap);
//  auto robotSSkeleton = robotS->m_ada->getMetaSkeleton();
//  auto transformS = getObjectPose(robotSSkeleton, containingMap);
//  auto handoverPose = Eigen::Isometry3d::Identity();
//  handoverPose.translation() = Eigen::Vector3d((transformM.translation()[0] + transformS.translation()[0]) / 2,
//                                               (transformM.translation()[1] + transformS.translation()[1]) / 2,
//                                               (transformM.translation()[2] + transformS.translation()[2]) / 2);
//  handoverPose.translation()[2] = 0.75;
//  handoverPose.translation()[0] = 0.3;
//  // move bowl to handover point
//  Eigen::Vector3d direction = Eigen::Vector3d(handoverPose.translation()[0] - objectPose.translation()[0],
//                                              handoverPose.translation()[1] - objectPose.translation()[1],
//                                              handoverPose.translation()[2] - objectPose.translation()[2]);
//  double distance = direction.norm();
//  std::cout << "Distance: " << direction.norm() << std::endl;
//  double epsilon = 0.01;
//  auto constraintTSR = std::make_shared<aikido::constraint::dart::TSR>();
//  constraintTSR->mT0_w = dart::math::computeTransform(direction / direction.norm(),
//                                                      objectPose.translation(),
//                                                      dart::math::AxisType::AXIS_Z);
//  constraintTSR->mTw_e = constraintTSR->mT0_w.inverse() * objectPose;
//  constraintTSR->mBw << -epsilon, epsilon, -epsilon, epsilon,
//      std::min(0., distance), std::max(0., distance), -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI
//      / 16;
//  auto goalTSRM2 = std::make_shared<aikido::constraint::dart::TSR>();
//  auto offset = Eigen::Isometry3d::Identity();
//  offset(2, 3) = distance;
//  goalTSRM2->mT0_w = constraintTSR->mT0_w * offset;
//  goalTSRM2->mTw_e = constraintTSR->mTw_e;
//  goalTSRM2->mBw << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon,
//      -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16;
//
//  auto motionM4 = std::make_shared<TSRMotionwithConstraintNode>(goalTSRM2,
//                                                                constraintTSR,
//                                                                objectSkeleton->getBodyNode(0),
//                                                                nullptr,
//                                                                armMSpace,
//                                                                armMSkeleton,
//                                                                nullptr,
//                                                                false);
//  motionSeqM.emplace_back(motionM4);
//
//  // Move robotS to grab bowl
//  std::vector<std::shared_ptr<MotionNode>> motionSeqS;
//  auto goalTSRS1 = std::make_shared<aikido::constraint::dart::TSR>();
//  goalTSRS1->mT0_w = goalTSRM2->mT0_w;
//  goalTSRS1->mT0_w.linear() = objectPose.linear();
//  std::cout << goalTSRS1->mT0_w.translation() << std::endl;
//  goalTSRS1->mTw_e.linear() = rot2 * rot;
//  goalTSRS1->mTw_e.translation() = Eigen::Vector3d(-0.05, -0.05, 0.08);
//  goalTSRS1->mBw = Eigen::Matrix<double, 6, 2>::Zero();
//  goalTSRS1->mBw(2, 0) = -0.01;
//  goalTSRS1->mBw(2, 1) = 0.01;
//  goalTSRS1->mBw(1, 0) = -0.01;
//  goalTSRS1->mBw(1, 1) = 0.01;
//  goalTSRS1->mBw(0, 0) = -0.01;
//  goalTSRS1->mBw(0, 1) = 0.01;
//  goalTSRS1->mBw(5, 0) = -M_PI;
//  goalTSRS1->mBw(5, 1) = M_PI;
//  goalTSRS1->mBw(4, 0) = -M_PI / 8;
//  goalTSRS1->mBw(4, 1) = M_PI / 8;
//  goalTSRS1->mBw(3, 0) = -M_PI / 8;
//  goalTSRS1->mBw(3, 1) = M_PI / 8;
//  auto condition1 = std::make_shared<TSRPreCondition>(goalTSRM2, objectSkeleton->getBodyNode(0));
//  auto motionS1 = std::make_shared<TSRMotionNode>(goalTSRS1,
//                                                  robotSHand->getEndEffectorBodyNode(),
//                                                  nullptr,
//                                                  armSSpace,
//                                                  armSSkeleton,
//                                                  condition1,
//                                                  false);
//  motionSeqS.emplace_back(motionS1);
//
//  // Close robotS hand
//  auto motionS2 = std::make_shared<ConfMotionNode>(conf, handSSpace, handSSkeleton);
//  motionSeqS.emplace_back(motionS2);
//
//  // Ungrab robotM hand
//  auto condition2 = std::make_shared<ConfPreCondition>(conf, handSSkeleton);
//  auto motionM5 = std::make_shared<GrabMotionNode>(objectSkeleton, false, armMSpace, armMSkeleton, condition2);
//  motionSeqM.emplace_back(motionM5);
//
//  conf << 0., 0.;
//  auto motionM6 = std::make_shared<ConfMotionNode>(conf, handMSpace, handMSkeleton);
//  motionSeqM.emplace_back(motionM6);
//
//  // robotS grab object
//  auto condition3 = std::make_shared<ConfPreCondition>(conf, handMSkeleton);
//  auto motionS3 = std::make_shared<GrabMotionNode>(objectSkeleton, true, armSSpace, armSSkeleton, condition3);
//  motionSeqS.emplace_back(motionS3);
//
//  // moving robotM a little bit
//  Eigen::Vector3d delta_x(0., 0., 0.001);
//  auto motionM7 =
//      std::make_shared<LinearDeltaMotionNode>(robotMHand->getEndEffectorBodyNode(),
//                                              delta_x,
//                                              dart::dynamics::Frame::World(),
//                                              50,
//                                              armMSpace,
//                                              armMSkeleton);
//  motionSeqM.emplace_back(motionM7);
//
//  actionNode->addMotionSeq(robotMPid, motionSeqM);
//  actionNode->addMotionSeq(robotSPid, motionSeqS);
//}

//void ActionPlanner::planHolding(ActionNode *actionNode,
//                                std::map<std::string,
//                                         std::shared_ptr<Agent>> &agents,
//                                std::shared_ptr<ContainingMap> &containingMap) {
//  std::vector<std::shared_ptr<MotionNode>> motionSeqM;
//  std::vector<std::shared_ptr<MotionNode>> motionSeqS;
//  // like handover, robot master is the agent initiating the intersection
//  auto robotM = agents[actionNode->getAction().get_pids()[0]];
//  auto robotMHand = robotM->getHand();
//  auto robotMArm = robotM->getArm();
//  auto robotMPid = actionNode->getAction().get_pids()[0];
//  auto armMSkeleton = robotMArm->getMetaSkeleton();
//  auto armMSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armMSkeleton.get());
//  auto handMSkeleton = robotMHand->getMetaSkeleton();
//  auto handMSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handMSkeleton.get());
//  auto worldM = robotM->getWorld();
//
//  auto robotS = agents[actionNode->getAction().get_pids()[1]];
//  auto robotSHand = robotS->getHand();
//  auto robotSArm = robotS->getArm();
//  auto robotSPid = actionNode->getAction().get_pids()[1];
//  auto armSSkeleton = robotSArm->getMetaSkeleton();
//  auto armSSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSSkeleton.get());
//  auto handSSkeleton = robotSHand->getMetaSkeleton();
//  auto handSSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSSkeleton.get());
//  auto worldS = robotS->getWorld();
//
//  auto actionName = actionNode->getAction().get_verb();
//  auto occurances = findAllOccurances(actionName, '_');
//  auto collaborativeAction = actionName.substr(occurances[1] + 1);
//  auto holdedObjectName = actionName.substr(occurances[0] + 1, occurances[1] - occurances[0] - 1);
//  if (collaborativeAction == "transfer") {
//    ROS_INFO("Collaborative transferring!");
//    ROS_INFO_STREAM(holdedObjectName << " " << collaborativeAction);
//
//    if (holdedObjectName != actionNode->getAction().get_location()[0]) {
//      // we need robot to move holded object to collaborative point
//      auto collaborativePoint = Eigen::Isometry3d::Identity();
//      collaborativePoint.translation() = Eigen::Vector3d(0.3, -0.175, 0.85);
//
//      // first move holded object to collaborative point
//      auto holdedObjectSkeleton = worldM->getSkeleton(holdedObjectName);
//      auto holdedObjectPose = getObjectPose(holdedObjectSkeleton, containingMap);
//      auto holdedObjectTSR = std::make_shared<aikido::constraint::dart::TSR>();
//      holdedObjectTSR->mT0_w = holdedObjectPose;
//      Eigen::Matrix3d rot;
//      rot <<
//          -1, 0, 0,
//          0, 1, 0,
//          0, 0, -1;
//      Eigen::Matrix3d rot2;
//      rot2 <<
//           0.7071072, 0.7071063, 0.0000000,
//          -0.7071063, 0.7071072, 0.0000000,
//          0.0000000, 0.0000000, 1.0000000;
//      holdedObjectTSR->mTw_e.linear() = rot2 * rot;
//      holdedObjectTSR->mTw_e.translation() = Eigen::Vector3d(-0.07, -0.07, 0.08);
//      holdedObjectTSR->mBw = Eigen::Matrix<double, 6, 2>::Zero();
//      holdedObjectTSR->mBw(2, 0) = -0.01;
//      holdedObjectTSR->mBw(2, 1) = 0.01;
//      holdedObjectTSR->mBw(0, 0) = -0.01;
//      holdedObjectTSR->mBw(0, 1) = 0.01;
//      holdedObjectTSR->mBw(5, 0) = -M_PI;
//      holdedObjectTSR->mBw(5, 1) = M_PI;
//      holdedObjectTSR->mBw(4, 0) = -M_PI / 8;
//      holdedObjectTSR->mBw(4, 1) = M_PI / 8;
//      holdedObjectTSR->mBw(3, 0) = -M_PI / 8;
//      holdedObjectTSR->mBw(3, 1) = M_PI / 8;
//      auto motionM1 = std::make_shared<TSRMotionNode>(holdedObjectTSR,
//                                                      robotMHand->getEndEffectorBodyNode(),
//                                                      nullptr,
//                                                      armMSpace,
//                                                      armMSkeleton,
//                                                      nullptr,
//                                                      false);
//      motionSeqM.emplace_back(motionM1);
//
//      auto conf = Eigen::Vector2d();
//      conf << 0.75, 0.75;
//      auto motionM2 = std::make_shared<ConfMotionNode>(conf, handMSpace, handMSkeleton);
//      motionSeqM.emplace_back(motionM2);
//      auto motionM3 =
//          std::make_shared<GrabMotionNode>(holdedObjectSkeleton, true, armMSpace, armMSkeleton);
//      motionSeqM.emplace_back(motionM3);
//
//      // move holded object a little bit up
//      for (int j = 0; j < 1; j++) {
//        Eigen::Vector3d delta_x(0., 0., 0.001);
//        auto motionM5 =
//            std::make_shared<LinearDeltaMotionNode>(holdedObjectSkeleton->getBodyNode(0),
//                                                    delta_x,
//                                                    dart::dynamics::Frame::World(),
//                                                    60,
//                                                    armMSpace,
//                                                    armMSkeleton);
//        motionSeqM.emplace_back(motionM5);
//      }
//
//      // now move the holded object to the collaborative point
//      auto newHoldedObjectPose = holdedObjectPose;
//      newHoldedObjectPose.translation()[2] += 0.06;
//      Eigen::Vector3d direction =
//          Eigen::Vector3d(collaborativePoint.translation()[0] - newHoldedObjectPose.translation()[0],
//                          collaborativePoint.translation()[1] - newHoldedObjectPose.translation()[1],
//                          collaborativePoint.translation()[2] - newHoldedObjectPose.translation()[2]);
//      double distance = direction.norm();
//      auto constraintTSR = std::make_shared<aikido::constraint::dart::TSR>();
//      constraintTSR->mT0_w = dart::math::computeTransform(direction / direction.norm(),
//                                                          newHoldedObjectPose.translation(),
//                                                          dart::math::AxisType::AXIS_Z);
//      constraintTSR->mTw_e = constraintTSR->mT0_w.inverse() * newHoldedObjectPose;
//      double epsilon = 0.01;
//      constraintTSR->mBw << -epsilon, epsilon, -epsilon, epsilon,
//          std::min(0., distance), std::max(0., distance), -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI
//          / 16;
//      auto goalTSRM2 = std::make_shared<aikido::constraint::dart::TSR>();
//      auto offset = Eigen::Isometry3d::Identity();
//      offset(2, 3) = distance;
//      goalTSRM2->mT0_w = constraintTSR->mT0_w * offset;
//      goalTSRM2->mTw_e = constraintTSR->mTw_e;
//      goalTSRM2->mBw << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon,
//          -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16;
//
//      auto motionM4 = std::make_shared<TSRMotionwithConstraintNode>(goalTSRM2,
//                                                                    constraintTSR,
//                                                                    holdedObjectSkeleton->getBodyNode(0),
//                                                                    nullptr,
//                                                                    armMSpace,
//                                                                    armMSkeleton,
//                                                                    nullptr,
//                                                                    false);
//      motionSeqM.emplace_back(motionM4);
//
//      // Move robotS to grab spoon
//      auto toolName = actionNode->getAction().get_tool();
//      auto toolSkeleton = worldS->getSkeleton(toolName);
//      auto toolPose = getObjectPose(toolSkeleton, containingMap);
//      auto toolTSR = std::make_shared<aikido::constraint::dart::TSR>();
//      toolTSR->mT0_w.translation() = toolPose.translation();
//      toolTSR->mTw_e.translation() = Eigen::Vector3d(-0.20, 0., 0.);
//      rot << 0.0, 0., 1., 0., 1., 0., -1., 0., 0.;
//      toolTSR->mTw_e.linear() = rot;
//      toolTSR->mBw << -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02;
//      auto motionS1 = std::make_shared<TSRMotionNode>(toolTSR,
//                                                      robotSHand->getEndEffectorBodyNode(),
//                                                      nullptr, armSSpace, armSSkeleton, nullptr, false);
//      motionSeqS.emplace_back(motionS1);
//
//      // grab spoon
//      conf << 0.75, 0.75;
//      auto motionS2 = std::make_shared<ConfMotionNode>(conf, handSSpace, handSSkeleton);
//      motionSeqS.emplace_back(motionS2);
//
//      auto motionS3 =
//          std::make_shared<GrabMotionNode>(toolSkeleton, true, armSSpace, armSSkeleton);
//      motionSeqS.emplace_back(motionS3);
//
//      for (auto foodName : actionNode->getAction().get_ingredients()) {
//        // Move spoon to be inside pot
//        auto oldLocationName = actionNode->getAction().get_location()[0];
//        auto oldLocationSkeleton = worldS->getSkeleton(oldLocationName);
//        auto oldLocationPose = getObjectPose(oldLocationSkeleton, containingMap);
//        auto oldLocationTSR = std::make_shared<aikido::constraint::dart::TSR>();
//        oldLocationTSR->mT0_w.translation() = oldLocationPose.translation();
//        rot << -1, 0, 0, 0, 1, 0, 0, 0, -1;
//        rot2 << 1, 0, 0,
//            0, 0.8660, -0.5,
//            0, 0.5, 0.866;
//        oldLocationTSR->mTw_e.linear() = rot2 * rot;
//        oldLocationTSR->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.08);
//        std::cout << (oldLocationTSR->mT0_w * oldLocationTSR->mTw_e).linear() << std::endl;
//        std::cout << (oldLocationTSR->mT0_w * oldLocationTSR->mTw_e).translation() << std::endl;
//        oldLocationTSR->mBw << -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02;
//        // First setup collision detector
//        dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
//        std::shared_ptr<dart::collision::CollisionGroup>
//            armCollisionGroup = collisionDetector->createCollisionGroup(armSSkeleton.get(), toolSkeleton.get());
//        std::shared_ptr<dart::collision::CollisionGroup>
//            envCollisionGroup = collisionDetector->createCollisionGroup(oldLocationSkeleton->getBodyNode(0));
//        std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
//            std::make_shared<aikido::constraint::dart::CollisionFree>(armSSpace, armSSkeleton, collisionDetector);
//        collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
//        auto motionS4 = std::make_shared<TSRMotionNode>(oldLocationTSR,
//                                                        toolSkeleton->getBodyNode(0),
//                                                        collisionFreeConstraint,
//                                                        armSSpace,
//                                                        armSSkeleton,
//                                                        nullptr,
//                                                        false);
//        motionSeqS.emplace_back(motionS4);
//
//        // connect food with tool
//        auto foodSkeleton = worldS->getSkeleton(foodName);
//        // unconnect food with old location
//        auto motionS5 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                         oldLocationSkeleton,
//                                                         oldLocationName,
//                                                         foodName,
//                                                         containingMap,
//                                                         false,
//                                                         armSSpace,
//                                                         armSSkeleton);
//        motionSeqS.emplace_back(motionS5);
//        // unconnect tool with hand
//        auto motionS8 = std::make_shared<GrabMotionNode>(toolSkeleton, false, armSSpace, armSSkeleton, nullptr);
//        motionSeqS.emplace_back(motionS8);
//
//        auto motionS6 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                         toolSkeleton,
//                                                         toolName,
//                                                         foodName,
//                                                         containingMap,
//                                                         true,
//                                                         armSSpace,
//                                                         armSSkeleton);
//        motionSeqS.emplace_back(motionS6);
//
//        // connect tool with hand again
//        auto motionS9 = std::make_shared<GrabMotionNode>(toolSkeleton, true, armSSpace, armSSkeleton, nullptr);
//        motionSeqS.emplace_back(motionS9);
//
//        // raise up food
//        Eigen::Vector3d delta_x(0., 0., 0.001);
//        auto motionS7 = std::make_shared<LinearDeltaMotionNode>(robotSHand->getEndEffectorBodyNode(),
//                                                                delta_x,
//                                                                dart::dynamics::Frame::World(),
//                                                                200,
//                                                                armSSpace,
//                                                                armSSkeleton);
//        motionSeqS.emplace_back(motionS7);
//
//        // sent food to collaborative point
//        // now move the holded object to the collaborative point
//        auto newToolPose = oldLocationTSR->mT0_w * oldLocationTSR->mTw_e;
//        newToolPose.translation()[2] += 0.2;
//        collaborativePoint.translation() = collaborativePoint.translation() + Eigen::Vector3d(0, 0, 0.05);
//        direction =
//            Eigen::Vector3d(collaborativePoint.translation()[0] - newToolPose.translation()[0],
//                            collaborativePoint.translation()[1] - newToolPose.translation()[1],
//                            collaborativePoint.translation()[2] + 0.05 - newToolPose.translation()[2]);
//        distance = direction.norm();
//        auto constraintTSRS1 = std::make_shared<aikido::constraint::dart::TSR>();
//        constraintTSRS1->mT0_w = dart::math::computeTransform(direction / direction.norm(),
//                                                              newToolPose.translation(),
//                                                              dart::math::AxisType::AXIS_Z);
//        constraintTSRS1->mTw_e = constraintTSRS1->mT0_w.inverse() * newToolPose;
//        epsilon = 0.01;
//        constraintTSRS1->mBw << -0.1, 0.1, -0.1, 0.1,
//            std::min(0., distance), std::max(0., distance), -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16,
//            M_PI
//                / 16;
//        auto goalTSRS2 = std::make_shared<aikido::constraint::dart::TSR>();
//        offset = Eigen::Isometry3d::Identity();
//        offset(2, 3) = distance;
//        goalTSRS2->mT0_w = constraintTSRS1->mT0_w * offset;
//        goalTSRS2->mTw_e = constraintTSRS1->mTw_e;
//        goalTSRS2->mBw << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon,
//            -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16;
//
//        auto motionS10 = std::make_shared<TSRMotionwithConstraintNode>(goalTSRS2,
//                                                                       constraintTSRS1,
//                                                                       toolSkeleton->getBodyNode(0),
//                                                                       nullptr,
//                                                                       armSSpace,
//                                                                       armSSkeleton,
//                                                                       nullptr,
//                                                                       false);
//
//        motionSeqS.emplace_back(motionS10);
//
//        auto motionM5 = std::make_shared<GrabMotionNode>(holdedObjectSkeleton, false, armMSpace, armMSkeleton);
//        motionSeqM.emplace_back(motionM5);
//
//        auto condition = std::make_shared<GrabPreCondition>(holdedObjectSkeleton, robotMHand, false);
//        auto motionS11 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                          toolSkeleton,
//                                                          toolName,
//                                                          foodName,
//                                                          containingMap,
//                                                          false,
//                                                          armSSpace,
//                                                          armSSkeleton,
//                                                          condition);
//        motionSeqS.emplace_back(motionS11);
//
//        // move food into new location
//        auto motionS12 = std::make_shared<GravityMotionNode>(foodSkeleton,
//                                                             5, armSSpace, armSSkeleton);
//        motionSeqS.emplace_back(motionS12);
//
//        // merge food with new location
//        auto motionS13 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                          holdedObjectSkeleton,
//                                                          holdedObjectName,
//                                                          foodName,
//                                                          containingMap,
//                                                          true,
//                                                          armSSpace,
//                                                          armSSkeleton);
//        motionSeqS.emplace_back(motionS13);
//
//        // grab holded object again
//        auto condition2 = std::make_shared<ConnPreCondition>(containingMap, holdedObjectName, foodName, true);
//        auto motionM6 =
//            std::make_shared<GrabMotionNode>(holdedObjectSkeleton, true, armMSpace, armMSkeleton, condition2);
//        motionSeqM.emplace_back(motionM6);
//      }
//    } else {
//
//    }
//  }
//  actionNode->addMotionSeq(robotMPid, motionSeqM);
//  actionNode->addMotionSeq(robotSPid, motionSeqS);
//}

//void ActionPlanner::planStir(ActionNode *actionNode,
//                             std::map<std::string,
//                                      std::shared_ptr<Agent>> &agents,
//                             std::shared_ptr<ContainingMap> &containingMap) {
//  std::vector<std::shared_ptr<MotionNode>> motionSeq;
//  // since this actionNode will only involve one agent
//  auto robot = agents[actionNode->getAction().get_pids()[0]];
//  auto robotHand = robot->getHand();
//  auto robotArm = robot->getArm();
//  auto robotPid = actionNode->getAction().get_pids()[0];
//  auto armSkeleton = robotArm->getMetaSkeleton();
//  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
//  auto handSkeleton = robotHand->getMetaSkeleton();
//  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
//
//  // create motion that moves hand to be above spoon
//  // first construct spoon tsr
//  auto world = robot->getWorld();
//  auto tableSkeleton = world->getSkeleton("table0");
//  auto spoonName = actionNode->getAction().get_tool();
//  // first check if robot has grabbed spoon
//  if (robotHand->isGrabbing(spoonName) == 0) {
//    ROS_INFO("Spoon has been grabbed!");
//
//  } else if (robotHand->isGrabbing(spoonName) == 1) {
//    ROS_INFO("Ada is grabbing something else, placing the grabbed object first...");
////    auto tableSkeleton = world->getSkeleton("table0");
//    auto tablePose = getObjectPose(tableSkeleton, containingMap);
//    aikido::constraint::dart::TSRPtr tableTSR = std::make_shared<aikido::constraint::dart::TSR>();
//    tableTSR->mT0_w.translation() = tablePose.translation();
//    std::cout << "placing..." << tablePose.translation() << std::endl;
//    tableTSR->mTw_e.translation() = Eigen::Vector3d(-0.2, -0.65, 0.7);
//    std::cout << (tableTSR->mT0_w * tableTSR->mTw_e).linear() << std::endl;
//    std::cout << (tableTSR->mT0_w * tableTSR->mTw_e).translation() << std::endl;
//    auto epsilon = 0.02;
//    tableTSR->mBw
//        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
//    auto motion0 = std::make_shared<TSRMotionNode>(tableTSR,
//                                                   robotHand->getGrabbedBodyNode(),
//                                                   nullptr,
//                                                   armSpace,
//                                                   armSkeleton,
//                                                   nullptr,
//                                                   false);
//    motionSeq.emplace_back(motion0);
//
//    // ungrab
//    auto conf = Eigen::Vector2d();
//    conf << 0., 0.;
//    auto motion2 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
//    motionSeq.emplace_back(motion2);
//
//    auto motion1 = std::make_shared<GrabMotionNode>(nullptr, false, armSpace, armSkeleton);
//    motionSeq.emplace_back(motion1);
//  } else if (robotHand->isGrabbing(spoonName) == 2) {
//    ROS_INFO("Ada is not grabbing anything, grabbing spoon...");
//  }
//  auto spoonSkeleton = world->getSkeleton(spoonName);
//  auto spoonPose = getObjectPose(spoonSkeleton, containingMap);
//  auto spoonTSR = std::make_shared<aikido::constraint::dart::TSR>();
//  spoonTSR->mT0_w.translation() = spoonPose.translation();
//  Eigen::Matrix3d rot;
//  rot << 1., 0., 0., 0., -1., 0., 0., 0., -1.;
//  spoonTSR->mTw_e.translation() = Eigen::Vector3d(0.0, 0., 0.20);
//  spoonTSR->mTw_e.linear() = rot;
//  double epsilon = 0.02;
//  spoonTSR->mBw
//      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
//  auto motion1 = std::make_shared<TSRMotionNode>(spoonTSR,
//                                                 robotHand->getEndEffectorBodyNode(),
//                                                 nullptr,
//                                                 armSpace,
//                                                 armSkeleton,
//                                                 nullptr,
//                                                 false);
//  motionSeq.emplace_back(motion1);
//
//  // grab spoon
//  auto conf = Eigen::Vector2d();
//  conf << 0.75, 0.75;
//  auto motion2 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
//  motionSeq.emplace_back(motion2);
//
//  auto motion3 =
//      std::make_shared<GrabMotionNode>(spoonSkeleton, true, armSpace, armSkeleton);
//  motionSeq.emplace_back(motion3);
//
//  // Move spoon to be above pot
//  auto locationName = actionNode->getAction().get_location()[0];
//  auto locationSkeleton = world->getSkeleton(locationName);
//  auto locationPose = getObjectPose(locationSkeleton, containingMap);
//  auto locationTSR = std::make_shared<aikido::constraint::dart::TSR>();
//  locationTSR->mT0_w.translation() = locationPose.translation();
//  rot << -1, 0, 0, 0, 1, 0, 0, 0, -1;
//  Eigen::Matrix3d rot2 = Eigen::Matrix3d::Identity();
//  rot2 <<
//       -1, 0, 0,
//      0, -1, 0,
//      0, 0, 1;
//  locationTSR->mTw_e.linear() = rot;
//  locationTSR->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.05);
//  locationTSR->mBw << -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -M_PI, M_PI;
//  // First setup collision detector
//  dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
//  std::shared_ptr<dart::collision::CollisionGroup>
//      armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), spoonSkeleton.get());
//  std::shared_ptr<dart::collision::CollisionGroup>
//      envCollisionGroup =
//      collisionDetector->createCollisionGroup(locationSkeleton->getBodyNode(0), tableSkeleton->getBodyNode(0));
//  std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
//      std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
//  collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
//  auto motion4 = std::make_shared<TSRMotionNode>(locationTSR,
//                                                 spoonSkeleton->getBodyNode(0),
//                                                 collisionFreeConstraint,
//                                                 armSpace,
//                                                 armSkeleton,
//                                                 nullptr,
//                                                 false);
//  motionSeq.emplace_back(motion4);
//
//  // Start stiring
//  for (int j = 0; j < 3; j++) {
//    Eigen::Vector3d delta_x(-0.001, 0., -0.);
//    auto motion5 =
//        std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
//                                                delta_x,
//                                                dart::dynamics::Frame::World(),
//                                                30,
//                                                armSpace,
//                                                armSkeleton);
//    motionSeq.emplace_back(motion5);
//
//    delta_x << +0.001, 0., 0.00;
//    auto motion6 =
//        std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
//                                                delta_x,
//                                                dart::dynamics::Frame::World(),
//                                                30,
//                                                armSpace,
//                                                armSkeleton);
//    motionSeq.emplace_back(motion6);
//  }
//
//  // Put spoon back
//  aikido::constraint::dart::TSR poseTSR = getDefaultPoseTSR();
//  poseTSR.mT0_w = spoonPose * poseTSR.mT0_w;
//  auto goalTSR3 = std::make_shared<aikido::constraint::dart::TSR>(poseTSR);
//  auto motion7 =
//      std::make_shared<TSRMotionNode>(goalTSR3,
//                                      spoonSkeleton->getBodyNode(0),
//                                      collisionFreeConstraint,
//                                      armSpace,
//                                      armSkeleton,
//                                      nullptr,
//                                      false);
//  motionSeq.emplace_back(motion7);
//
//  // release
//  // ungrab spoon
//  conf << 0., 0.;
//  auto motion8 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
//  motionSeq.emplace_back(motion8);
//
//  auto motion9 =
//      std::make_shared<GrabMotionNode>(spoonSkeleton, false, armSpace, armSkeleton);
//  motionSeq.emplace_back(motion9);
//
//  actionNode->addMotionSeq(robotPid, motionSeq);
//}

//void ActionPlanner::planCut(ActionNode *actionNode,
//                            std::map<std::string,
//                                     std::shared_ptr<Agent>> &agents,
//                            std::shared_ptr<ContainingMap> &containingMap) {
//  std::vector<std::shared_ptr<MotionNode>> motionSeq;
//
//  auto robot = agents[actionNode->getAction().get_pids()[0]];
//  auto robotHand = robot->getHand();
//  auto robotArm = robot->getArm();
//  auto robotPid = actionNode->getAction().get_pids()[0];
//  auto armSkeleton = robotArm->getMetaSkeleton();
//  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
//  auto handSkeleton = robotHand->getMetaSkeleton();
//  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
//
//  // create move hand to be above knife
//  // first construct knife tsr
//  auto world = robot->getWorld();
//  auto knifeName = actionNode->getAction().get_tool();
//  auto knifeSkeleton = world->getSkeleton(knifeName);
//  auto knifePose = getObjectPose(knifeSkeleton, containingMap);
//  aikido::constraint::dart::TSR knifeTSR = getDefaultKnifeTSR();
//  knifeTSR.mT0_w = knifePose * knifeTSR.mT0_w;
//  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(knifeTSR);
//  auto motion1 =
//      std::make_shared<TSRMotionNode>(goalTSR,
//                                      robotHand->getEndEffectorBodyNode(),
//                                      nullptr,
//                                      armSpace,
//                                      armSkeleton,
//                                      nullptr,
//                                      false);
//  motionSeq.emplace_back(motion1);
//
//  // grab knife
//  auto conf = Eigen::Vector2d();
//  conf << 0.75, 0.75;
//  auto motion2 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
//  motionSeq.emplace_back(motion2);
//  auto motion7 =
//      std::make_shared<GrabMotionNode>(knifeSkeleton, true, armSpace, armSkeleton);
//  motionSeq.emplace_back(motion7);
//
//  // Move knife to be above food
//  auto foodName = actionNode->getAction().get_ingredients()[0];
//  auto foodSkeleton = world->getSkeleton(foodName);
//  auto foodPose = getObjectPose(foodSkeleton, containingMap);
//  auto foodTSR = std::make_shared<aikido::constraint::dart::TSR>();
//  foodTSR->mT0_w = foodPose * foodTSR->mT0_w;
//  foodTSR->mTw_e.linear() = knifePose.linear();
//  double knifeHandleLength = 0.08;
//  double knifeBladeLength = 0.08;
//  double foodHeight = 0.045;
//  foodTSR->mTw_e.translation() = Eigen::Vector3d(-(knifeHandleLength + knifeBladeLength / 2), 0, 0.045);
//  auto epsilon = 0.02;
//  foodTSR->mBw
//      << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
//  auto motion3 = std::make_shared<TSRMotionNode>(foodTSR,
//                                                 knifeSkeleton->getBodyNode(0),
//                                                 nullptr,
//                                                 armSpace,
//                                                 armSkeleton,
//                                                 nullptr,
//                                                 false);
//  motionSeq.emplace_back(motion3);
//
//  // Cutting
//  for (int j = 0; j < 3; j++) {
//    Eigen::Vector3d delta_x(0., 0., -0.001);
//    auto motion4 =
//        std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
//                                                delta_x,
//                                                dart::dynamics::Frame::World(),
//                                                30,
//                                                armSpace,
//                                                armSkeleton);
//    motionSeq.emplace_back(motion4);
//
//    delta_x << 0., 0., 0.001;
//    auto motion5 =
//        std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
//                                                delta_x,
//                                                dart::dynamics::Frame::World(),
//                                                30,
//                                                armSpace,
//                                                armSkeleton);
//    motionSeq.emplace_back(motion5);
//  }
//
//  aikido::constraint::dart::TSR poseTSR = getDefaultPoseTSR();
//  poseTSR.mT0_w = knifePose * poseTSR.mT0_w;
//  auto goalTSR3 = std::make_shared<aikido::constraint::dart::TSR>(poseTSR);
//  auto motion6 = std::make_shared<TSRMotionNode>(goalTSR3, knifeSkeleton->getBodyNode(0),
//                                                 nullptr, armSpace, armSkeleton, nullptr, false);
//  motionSeq.emplace_back(motion6);
//
//  // ungrab knife
//  conf << 0., 0.;
//  auto motion8 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
//  motionSeq.emplace_back(motion8);
//
//  auto motion9 =
//      std::make_shared<GrabMotionNode>(knifeSkeleton, false, armSpace, armSkeleton);
//  motionSeq.emplace_back(motion9);
//
//  actionNode->addMotionSeq(robotPid, motionSeq);
//}

//void ActionPlanner::planTransfer(ActionNode *actionNode,
//                                 std::map<std::string,
//                                          std::shared_ptr<Agent>> &agents,
//                                 std::shared_ptr<ContainingMap> &containingMap) {
//  std::vector<std::shared_ptr<MotionNode>> motionSeq;
//
//  auto robot = agents[actionNode->getAction().get_pids()[0]];
//  auto robotHand = robot->getHand();
//  auto robotArm = robot->getArm();
//  auto robotPid = actionNode->getAction().get_pids()[0];
//  auto armSkeleton = robotArm->getMetaSkeleton();
//  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(armSkeleton.get());
//  auto handSkeleton = robotHand->getMetaSkeleton();
//  auto handSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(handSkeleton.get());
//  auto world = robot->getWorld();
//
//  auto tableSkeleton = world->getSkeleton("table0");
//  dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
//  std::shared_ptr<dart::collision::CollisionGroup>
//      armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get());
//  std::shared_ptr<dart::collision::CollisionGroup>
//      envCollisionGroup = collisionDetector->createCollisionGroup(tableSkeleton->getBodyNode(0));
//  std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
//      std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace, armSkeleton, collisionDetector);
//  collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
//
//  if (actionNode->getAction().get_tool() == "hand") {
//    auto foodName = actionNode->getAction().get_ingredients()[0];
//    auto foodSkeleton = world->getSkeleton(foodName);
//    auto foodPose = getObjectPose(foodSkeleton, containingMap);
//    auto foodTSR = std::make_shared<aikido::constraint::dart::TSR>();
//    foodTSR->mT0_w = foodPose * foodTSR->mT0_w;
//    Eigen::Matrix3d rot;
//    rot <<
//        -1, 0, 0,
//        0, 1, 0,
//        0, 0, -1;
//    foodTSR->mTw_e.linear() = rot;
//    foodTSR->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.10);
//    auto epsilon = 0.02;
//    foodTSR->mBw
//        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI, M_PI;
//    auto motion1 = std::make_shared<TSRMotionNode>(foodTSR,
//                                                   robotHand->getEndEffectorBodyNode(),
//                                                   collisionFreeConstraint,
//                                                   armSpace,
//                                                   armSkeleton,
//                                                   nullptr,
//                                                   false);
//    motionSeq.emplace_back(motion1);
//
//    // unconnect food with old container
//    auto oldLocationName = actionNode->getAction().get_location()[0];
//    auto oldLocationSkeleton = world->getSkeleton(oldLocationName);
//    auto motion8 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                    oldLocationSkeleton,
//                                                    oldLocationName,
//                                                    foodName,
//                                                    containingMap,
//                                                    false,
//                                                    armSpace,
//                                                    armSkeleton);
//    motionSeq.emplace_back(motion8);
//
//    // grab food
//    auto conf = Eigen::Vector2d();
//    conf << 0.75, 0.75;
//    auto motion2 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
//    motionSeq.emplace_back(motion2);
//    auto motion3 =
//        std::make_shared<GrabMotionNode>(foodSkeleton, true, armSpace, armSkeleton);
//    motionSeq.emplace_back(motion3);
//
//    // move food a little bit up
//    for (int j = 0; j < 1; j++) {
//      Eigen::Vector3d delta_x(0., 0., 0.001);
//      auto motion7 =
//          std::make_shared<LinearDeltaMotionNode>(robotHand->getEndEffectorBodyNode(),
//                                                  delta_x,
//                                                  dart::dynamics::Frame::World(),
//                                                  30,
//                                                  armSpace,
//                                                  armSkeleton);
//      motionSeq.emplace_back(motion7);
//    }
//
//    // move food to new position
//    auto locationName = actionNode->getAction().get_location()[1];
//    auto locationSkeleton = world->getSkeleton(locationName);
//    auto locationPose = getObjectPose(locationSkeleton, containingMap);
//    auto locationTSR = std::make_shared<aikido::constraint::dart::TSR>();
//    locationTSR->mT0_w = locationPose * locationTSR->mT0_w;
//    locationTSR->mTw_e.linear() = rot;
//    locationTSR->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.15);
//    locationTSR->mBw
//        << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI / 8, M_PI / 8, -M_PI / 8, M_PI
//        / 8, -M_PI, M_PI;
//    auto motion4 = std::make_shared<TSRMotionNode>(locationTSR,
//                                                   robotHand->getEndEffectorBodyNode(),
//                                                   collisionFreeConstraint,
//                                                   armSpace,
//                                                   armSkeleton,
//                                                   nullptr,
//                                                   false);
//    motionSeq.emplace_back(motion4);
//
//
//    // ungrab food
//    conf << 0., 0.;
//    auto motion5 = std::make_shared<ConfMotionNode>(conf, handSpace, handSkeleton);
//    motionSeq.emplace_back(motion5);
//    auto motion6 =
//        std::make_shared<GrabMotionNode>(foodSkeleton, false, armSpace, armSkeleton);
//    motionSeq.emplace_back(motion6);
//
//    // now food should be place in a good place
//    auto motion10 = std::make_shared<FakeMotionNode>(foodSkeleton,
//                                                     locationPose.translation() + Eigen::Vector3d(0., 0., 0.025),
//                                                     armSpace,
//                                                     armSkeleton);
//    motionSeq.emplace_back(motion10);
//
//    // connect food and new location
//    auto motion9 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                    locationSkeleton,
//                                                    locationName,
//                                                    foodName,
//                                                    containingMap,
//                                                    true,
//                                                    armSpace,
//                                                    armSkeleton);
//    motionSeq.emplace_back(motion9);
//  } else if (actionNode->getAction().get_tool() == actionNode->getAction().get_location()[0]) {
//    ROS_INFO_STREAM(
//        "Transferring " << actionNode->getAction().get_ingredients()[0] << " from "
//                        << actionNode->getAction().get_location()[0] << "  to "
//                        << actionNode->getAction().get_location()[1]);
//    // transfer food by doing pouring
//    auto oldLocationName = actionNode->getAction().get_location()[0];
//    auto oldLocationSkeleton = world->getSkeleton(oldLocationName);
//    auto oldLocationPose = Eigen::Isometry3d::Identity(); // need to find it out later
//    dart::dynamics::BodyNode *oldLocationBodyNodePtr = nullptr; // need to find it out later
//
//    // first check if oldlocation has been grabbed
//    if (robotHand->isGrabbing(oldLocationName) == 0) {
//      ROS_INFO("The old location object has been grabbed!");
//      oldLocationBodyNodePtr = robotHand->getGrabbedBodyNode();
//      oldLocationPose = oldLocationBodyNodePtr->getWorldTransform();
//      std::cout << oldLocationPose.translation() << std::endl;
//    } else {
//      // first try to grab old location object
//      oldLocationPose = getObjectPose(oldLocationSkeleton, containingMap);
//      oldLocationBodyNodePtr = oldLocationSkeleton->getBodyNode(0);
//    }
//
//    auto epsilon = 0.02;
//    // now move the old location object to the new location,
//    // to easily pour food from old container to new container.
//    auto newLocationName = actionNode->getAction().get_location()[1];
//    auto newLocationSkeleton = world->getSkeleton(newLocationName);
//    auto newLocationPose = getObjectPose(newLocationSkeleton, containingMap);
//    auto newLocationTSR = std::make_shared<aikido::constraint::dart::TSR>();
//    newLocationTSR->mT0_w = newLocationPose * newLocationTSR->mT0_w;
//    std::cout << newLocationPose.translation() << std::endl;
//    newLocationTSR->mTw_e.translation() = Eigen::Vector3d(-0.08, 0.08, 0.25);
//    newLocationTSR->mBw
//        << -epsilon / 2, epsilon / 2, -epsilon / 2, epsilon
//        / 2, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
//    auto constraintTSR = std::make_shared<aikido::constraint::dart::TSR>();
//    auto newGoalPose = newLocationTSR->mT0_w * newLocationTSR->mTw_e;
//    Eigen::Vector3d direction = Eigen::Vector3d(newGoalPose.translation()[0] - oldLocationPose.translation()[0],
//                                                newGoalPose.translation()[1] - oldLocationPose.translation()[1],
//                                                newGoalPose.translation()[2] - oldLocationPose.translation()[2]);
//    double distance = direction.norm();
//    constraintTSR->mT0_w = dart::math::computeTransform(direction / direction.norm(),
//                                                        oldLocationPose.translation(),
//                                                        dart::math::AxisType::AXIS_Z);
//    constraintTSR->mTw_e = constraintTSR->mT0_w.inverse() * oldLocationPose;
//    constraintTSR->mBw << -epsilon, epsilon, -epsilon, epsilon,
//        std::min(0., distance), std::max(0., distance), -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI
//        / 16;
//    auto offset = Eigen::Isometry3d::Identity();
//    offset(2, 3) = distance;
//    newLocationTSR->mT0_w = constraintTSR->mT0_w * offset;
//    newLocationTSR->mTw_e = constraintTSR->mTw_e;
//    auto motion1 = std::make_shared<TSRMotionwithConstraintNode>(newLocationTSR,
//                                                                 constraintTSR,
//                                                                 oldLocationBodyNodePtr,
//                                                                 nullptr,
//                                                                 armSpace,
//                                                                 armSkeleton,
//                                                                 nullptr,
//                                                                 false);
//    motionSeq.emplace_back(motion1);
//
//    // rotate bowl to pour food
////    auto rotateTSR = std::make_shared<aikido::constraint::dart::TSR>();
////    rotateTSR->mT0_w = newGoalPose;
////    Eigen::Matrix3d rot;
////    rot << 0.5000, 0.500000, 0.7071, 0.5000, 0.5000, -0.7071, -0.7071, 0.7071, 0.0000;
//    // quat: 0.4999997, 0.4999997, 0, 0.7071073 x, y, z, w
//    // ori: 0, 0, 0, 1
////    rotateTSR->mTw_e.linear() = rot;
////    auto motion2 = std::make_shared<TSRMotionNode>(rotateTSR,
////                                                      oldLocationBodyNodePtr,
////                                                      nullptr,
////                                                      armSpace,
////                                                      armSkeleton,
////                                                      nullptr,
////                                                      false);
//    // TODO rotate
//    auto rotatePoseTWE = Eigen::Isometry3d::Identity();
//    Eigen::Matrix3d rot;
//    rot << 0.5000, 0.500000, 0.7071, 0.5000, 0.5000, -0.7071, -0.7071, 0.7071, 0.0000;
//    rotatePoseTWE.linear() = rot;
//    auto rotatePoseT0E = newGoalPose * rotatePoseTWE;
//    std::cout << rotatePoseT0E.linear() << std::endl;
//    auto rotateEuler = rotatePoseT0E.linear().eulerAngles(0, 1, 2);
//    std::cout << rotateEuler << std::endl;
//    // compute delta_x w.r.t world farame
//    Eigen::Vector6d delta_x = Eigen::Vector6d::Zero();
//    delta_x << rotateEuler(0, 0) / 50, rotateEuler(1, 0) / 50, rotateEuler(2, 0) / 50, 0., 0., 0.;
//
////    auto incoordinatesOf = new FixedFrame(dart::dynamics::Frame::World(), newLocationPose);
//    auto motion2 = std::make_shared<IKMotionNode>(oldLocationBodyNodePtr,
//                                                  rotatePoseT0E,
//                                                  dart::dynamics::Frame::World(),
//                                                  armSpace,
//                                                  armSkeleton,
//                                                  nullptr);
//    motionSeq.emplace_back(motion2);
//
//    // now food in the old container will be moved to the new container
//    // unconnect
//    auto foodName = actionNode->getAction().get_ingredients()[0];
//    auto foodSkeleton = world->getSkeleton(foodName);
//    auto motion3 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                    oldLocationSkeleton,
//                                                    oldLocationName,
//                                                    foodName,
//                                                    containingMap,
//                                                    false,
//                                                    armSpace,
//                                                    armSkeleton);
//    motionSeq.emplace_back(motion3);
//
//    // move food into new location
//    auto motion4 = std::make_shared<GravityMotionNode>(foodSkeleton,
//                                                       20, armSpace, armSkeleton);
//    motionSeq.emplace_back(motion4);
//
//    // merge food with new location
//    auto motion5 = std::make_shared<ConnMotionNode>(foodSkeleton,
//                                                    newLocationSkeleton,
//                                                    newLocationName,
//                                                    foodName,
//                                                    containingMap,
//                                                    true,
//                                                    armSpace,
//                                                    armSkeleton);
//    motionSeq.emplace_back(motion5);
//  } else {
//
//  }
//  actionNode->addMotionSeq(robotPid, motionSeq);
//}