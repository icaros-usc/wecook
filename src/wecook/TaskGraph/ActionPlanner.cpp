//
// Created by hejia on 8/2/19.
//
#include <aikido/common/PseudoInverse.hpp>

#include "wecook/tsr/knife.h"
#include "wecook/TaskGraph/ActionPlanner.h"
#include "wecook/strutils.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveGraspNode.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveEngageNode.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveActuateNode.h"
#include "wecook/PrimitiveTaskGraph/PrimitivePlaceNode.h"
#include "wecook/Agents/Robot.h"

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
    } else if (actionNode->getAction().get_verb() == "feeding") {
        planFeeding(actionNode, agents, containingMap, objectMgr);
    } else if (actionNode->getAction().get_verb() == "wrap") {
        planWrap(actionNode, agents, containingMap, objectMgr);
    } else if (actionNode->getAction().get_verb() == "dip") {
        planDip(actionNode, agents, containingMap, objectMgr);
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
            std::make_shared<PrimitiveActuateNode>(pid,
                                                   "end-effector",
                                                   "roll",
                                                   toolName,
                                                   "",
                                                   MetaActuateInfo(Action(actionNode->getAction())),
                                                   false,
                                                   false);

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
            std::make_shared<PrimitiveActuateNode>(pid,
                                                   "end-effector",
                                                   "heat",
                                                   toolName,
                                                   "",
                                                   MetaActuateInfo(Action(actionNode->getAction())),
                                                   false,
                                                   false);

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
    // since every object will be placed back to the original position, we should save it.
    auto table0_pose = objectMgr->getObjTransform("table0");
    std::cout << table0_pose.matrix() << std::endl;

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
    auto epsilon = 0.005;
    grabPose->mBw
            << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    // when we create grab pose we can also create a place pose since we will place the object to the same position
    auto toolName = actionNode->getAction().get_tool();
    auto tool_pose = objectMgr->getObjTransform(toolName);
    auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
    placePose->mTw_e = tool_pose * table0_pose.inverse();
    std::cout << placePose->mTw_e.matrix() << std::endl;
    Eigen::Matrix4d place_correction_rot;
//    place_correction_rot <<
//                         Eigen::Vector3d(-0.32, -0.55, 0.78) * placePose->mTw_e.translation().inverse();
    Eigen::Matrix4d c1;
    c1 <<     1,     0,     0, -0.55,
    0,     1,     0, 0.32,
    0,     0,     1,  0.78,
    0,     0,     0,     1;

    Eigen::Matrix4d c2;
    c2 <<     1,     0,     0, -0.32,
            0,     1,     0, -0.55,
            0,     0,     1,  0.78,
            0,     0,     0,     1;

    place_correction_rot << c2 * c1.inverse();
    std::cout << place_correction_rot << std::endl;
//    placePose->mTw_e.translation() = place_correction_rot * placePose->mTw_e.translation();
    placePose->mTw_e.translation() = Eigen::Vector3d(-0.32, -0.55, 0.78);
    std::cout << placePose->mTw_e.matrix() << std::endl;

    auto pid = actionNode->getAction().get_pids()[0];
    auto agent = agents[pid];

    auto grabNode =
            std::make_shared<PrimitiveGraspNode>(grabPose, toolName, toolName, pid, toolName, "", true, false);

    // 2) create move to node
    // create start pose
    auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
    targetPose->mTw_e.translation() = Eigen::Vector3d(-0.08, 0., 0.06);
    targetPose->mBw
            << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto toCutName = actionNode->getAction().get_ingredients()[0];
    std::cout << objectMgr->getObjTransform(toCutName).matrix() << std::endl;
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
            std::make_shared<PrimitiveActuateNode>(pid,
                                                   "end-effector",
                                                   "cut",
                                                   toolName,
                                                   "",
                                                   MetaActuateInfo(Action(actionNode->getAction())),
                                                   false,
                                                   false);

    // 4) create place back node
//    auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
    // get the original place of the tool
//    auto translation = objectMgr->getObjTransform(toolName).translation();
    epsilon = 0.005;
//    placePose->mTw_e.translation() = translation - Eigen::Vector3d(0.5, 0.0, 0.);
//    placePose->mTw_e.translation() = translation - Eigen::Vector3d(-0.2, -0.45, 0.0);
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
    grabPose->mTw_e.translation() = Eigen::Vector3d(0.0, 0.15, 0.03);
    Eigen::Matrix3d rot;
    rot <<
        1., 0., 0.,
            0., -1., 0.,
            0., 0., -1;
    Eigen::Matrix3d rot2;
    rot2 <<
         1, 0., 0.,
            0., 0, 1.,
            -0., -1., 0;
    Eigen::Matrix3d rot4;
    rot4 <<
         0, 1., 0.,
            -1., 0., 0.,
            0., 0., 1;
    Eigen::Matrix3d rot5;
    rot5 <<
         1, 0., 0.,
            0., 0, 1.,
            -0., -1., 0;
    Eigen::Matrix3d rot6;
    rot6 <<
         0, -1., 0.,
            1., 0, 0.,
            0., 0., 1.;
    grabPose->mTw_e.linear() = rot6 * rot5 * rot4 * rot2 * rot;
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
    Eigen::Matrix3d rot0;
    rot0 <<
         1., 0., 0.,
            0., 0, -1.,
            0., 1., 0.;
    targetPose->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.15);
    targetPose->mTw_e.linear() = rot0;
    targetPose->mBw
            << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI, M_PI;
    auto toStirName = actionNode->getAction().get_locations()[0];
    auto moveToNode =
            std::make_shared<PrimitiveEngageNode>(targetPose,
                                                  toolName,
                                                  toStirName,
                                                  pid,
                                                  toolName,
                                                  "",
                                                  false,
                                                  false);

    auto targetPose2 = std::make_shared<aikido::constraint::dart::TSR>();
    Eigen::Matrix3d rot1;
    rot1 <<
         1., 0., 0.,
            0., 0, -1.,
            0., 1., 0.;
    targetPose2->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.08);
    targetPose2->mTw_e.linear() = rot1;
    targetPose2->mBw
            << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI, M_PI;
    auto moveToNode2 =
            std::make_shared<PrimitiveEngageNode>(targetPose2,
                                                  toolName,
                                                  toStirName,
                                                  pid,
                                                  toolName,
                                                  "",
                                                  false,
                                                  false);

    // 3) create predefined stirring node
    auto predefinedNode =
            std::make_shared<PrimitiveActuateNode>(pid,
                                                   "end-effector",
                                                   "stir",
                                                   toolName,
                                                   "",
                                                   MetaActuateInfo(Action(actionNode->getAction())),
                                                   false,
                                                   false);

    // 4) create place back node
    auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
    auto translation = objectMgr->getObjTransform(toolName).translation();
    placePose->mTw_e.translation() = translation - Eigen::Vector3d(-0.2, -0.45, 0.0);
    placePose->mTw_e.linear() = objectMgr->getObjTransform(toolName).linear();
    placePose->mBw
            << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto
            placeNode =
            std::make_shared<PrimitivePlaceNode>(placePose,
                                                 toolName,
                                                 "spoonHolder1",
                                                 pid,
                                                 "",
                                                 toolName,
                                                 false,
                                                 true);

    // connect these nodes
    grabNode->addChild(moveToNode);
    moveToNode->addChild(moveToNode2);
    moveToNode2->addFather(moveToNode);
    moveToNode->addFather(grabNode);
    moveToNode2->addChild(predefinedNode);
    predefinedNode->addFather(moveToNode2);
    predefinedNode->addChild(placeNode);
    placeNode->addFather(predefinedNode);

    // build sub primitive task graph for this action node
    PrimitiveTaskGraph ptg{};
    ptg.addNode(grabNode);
    ptg.addNode(moveToNode);
    ptg.addNode(moveToNode2);
    ptg.addNode(predefinedNode);
    ptg.addNode(placeNode);

    actionNode->setPrimitiveTaskGraph(ptg);
}

void ActionPlanner::planTransfer(ActionNode *actionNode,
                                 std::map<std::string,
                                         std::shared_ptr<Agent>> &agents,
                                 std::shared_ptr<ContainingMap> &containingMap,
                                 std::shared_ptr<ObjectMgr> &objectMgr) {
//    auto agentS = agents["p2"];
//    if (agentS->getType() == "h") {
//        return;
//    }
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
                std::make_shared<PrimitiveGraspNode>(grabPose, objectName, objectName, pid, objectName, "", true,
                                                     false);

        // 2) create place node
        auto placePose = std::make_shared<aikido::constraint::dart::TSR>();
        auto newLocationName = actionNode->getAction().get_locations()[1];
        placePose->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.05);
        placePose->mBw
                << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI / 8, M_PI / 8, -M_PI / 8, M_PI
                                                                                                            /
                                                                                                            8, -M_PI, M_PI;
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
    } else if (actionNode->getAction().get_tool() == actionNode->getAction().get_locations()[0]) {
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
        auto objectName = actionNode->getAction().get_locations()[0];
        auto pid = actionNode->getAction().get_pids()[0];
        auto grabNode =
                std::make_shared<PrimitiveGraspNode>(grabPose, objectName, objectName, pid, objectName, "", true,
                                                     false);

        // 2) create move to node
        auto targetPose = std::make_shared<aikido::constraint::dart::TSR>();
        targetPose->mTw_e.translation() = Eigen::Vector3d(-0.0, 0.10, 0.2);
        auto epsilon = 0.02;
        targetPose->mBw << -epsilon / 2, epsilon / 2, -epsilon / 2, epsilon
                                                                    /
                                                                    2, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
        auto newLocationName = actionNode->getAction().get_locations()[1];
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
                std::make_shared<PrimitiveActuateNode>(pid,
                                                       objectName,
                                                       "transfer1",
                                                       objectName,
                                                       "",
                                                       MetaActuateInfo(Action(actionNode->getAction())),
                                                       false,
                                                       false);

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
        auto oldLocationName = actionNode->getAction().get_locations()[0];
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
                std::make_shared<PrimitiveActuateNode>(pid,
                                                       toolName,
                                                       "transfer2",
                                                       toolName,
                                                       "",
                                                       MetaActuateInfo(Action(actionNode->getAction())),
                                                       false,
                                                       false);

        // 4) move food to new location
        auto targetPose2 = std::make_shared<aikido::constraint::dart::TSR>();
        Eigen::Matrix3d rot2;
        rot2 <<
             1., 0., 0.,
                0., 0.7071055, -0.7071081,
                0., 0.7071081, 0.7071055;
        targetPose2->mTw_e.linear() = rot2;
        targetPose2->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.06);
        targetPose2->mBw << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI / 8, M_PI / 8, -M_PI / 8,
                M_PI
                / 8, -M_PI / 8, M_PI / 8;
        auto newLocationName = actionNode->getAction().get_locations()[1];
        auto moveToNode2 =
                std::make_shared<PrimitiveEngageNode>(targetPose2,
                                                      toolName,
                                                      newLocationName,
                                                      pid,
                                                      toolName,
                                                      "",
                                                      false,
                                                      false);
        moveToNode2->setTimeStep(0.02);


        // 5) create predefined pouring node
        auto predefinedNode2 =
                std::make_shared<PrimitiveActuateNode>(pid,
                                                       toolName,
                                                       "transfer3",
                                                       toolName,
                                                       "",
                                                       MetaActuateInfo(Action(actionNode->getAction())),
                                                       false,
                                                       false);

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

void ActionPlanner::planFeeding(ActionNode *actionNode,
                                std::map<std::string, std::shared_ptr<Agent>> &agents,
                                std::shared_ptr<ContainingMap> &containingMap,
                                std::shared_ptr<ObjectMgr> &objectMgr) {
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
    targetPose->mTw_e.translation() = Eigen::Vector3d(0., 0., 0.1);
    targetPose->mBw
            << -0.03, 0.03, -0.03, 0.03, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
    auto oldLocationName = actionNode->getAction().get_locations()[0];
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
            std::make_shared<PrimitiveActuateNode>(pid,
                                                   toolName,
                                                   "feeding1",
                                                   toolName,
                                                   "",
                                                   MetaActuateInfo(Action(actionNode->getAction())),
                                                   false,
                                                   false);

    // 4) move food to new location
    auto targetPose2 = std::make_shared<aikido::constraint::dart::TSR>();
    Eigen::Matrix3d rot2;
    rot2 <<
         1., 0., 0.,
            0., 0.7071055, -0.7071081,
            0., 0.7071081, 0.7071055;
    Eigen::Matrix3d rot3;
    rot3 << 0., -1., 0., 1, 0., 0., 0., 0., 1;
    Eigen::Matrix3d rot4;
    rot4 << 0, 0., 1., 0., 1., 0., -1., 0, 0.;
    targetPose2->mTw_e.linear() = rot4 * rot3;
    targetPose2->mTw_e.translation() = Eigen::Vector3d(0, 0, 0.06);
    targetPose2->mBw << -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon, -M_PI / 16, M_PI / 16, -M_PI / 16, M_PI
                                                                                                                    /
                                                                                                                    16,
            -M_PI / 16, M_PI / 16;
    auto newLocationName = actionNode->getAction().get_locations()[1];
    auto moveToNode2 =
            std::make_shared<PrimitiveEngageNode>(targetPose2,
                                                  toolName,
                                                  newLocationName,
                                                  pid,
                                                  toolName,
                                                  "",
                                                  false,
                                                  false);
    moveToNode2->setTimeStep(0.02);


    // 5) create predefined pouring node
    auto predefinedNode2 =
            std::make_shared<PrimitiveActuateNode>(pid,
                                                   toolName,
                                                   "feeding2",
                                                   toolName,
                                                   "",
                                                   MetaActuateInfo(Action(actionNode->getAction())),
                                                   false,
                                                   false);

    // 6) create place back node
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
    auto holdedObjectName = actionName.substr(occurances[0] + 1, occurances.back() - occurances[0] - 1);
    if (collaborativeAction == "transfer") {
        if (holdedObjectName == actionNode->getAction().get_locations()[1]) {
            // To do collaborative transferring, we have 9 steps
            // 1) create robotM grab node
            auto grabPoseM = std::make_shared<aikido::constraint::dart::TSR>();
            Eigen::Matrix3d rot;
            rot <<
                -1, 0, 0,
                    0, 1, 0,
                    0, 0, -1;
            Eigen::Matrix3d rot2;
            rot2 <<
                 0.8678, 0., 0.4969,
                    0., 1, 0.,
                    -0.4969, 0., 0.8678;

            grabPoseM->mTw_e.linear() = rot2 * rot;
            grabPoseM->mTw_e.translation() = Eigen::Vector3d(0.14, 0.0, 0.08);
//      grabPoseM->mTw_e.translation() = Eigen::Vector3d(0.0, 0.14, 0.08);
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
            Action action{std::vector<std::string>{pidS}, actionNode->getAction().get_locations(),
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
        } else if (holdedObjectName == actionNode->getAction().get_locations()[0]) {
            // If the holder is holding the coontainer which is the source of transfer
            // 1) create robotM grab node
            auto grabPoseM = std::make_shared<aikido::constraint::dart::TSR>();
            Eigen::Matrix3d rot2;
            rot2 <<
                 1, 0., 0.,
                    0., 0, 1.,
                    -0., -1., 0;
            Eigen::Matrix3d rot3;
            rot3 <<
                 0, 0., 1.,
                    0., 1., 0.,
                    -1., 0., 0;
            Eigen::Matrix3d rot4;
            rot4 <<
                 0, 1., 0.,
                    -1., 0., 0.,
                    0., 0., 1;

            Eigen::Matrix3d rot5;
            rot5 <<
                 0, 0., 1.,
                    0., 1., 0.,
                    -1., 0., 0;

            Eigen::Matrix3d rot6;
            rot6 <<
                 0, -1., 0.,
                    1., 0., 0.,
                    0., 0., 1;

            grabPoseM->mTw_e.linear() = rot6 * rot5 * rot4 * rot3 * rot2;
            grabPoseM->mTw_e.translation() = Eigen::Vector3d(-0.08, 0.0, 0.07);
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
                    Eigen::Vector3d(-0.15, 0.15, 1.15);
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
//            placePoseM->mTw_e.translation() = translation - Eigen::Vector3d(-0.5,0.0,0.);
            placePoseM->mTw_e.translation() = translation - Eigen::Vector3d(0.5075, -0.3, -1.015);
            auto epsilon = 0.005;
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
            Action action{std::vector<std::string>{pidS}, actionNode->getAction().get_locations(),
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
//      grabNodeM->addChild(grabNodeS);
//      grabNodeS->addFather(grabNodeM);
            moveToNodeM->addChild(grabNodeS);
            grabNodeS->addFather(moveToNodeM);
            // TODO inherit;
            moveToNodeM->addChild(moveToNodeS);
            moveToNodeS->addFather(moveToNodeM);
            moveToNodeS2->addChild(placeNodeM);
            placeNodeM->addFather(moveToNodeS2);

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
        Action action{std::vector<std::string>{pidS}, actionNode->getAction().get_locations(),
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

void ActionPlanner::planWrap(ActionNode *actionNode,
                             std::map<std::string, std::shared_ptr<Agent>> &agents,
                             std::shared_ptr<ContainingMap> &containingMap,
                             std::shared_ptr<ObjectMgr> &objectMgr) {

}

void ActionPlanner::planDip(ActionNode *actionNode,
                            std::map<std::string, std::shared_ptr<Agent>> &agents,
                            std::shared_ptr<ContainingMap> &containingMap,
                            std::shared_ptr<ObjectMgr> &objectMgr) {

}
