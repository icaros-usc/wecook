//
// Created by hejia on 8/22/19.
//

#include <algorithm>

#include "wecook/TaskGraph/TaskGraph.h"

using namespace wecook;

void TaskGraph::addNode(const wecook::Action &action) {
  auto pids = action.get_pids();

  // we need to find father node for each pid
  auto fatherNodes = std::vector<ActionNode *>{};
  for (const auto &pid : pids) {
    auto fatherNode = findFatherNode(pid);

    if (fatherNode && std::find(fatherNodes.begin(), fatherNodes.end(), fatherNode) == fatherNodes.end()) {
      fatherNodes.emplace_back(fatherNode);
    }
  }
  if (fatherNodes.empty()) {
    auto newNode = new ActionNode(action, pids, true);
    m_nodes.emplace_back(newNode);
    // if this node is a head node then every agent invloved in this action should use this as head action node
    for (const auto &pid : pids) {
      m_headMap.emplace(std::pair<std::string, ActionNode *>{pid, newNode});
    }
  } else {
    auto newNode = new ActionNode(action, pids, false);
    for (auto &fatherNode : fatherNodes) {
      fatherNode->addChild(newNode);
      newNode->addFather(fatherNode);
    }
    m_nodes.emplace_back(newNode);
    // check if some agent doesn't have any head node
    for (const auto &pid : pids) {
      if (m_headMap.find(pid) == m_headMap.end()) {
        m_headMap.emplace(std::pair<std::string, ActionNode *>{pid, newNode});
      }
    }
  }
}

void TaskGraph::addArc() {

}

void TaskGraph::visualize() {

}

ActionNode *TaskGraph::findFatherNode(const std::string &pid) {
  ActionNode *fatherNode = nullptr;
  if ( m_headMap.find(pid) == m_headMap.end()) {
    return fatherNode;
  } else {
    auto start = m_headMap[pid];
    // traverse the graph to reach the tail node consisting the pid
    while (!start->getChildren().empty()) {
      auto children = start->getChildren();
      ActionNode *next = nullptr;
      for (const auto &child : children) {
        auto childPids = child->getAction().get_pids();
        if (std::find(childPids.begin(), childPids.end(), pid) != childPids.end()) {
          next = child;
          break;
        }
      }
      if (!next) {
        break;
      }
      start = next;
    }
    fatherNode = start;
  }
  return fatherNode;
}

void TaskGraph::merge() {
  // when we merge primitive task graph of each action ndoe in task graph,
  // we want to remove redundant primitive node, for example we don't want to
  // grab the same object twice. We also want to add necessary primitive motion,
  // for example place grabbed object first to grab another object
  for (const auto &pair : m_headMap) {
    auto pid = pair.first;
    auto lastActionNode = pair.second;
    ActionNode *currActionNode = nullptr;
    // find child
    auto children = lastActionNode->getChildren();
    for (auto &child : children) {
      auto childPids = child->getPids();
      if (std::find(childPids.begin(), childPids.end(), pid) != childPids.end()) {
        currActionNode = child;
        break;
      }
    }
    while (currActionNode) {
      // we want to check if there are redundant primitive node
      auto currPHN = currActionNode->m_primitiveTaskGraph.getHeadNode(pid);
      auto lastPTN = lastActionNode->m_primitiveTaskGraph.getTailNode(pid);

      if (currPHN->getType() == "grab" && lastPTN->getType() == "place"
          && currPHN->getGrabbingObj() == lastPTN->getPlacingObj()) {
        ROS_INFO_STREAM("Remove placing and grabbing nodes" << " " << pid);
        // we need to remove place node (tail node) in last action node and remove grab node in current action node
        lastActionNode->m_primitiveTaskGraph.removeTailNode(pid);
        currActionNode->m_primitiveTaskGraph.removeHeadNode(pid);
      } else if (currPHN->getType() == "grab" && !lastPTN->getGrabbingObj().empty()
          && currPHN->getGrabbingObj() == lastPTN->getGrabbingObj()) {
        // we need to remove grab node in current action node
        currActionNode->m_primitiveTaskGraph.removeHeadNode(pid);
      } else if (currPHN->getType() == "grab" && !lastPTN->getGrabbingObj().empty()
          && currPHN->getGrabbingObj() != lastPTN->getGrabbingObj()) {
        // we need a add place node in current action node
        // TODO
      }
      // now we need to find the next action node
      lastActionNode = currActionNode;
      currActionNode = nullptr;
      children = lastActionNode->getChildren();
      for (auto &child : children) {
        auto childPids = child->getPids();
        if (std::find(childPids.begin(), childPids.end(), pid) != childPids.end()) {
          currActionNode = child;
          break;
        }
      }
    }
  }
}