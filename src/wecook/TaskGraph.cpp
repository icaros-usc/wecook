//
// Created by hejia on 8/22/19.
//

#include <algorithm>

#include "wecook/TaskGraph.h"

using namespace wecook;

void TaskGraph::addNode(const wecook::Action &action, bool ifEnd) {
  auto pids = action.get_pids();

  ActionNode *newFatherNode = nullptr;

  if (ifEnd) {
    std::cout << "Is end!" << std::endl;
    newFatherNode = m_nodes.back();
  }

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
    m_heads.emplace_back(newNode);
    m_nodes.emplace_back(newNode);
    // if this node is a head node then every agent invloved in this action should use this as head action node
    for (const auto &pid : pids) {
      m_headMap.emplace(std::pair<std::string, ActionNode *>{pid, newNode});
    }
    if (newFatherNode) {
      newFatherNode->addChild(newNode);
      newNode->addFather(newFatherNode);
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

    if (newFatherNode) {
      newFatherNode->addChild(newNode);
      newNode->addFather(newFatherNode);
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
  // when we merge sub primitive task graph of each action ndoe in task graph,
  // we want to remove redundant primitive node, for example we don't want to
  // grab the same object twice. We also want to add necessary primitive motion,
  // for example place grabbed object first to grab another object
  for (const auto &pair : m_headMap) {
    auto pid = pair.first;
    auto last = pair.second;
    ActionNode *curr = nullptr;
    // find child
    auto children = last->getChildren();
    for (auto &child : children) {
      auto childPids = child->getPids();
      if (std::find(childPids.begin(), childPids.end(), pid) != childPids.end()) {
        curr = child;
        break;
      }
    }
    while (curr) {
      // we want to check if there are redundant primitive node
      auto currPHN = curr->m_primitiveTaskGraph.getHeadNode(pid);
      auto lastPTN = last->m_primitiveTaskGraph.getTailNode(pid);

      if (currPHN->getType() == "grab" && lastPTN->getType() == "place"
          && currPHN->getGrabbingObj() == lastPTN->getPlacingObj()) {
        ROS_INFO_STREAM("Remove placing and grabbing nodes" << " " << pid);
        // we need to remove place node (tail node) in last action node and remove grab node in current action node
        last->m_primitiveTaskGraph.removeTailNode(pid);
        curr->m_primitiveTaskGraph.removeHeadNode(pid);
      } else if (currPHN->getType() == "grab" && !lastPTN->getGrabbingObj().empty()
          && currPHN->getGrabbingObj() == lastPTN->getGrabbingObj()) {
        // we need to remove grab node in current action node
        curr->m_primitiveTaskGraph.removeHeadNode(pid);
      } else if (currPHN->getType() == "grab" && !lastPTN->getGrabbingObj().empty()
          && currPHN->getGrabbingObj() != lastPTN->getGrabbingObj()) {
        // we need a add place node in current action node
        // TODO
      }
      // now we need to find the next action node
      last = curr;
      curr = nullptr;
      children = last->getChildren();
      for (auto &child : children) {
        auto childPids = child->getPids();
        if (std::find(childPids.begin(), childPids.end(), pid) != childPids.end()) {
          curr = child;
          break;
        }
      }
    }
  }
}