//
// Created by hejia on 8/22/19.
//

#include <algorithm>

#include "wecook/TaskGraph.h"

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
    auto newNode = new ActionNode(action, true);
    m_heads.emplace_back(newNode);
    m_nodes.emplace_back(newNode);
    // if this node is a head node then every agent invloved in this action should use this as head action node
    for (const auto &pid : pids) {
      m_headMap.emplace(std::pair<std::string, ActionNode *>{pid, newNode});
    }
  } else {
    auto newNode = new ActionNode(action, false);
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

ActionNode * TaskGraph::findFatherNode(const std::string &pid) {
  ActionNode *fatherNode = nullptr;
  // for each pid there will be only one father node
  for (const auto &head : m_heads) {
    auto pids = head->getAction().get_pids();
    if (std::find(pids.begin(), pids.end(), pid) != pids.end()) {
      // traverse graph to reach the tail consisting the pid
      auto start = head;
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
      break;
    }
  }
  return fatherNode;
}