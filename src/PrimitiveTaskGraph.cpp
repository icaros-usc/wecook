//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveTaskGraph.h"

using namespace wecook;

void PrimitiveTaskGraph::removeTailNode(const std::string &pid) {
  auto tailNode = m_tailMap[pid];
  // first find tail node's father nodes
  auto fatherNodes = tailNode->getFathers();
  for (auto &fatherNode : fatherNodes) {
    // if this father node has the same pid
    if (fatherNode->getPid() == pid) {
      // this father node becomes new tail node
      fatherNode->setIfTail(true);
      m_tails.emplace_back(fatherNode);
      m_tailMap[pid] = fatherNode;
    }
    fatherNode->removeChild(tailNode);
  }

  // remove tail node from m_tails and m_nodes
  for (auto itr = m_tails.begin(); itr != m_tails.end(); ++itr) {
    if (*itr == tailNode) {
      m_tails.erase(itr);
    }
  }

  for (auto itr = m_nodes.begin(); itr != m_nodes.end(); ++itr) {
    if (*itr == tailNode) {
      m_nodes.erase(itr);
    }
  }
}

void PrimitiveTaskGraph::removeHeadNode(const std::string &pid) {
  auto headNode = m_headMap[pid];
  // first find head node's child nodes
  auto childNodes = headNode->getChildren();
  for (auto &childNode : childNodes) {
    // if this child node has the same pid
    if (childNode->getPid() == pid) {
      // this child node becomes new head node
      childNode->setIfHead(true);
      m_heads.emplace_back(childNode);
      m_headMap[pid] = childNode;
    }
    childNode->removeFather(headNode);
  }

  // remove head node from m_heads and m_nodes
  for (auto itr = m_heads.begin(); itr != m_heads.end(); ++itr) {
    if (*itr == headNode) {
      m_heads.erase(itr);
    }
  }

  for (auto itr = m_nodes.begin(); itr != m_nodes.end(); ++itr) {
    if (*itr == headNode) {
      m_nodes.erase(itr);
    }
  }
}

