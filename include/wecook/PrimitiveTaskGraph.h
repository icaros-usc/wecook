//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVETASKGRAPH_H
#define WECOOK_PRIMITIVETASKGRAPH_H

#include "PrimitiveActionNode.h"

namespace wecook {

class PrimitiveTaskGraph {
 public:
  PrimitiveTaskGraph() {

  }

  PrimitiveTaskGraph(const PrimitiveTaskGraph &other) {
    m_heads = other.getHeads();
    m_nodes = other.getNodes();
    m_headMap = other.getHeadMap();
  }

  inline void addNode(const std::shared_ptr<PrimitiveActionNode> &node) {
    if (node->ifHead()) {
      m_heads.emplace_back(node);
      m_headMap.emplace(std::pair<std::string, std::shared_ptr<PrimitiveActionNode>>{node->getPid(), node});
    }
    m_nodes.emplace_back(node);
  }

  inline std::vector<std::shared_ptr<PrimitiveActionNode>> getHeads() const {
    return m_heads;
  }

  inline std::vector<std::shared_ptr<PrimitiveActionNode>> getNodes() const {
    return m_nodes;
  }

  inline std::map<std::string, std::shared_ptr<PrimitiveActionNode>> getHeadMap() const {
    return m_headMap;
  }

  inline std::shared_ptr<PrimitiveActionNode> getHeadNode(const std::string &pid) {
    return m_headMap[pid];
  }

 private:
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_heads;
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_nodes;
  std::map<std::string, std::shared_ptr<PrimitiveActionNode>> m_headMap;
};

}

#endif //WECOOK_PRIMITIVETASKGRAPH_H
