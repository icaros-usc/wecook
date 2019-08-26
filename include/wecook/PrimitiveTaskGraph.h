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
    m_tails = other.getTails();
    m_tailMap = other.getTailMap();
  }

  inline void addNode(const std::shared_ptr<PrimitiveActionNode> &node) {
    if (node->ifHead()) {
      m_heads.emplace_back(node);
      m_headMap.emplace(std::pair<std::string, std::shared_ptr<PrimitiveActionNode>>{node->getPid(), node});
    }
    if (node->ifTail()) {
      m_tails.emplace_back(node);
      m_tailMap.emplace(std::pair<std::string, std::shared_ptr<PrimitiveActionNode>>{node->getPid(), node});
    }
    m_nodes.emplace_back(node);
  }

  inline std::vector<std::shared_ptr<PrimitiveActionNode>> getHeads() const {
    return m_heads;
  }

  inline std::vector<std::shared_ptr<PrimitiveActionNode>> getTails() const {
    return m_tails;
  }

  inline std::vector<std::shared_ptr<PrimitiveActionNode>> getNodes() const {
    return m_nodes;
  }

  inline std::map<std::string, std::shared_ptr<PrimitiveActionNode>> getHeadMap() const {
    return m_headMap;
  }

  inline std::map<std::string, std::shared_ptr<PrimitiveActionNode>> getTailMap() const {
    return m_tailMap;
  }

  inline std::shared_ptr<PrimitiveActionNode> getTailNode(const std::string &pid) {
    return m_tailMap[pid];
  }

  inline std::shared_ptr<PrimitiveActionNode> getHeadNode(const std::string &pid) {
    return m_headMap[pid];
  }

  void removeTailNode(const std::string &pid);

  void removeHeadNode(const std::string &pid);

 private:
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_heads;
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_tails;
  std::vector<std::shared_ptr<PrimitiveActionNode>> m_nodes;
  std::map<std::string, std::shared_ptr<PrimitiveActionNode>> m_headMap;
  std::map<std::string, std::shared_ptr<PrimitiveActionNode>> m_tailMap;
};

}

#endif //WECOOK_PRIMITIVETASKGRAPH_H
