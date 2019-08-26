//
// Created by hejia on 8/22/19.
//

#ifndef WECOOK_ACTIONGRAPH_H
#define WECOOK_ACTIONGRAPH_H

#include "ActionNode.h"
#include "PrimitiveTaskGraph.h"

namespace wecook {

class TaskGraph {
 public:
  TaskGraph(const std::vector<Action> &actionSeq) {
    for (auto &action : actionSeq) {
      addNode(action);
    }
  }

  virtual ~TaskGraph() {
    for (auto node : m_nodes) {
      delete(node);
    }
  }

  void addNode(const Action& action);

  void addArc();

  void visualize();

  inline std::vector<ActionNode *> getNodes() {
    return m_nodes;
  }

  inline std::vector<ActionNode *> getHeads() {
    return m_heads;
  }

  inline ActionNode * getHeadNode(const std::string &pid) {
    return m_headMap[pid];
  }

  void merge(std::shared_ptr<PrimitiveTaskGraph> &primitiveTaskGraph);

 private:
  ActionNode *findFatherNode(const std::string &pid);

  std::vector<ActionNode *> m_heads;

  std::vector<ActionNode *> m_nodes;

  std::map<std::string, ActionNode *> m_headMap; // head action node for every agent
};
}

#endif //WECOOK_ACTIONGRAPH_H
