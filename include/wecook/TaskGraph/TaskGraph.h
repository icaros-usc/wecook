//
// Created by hejia on 8/22/19.
//

#ifndef WECOOK_ACTIONGRAPH_H
#define WECOOK_ACTIONGRAPH_H

#include "ActionNode.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveTaskGraph.h"

namespace wecook {

/*!
 * TaskGraph is a class used to represent the task plan that the robot will execute
 */
class TaskGraph {
 public:
  /*! This construct function will load in sequence of actions and construct the task graph*/
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

  /*!
   * This function does the job of adding node;
   * For each new adding node, it will do following staff:
   * 1) Find all father nodes of the new node;
   * 2) If it has no father node, the new node is the head node, so we set it as the head node
   * 3) If it has father node, we set the father nodes for this node
   * @param action
   */
  void addNode(const Action& action);

  void addArc();

  void visualize();

  inline std::vector<ActionNode *> getNodes() {
    return m_nodes;
  }

  inline ActionNode * getHeadNode(const std::string &pid) {
    return m_headMap[pid];
  }

  void merge();

 private:
  ActionNode *findFatherNode(const std::string &pid);

  std::vector<ActionNode *> m_nodes;

  std::map<std::string, ActionNode *> m_headMap; // head action node for every agent
};
}

#endif //WECOOK_ACTIONGRAPH_H
