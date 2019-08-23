//
// Created by hejia on 7/31/19.
//

#ifndef WECOOK_ACTIONPLANNER_H
#define WECOOK_ACTIONPLANNER_H

#include "TaskGraph.h"
#include "Action.h"
#include "Agent.h"
#include "ContainingMap.h"

namespace wecook {

class ActionPlanner {
 public:
  ActionPlanner() = default;

  void compile(std::shared_ptr<TaskGraph> &taskGraph,
               std::map<std::string, std::shared_ptr<Agent>> &robots,
               std::shared_ptr<ContainingMap> &containingMap);

 private:
  void plan(ActionNode *actionNode,
            std::map<std::string, std::shared_ptr<Agent>> &robots,
            std::shared_ptr<ContainingMap> &containingMap);

  void planStir(ActionNode *actionNode,
                std::map<std::string, std::shared_ptr<Agent>> &robots,
                std::shared_ptr<ContainingMap> &containingMap);

  void planCut(ActionNode *actionNode,
               std::map<std::string, std::shared_ptr<Agent>> &robots,
               std::shared_ptr<ContainingMap> &containingMap);

  void planTransfer(ActionNode *actionNode,
                    std::map<std::string, std::shared_ptr<Agent>> &robots,
                    std::shared_ptr<ContainingMap> &containingMap);

  void planHolding(ActionNode *actionNode,
                   std::map<std::string, std::shared_ptr<Agent>> &robots,
                   std::shared_ptr<ContainingMap> &containingMap);

  void planHandover(ActionNode *actionNode,
                    std::map<std::string, std::shared_ptr<Agent>> &robots,
                    std::shared_ptr<ContainingMap> &containingMap);
};
}

#endif //WECOOK_ACTIONPLANNER_H
