//
// Created by hejia on 7/31/19.
//

#ifndef WECOOK_ACTIONPLANNER_H
#define WECOOK_ACTIONPLANNER_H

#include "PrimitiveTaskGraph.h"
#include "TaskGraph.h"
#include "Action.h"
#include "Agent.h"
#include "ContainingMap.h"
#include "ObjectMgr.h"

namespace wecook {

class ActionPlanner {
 public:
  ActionPlanner() = default;

  void compile(std::shared_ptr<TaskGraph> &taskGraph,
               std::map<std::string, std::shared_ptr<Agent>> &agents,
               std::shared_ptr<ContainingMap> &containingMap,
               std::shared_ptr<ObjectMgr> &objectMgr);

 private:
  void plan(ActionNode *actionNode,
            std::map<std::string, std::shared_ptr<Agent>> &agents,
            std::shared_ptr<ContainingMap> &containingMap,
            std::shared_ptr<ObjectMgr> &objectMgr);

  void planStir(ActionNode *actionNode,
                std::map<std::string, std::shared_ptr<Agent>> &agents,
                std::shared_ptr<ContainingMap> &containingMap,
                std::shared_ptr<ObjectMgr> &objectMgr);

  void planCut(ActionNode *actionNode,
               std::map<std::string, std::shared_ptr<Agent>> &agents,
               std::shared_ptr<ContainingMap> &containingMap,
               std::shared_ptr<ObjectMgr> &objectMgr);

  void planTransfer(ActionNode *actionNode,
                    std::map<std::string, std::shared_ptr<Agent>> &agents,
                    std::shared_ptr<ContainingMap> &containingMap,
                    std::shared_ptr<ObjectMgr> &objectMgr);

  void planHolding(ActionNode *actionNode,
                   std::map<std::string, std::shared_ptr<Agent>> &agents,
                   std::shared_ptr<ContainingMap> &containingMap,
                   std::shared_ptr<ObjectMgr> &objectMgr);

  void planHandover(ActionNode *actionNode,
                    std::map<std::string, std::shared_ptr<Agent>> &agents,
                    std::shared_ptr<ContainingMap> &containingMap,
                    std::shared_ptr<ObjectMgr> &objectMgr);

  void planRoll(ActionNode *actionNode,
                std::map<std::string, std::shared_ptr<Agent>> &agents,
                std::shared_ptr<ContainingMap> &containingMap,
                std::shared_ptr<ObjectMgr> &objectMgr);

  void planHeat(ActionNode *actionNode,
                std::map<std::string, std::shared_ptr<Agent>> &agents,
                std::shared_ptr<ContainingMap> &containingMap,
                std::shared_ptr<ObjectMgr> &objectMgr);
};
}

#endif //WECOOK_ACTIONPLANNER_H
