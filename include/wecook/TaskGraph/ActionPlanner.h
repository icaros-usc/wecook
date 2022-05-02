//
// Created by hejia on 7/31/19.
//

#ifndef WECOOK_ACTIONPLANNER_H
#define WECOOK_ACTIONPLANNER_H

#include "wecook/PrimitiveTaskGraph/PrimitiveTaskGraph.h"
#include "wecook/TaskGraph/TaskGraph.h"
#include "wecook/Action.h"
#include "wecook/Agents/Agent.h"
#include "wecook/ContainingMap.h"
#include "wecook/ObjectMgr.h"

namespace wecook {

    class ActionPlanner {
    public:
        ActionPlanner() = default;

        /*!
         * This function compile the high level task, e.g., cut tomate with knife to the mid-level motion, e.g.,
         * grab knife -> engage the knife -> do the cut motion -> place the knife back
         * @param taskGraph
         * @param agents
         * @param containingMap
         * @param objectMgr
         */
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

        void planFeeding(ActionNode *actionNode,
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

        void planSprinkle(ActionNode *actionNode,
                          std::map<std::string, std::shared_ptr<Agent>> &agents,
                          std::shared_ptr<ContainingMap> &containingMap,
                          std::shared_ptr<ObjectMgr> &objectMgr);

        void planSqueeze(ActionNode *actionNode,
                         std::map<std::string, std::shared_ptr<Agent>> &agents,
                         std::shared_ptr<ContainingMap> &containingMap,
                         std::shared_ptr<ObjectMgr> &objectMgr);

        void planWrap(ActionNode *actionNode,
                      std::map<std::string, std::shared_ptr<Agent>> &agents,
                      std::shared_ptr<ContainingMap> &containingMap,
                      std::shared_ptr<ObjectMgr> &objectMgr);

        void planDip(ActionNode *actionNode,
                     std::map<std::string, std::shared_ptr<Agent>> &agents,
                     std::shared_ptr<ContainingMap> &containingMap,
                     std::shared_ptr<ObjectMgr> &objectMgr);
    };
}

#endif //WECOOK_ACTIONPLANNER_H
