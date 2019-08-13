//
// Created by hejia on 7/31/19.
//

#ifndef WECOOK_ACTIONPLANNER_H
#define WECOOK_ACTIONPLANNER_H

#include "Action.h"
#include "Robot.h"
#include "ContainingMap.h"

namespace wecook {

class ActionPlanner {
 public:
  ActionPlanner() = default;

  void plan(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots, std::shared_ptr<ContainingMap> &containingMap);

 private:
  void planStir(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots, std::shared_ptr<ContainingMap> &containingMap);

  void planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots,  std::shared_ptr<ContainingMap> &containingMap);

  void planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots,  std::shared_ptr<ContainingMap> &containingMap);

  void planHolding(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots,  std::shared_ptr<ContainingMap> &containingMap);

  void planHandover(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots,  std::shared_ptr<ContainingMap> &containingMap);
};
}

#endif //WECOOK_ACTIONPLANNER_H
