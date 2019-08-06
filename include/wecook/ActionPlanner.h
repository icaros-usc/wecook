//
// Created by hejia on 7/31/19.
//

#ifndef WECOOK_ACTIONPLANNER_H
#define WECOOK_ACTIONPLANNER_H

#include "Action.h"
#include "Robot.h"

namespace wecook {

class ActionPlanner {
 public:
  ActionPlanner() = default;

  void plan(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots);

 private:
  void planStir(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots);

  void planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots);

  void planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots);
};
}

#endif //WECOOK_ACTIONPLANNER_H
