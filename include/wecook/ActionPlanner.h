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

  void plan(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
    if (action.get_verb() == "cut") {
      planCut(action, robots);
    } else if (action.get_verb() == "transfer") {
      planTransfer(action, robots);
    }
  }

 private:
  void planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
    auto robot = robots[action.get_pid()[0]];

    auto world = robot->getWorld();

    auto knifeName = action.get_tool();

    auto knifeSkeleton = world->getSkeleton(knifeName);
  }

  void planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

  }
};
}

#endif //WECOOK_ACTIONPLANNER_H
