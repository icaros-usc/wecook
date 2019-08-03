//
// Created by hejia on 8/2/19.
//

#include "wecook/ActionPlanner.h"

using namespace wecook;

void ActionPlanner::plan (Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
  if (action.get_verb() == "cut") {
    planCut(action, robots);
  } else if (action.get_verb() == "transfer") {
    planTransfer(action, robots);
  }
}

void ActionPlanner::planCut(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {
  auto robot = robots[action.get_pid()[0]];

  auto world = robot->getWorld();

  auto knifeName = action.get_tool();

  auto knifeSkeleton = world->getSkeleton(knifeName);

  
}

void ActionPlanner::planTransfer(Action &action, std::map<std::string, std::shared_ptr<Robot>> &robots) {

}