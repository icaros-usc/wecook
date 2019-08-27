//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_HUMAN_H
#define WECOOK_HUMAN_H

#include "Agent.h"

namespace wecook {

class Human : public Agent {

 public:
  Human(const std::string &pid) : Agent(pid, false) {

  }

  std::string getType() {
    return "human";
  }

  void init(std::shared_ptr<aikido::planner::World> &env);

  Eigen::Vector3d getPosition();

};

}

#endif //WECOOK_HUMAN_H
