//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_HUMAN_H
#define WECOOK_HUMAN_H

#include "Agent.h"

namespace wecook {

class Human : public Agent {

 public:
  Human(const Eigen::Isometry3d &transform,
          const std::string &pid) : Agent(pid, false), m_transform(transform) {

  }

  std::string getType() {
    return "human";
  }

  void init(std::shared_ptr<aikido::planner::World> &env);

  void end();

  Eigen::Vector3d getPosition();

  Eigen::Isometry3d m_transform;

};

}

#endif //WECOOK_HUMAN_H
