//
// Created by hejia on 8/26/19.
//

#include "wecook/Agents/Human.h"

using namespace wecook;

void Human::init(std::shared_ptr<aikido::planner::World> &env) {
  // TODO initialze human
}

Eigen::Vector3d Human::getPosition() {
  // TODO get position of human
  // fix the position
    return m_transform.translation();
}

void Human::end() {
  // TODO release human
}