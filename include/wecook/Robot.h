//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_ROBOT_H
#define WECOOK_ROBOT_H

#include "Task.h"

namespace wecook {
class Robot {
 public:
  Robot() = default;

  int execute(Task &task);
};
}
#endif //WECOOK_ROBOT_H
