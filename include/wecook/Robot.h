//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_ROBOT_H
#define WECOOK_ROBOT_H

#include <boost/thread.hpp>

#include "Action.h"

namespace wecook {
class Robot {
 public:
  Robot() : m_thread(&Robot::run, this) {

  }

  int execute(Action &action);

  void stop();

  inline bool isFree() {
    return m_isFree;
  }

  inline bool isEnd() {
    return m_isEnd;
  }

 private:
  void run();

  boost::thread m_thread;
  bool m_isFree = true;
  bool m_isEnd = false;
  std::vector<Action> m_action;
};
}
#endif //WECOOK_ROBOT_H
