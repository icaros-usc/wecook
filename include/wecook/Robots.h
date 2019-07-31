//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_ROBOTS_H
#define WECOOK_ROBOTS_H

#include <vector>
#include <map>
#include <boost/thread.hpp>

#include "Task.h"
#include "Robot.h"

namespace wecook {
class Robots {
 public:
  Robots(const std::map<std::string, std::shared_ptr<Robot>> &robots) : m_thread(&Robots::run, this), m_robots(robots) {
  }

  int execute(Task &task);
  inline bool isFree() {
    return m_isFree;
  }

  void stop();

 private:
  void run();

  boost::thread m_thread;
  std::map<std::string, std::shared_ptr<Robot>> m_robots;
  bool m_isFree = true;
  bool m_isEnd = false;
  std::vector<Task> m_task;
};
}
#endif //WECOOK_ROBOTS_H
