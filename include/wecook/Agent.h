//
// Created by hejia on 7/30/19.
//

#ifndef WECOOK_AGENT_H
#define WECOOK_AGENT_H

#include <libada/Ada.hpp>
#include <Eigen/Dense>

namespace wecook {

//! Agent class
/*!
 * Agent class, super class of Robot and Human.
 */
class Agent {

 public:
  Agent(const std::string &pid,
        bool ifSim)
      : m_pid(pid),
        m_ifSim(ifSim) {
  }

  inline bool ifSim() {
    return m_ifSim;
  }

  virtual std::string getType() = 0;

  virtual void init(std::shared_ptr<aikido::planner::World> &env) = 0;

  virtual Eigen::Vector3d getPosition() = 0;

  inline std::string getPid() {
    return m_pid;
  }

 protected:
  std::string m_pid;

  bool m_isFree = true;
  bool m_isEnd = false;
  bool m_ifSim;
};
}
#endif //WECOOK_AGENT_H
