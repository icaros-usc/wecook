//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_ROBOT_H
#define WECOOK_ROBOT_H

#include "Agent.h"

namespace wecook {
/*! Robot class, used for represent a robot agent. */
class Robot : public Agent {

 public:
  Robot(const Eigen::Isometry3d &transform,
        const std::string &pid,
        bool ifSim = true,
        bool ifFloat = false,
        std::vector<double> homePositions = std::vector<double>{4.8, 2.9147, 1.009, 4.1957, 1.44237, 1.3166})
      : m_transform(transform), Agent(pid, ifSim), m_ifFloat(ifFloat) {
    m_homePositions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(homePositions.data(), homePositions.size());
  }

  void init(std::shared_ptr<aikido::planner::World> &env);

  void end();

  Eigen::Vector3d getPosition();

  std::string getType() {
    return "robot";
  }

  inline aikido::planner::WorldPtr getWorld() {
    return m_ada->getWorld();
  }

  inline auto getArm() {
    return m_ada->getArm();
  }

  inline auto getHand() {
    return m_ada->getHand();
  }

  std::shared_ptr<ada::Ada> m_ada = nullptr; /*! This ada is used to do real planning job (tons of sampling), and visualize, i.e., robot's mind. */

  std::shared_ptr<ada::Ada> m_adaImg = nullptr; /*! This ada is used to represent robot itself. */

 private:
  void createAda(const aikido::planner::WorldPtr &env) {
    if (m_ifFloat) {
      m_ada = std::make_shared<ada::Ada>(env, m_ifSim, m_pid, m_transform, true);
    } else {
      m_ada = std::make_shared<ada::Ada>(env, m_ifSim, m_pid, true);
    }
  }

  void createAdaImg(const aikido::planner::WorldPtr &env) {
    if (m_ifFloat) {
      m_adaImg = std::make_shared<ada::Ada>(env, m_ifSim, m_pid + "_img", m_transform, false);
    } else {
      m_adaImg = std::make_shared<ada::Ada>(env, m_ifSim, m_pid, false);
    }
  }

  void moveToHome();

  Eigen::Isometry3d m_transform;
  Eigen::VectorXd m_homePositions;
  bool m_ifFloat;

};

}

#endif //WECOOK_ROBOT_H
