//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_ROBOT_H
#define WECOOK_ROBOT_H

#include "Agent.h"

namespace wecook {

class Robot : public Agent {

 public:
  Robot(const Eigen::Isometry3d &transform,
        const std::string &pid,
        bool ifSim = true,
        std::vector<double> homePositions = std::vector<double>{4.8, 2.9147, 1.009, 4.1957, 1.44237, 1.3166})
      : m_transform(transform), Agent(pid, ifSim) {
    m_homePositions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(homePositions.data(), homePositions.size());
  }

  void init(std::shared_ptr<aikido::planner::World> &env);

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

  std::shared_ptr<ada::Ada> m_ada = nullptr;

 private:
  void createAda(const aikido::planner::WorldPtr &env) {
    m_ada = std::make_shared<ada::Ada>(env, true, m_pid, m_transform);
  }

  void moveToHome();

  Eigen::Isometry3d m_transform;
  Eigen::VectorXd m_homePositions;

};

}

#endif //WECOOK_ROBOT_H
