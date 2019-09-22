//
// Created by hejia on 9/17/19.
//

#ifndef WECOOK_SIMPLEDYNAMICPATH_H
#define WECOOK_SIMPLEDYNAMICPATH_H

#include <Eigen/Eigen>

namespace wecook {
namespace planner {
namespace detail {
class SimpleDynamicPath {
 public:
  SimpleDynamicPath();
  void Init(const Eigen::VectorXd &maxVelocity,
            const Eigen::VectorXd &maxAcceleration);
  void SetMilestones(const std::vector<Eigen::VectorXd>& milestones, const std::vector<Eigen::VectorXd>& velocities);
  void SetMilestones(const std::vector<Eigen::VectorXd>& milestones);
  std::vector<Eigen::VectorXd> getWaypoints() const {
    return m_milestones;
  }
  Eigen::VectorXd getWaypoint(int i) const {
    return m_milestones[i];
  }
  void Shortcut(int i1, int i2);

 private:
  std::vector<Eigen::VectorXd> m_milestones;
  std::vector<Eigen::VectorXd> m_velocities;
  Eigen::VectorXd m_maxVelocity;
  Eigen::VectorXd m_maxAcceleration;
};
}
}
}

#endif //WECOOK_SIMPLEDYNAMICPATH_H
