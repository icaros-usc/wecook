//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_HUMAN_H
#define WECOOK_HUMAN_H

#include "Agent.h"
#include <Human.hpp>

namespace wecook {

    class Human : public Agent {

    public:
        Human(const Eigen::Isometry3d &transform,
              const std::string &pid,
              bool ifSim = true,
              std::vector<double> homePositions = std::vector<double>{0., 0., 0., 0., 0., 0., 0.})
                : m_transform(transform), Agent(pid, ifSim) {
          m_homePositions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(homePositions.data(), homePositions.size());
        }

        std::string getType() {
            return "human";
        }

        void init(std::shared_ptr<aikido::planner::World> &env);

        void end();

        Eigen::Vector3d getPosition();

        std::shared_ptr<human::Human> m_human = nullptr;

        std::shared_ptr<human::Human> m_humanImg = nullptr;

    private:
        void createHuman(const aikido::planner::WorldPtr &env) {
            m_human = std::make_shared<human::Human>(env, m_ifSim, m_pid, m_transform, true, "icaros");
        }

        void createHumanImg(const aikido::planner::WorldPtr &env) {
            m_humanImg = std::make_shared<human::Human>(env, m_ifSim, m_pid + "_img", m_transform, false, "icaros");
        }

        void moveToHome();

        Eigen::Isometry3d m_transform;
        Eigen::VectorXd m_homePositions;
    };

}

#endif //WECOOK_HUMAN_H
