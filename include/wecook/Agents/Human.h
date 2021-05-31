//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_HUMAN_H
#define WECOOK_HUMAN_H

#include "Agent.h"
#include <libhuman/Human.h>

namespace wecook {

    class Human : public Agent {

    public:
        Human(const Eigen::Isometry3d &transform,
              const std::string &pid,
              bool isSim = true,
              std::vector<double> homePositions = std::vector<double>{0., 0., 0., 0., 0., 0., 0.})
                : Agent(pid, false) {

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
            m_human = std::make_shared<human::Human>(env, m_ifSim);
        }

        void createHumanImg(const aikido::planner::WorldPtr &env) {
            m_humanImg = std::make_shared<human::Human>(env, m_ifSim);
        }

        void moveToHome();

    };

}

#endif //WECOOK_HUMAN_H
