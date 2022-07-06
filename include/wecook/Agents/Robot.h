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
            return m_adaPlan->getWorld();
        }

        inline auto getArm() {
            return m_adaPlan->getArm();
        }

        inline auto getHand() {
            return m_adaPlan->getHand();
        }

        std::shared_ptr<ada::Ada> m_adaPlan = nullptr; /*! This ada is used to do real planning job (tons of sampling), and visualize, i.e., robot's mind. It will always be simulated.*/

        std::shared_ptr<ada::Ada> m_adaExec = nullptr; /*! This ada is used to execute the planned trajectories. */

    private:
        void createAdaPlan(const aikido::planner::WorldPtr &env) {
            // TODO remove ifFloat, always float?
            if (m_ifFloat) {
                m_adaPlan = std::make_shared<ada::Ada>(env, true, m_pid + "_plan", m_transform, true);
            } else {
                m_adaPlan = std::make_shared<ada::Ada>(env, true, m_pid + "_plan", true);
            }
        }

        void createAdaExec(const aikido::planner::WorldPtr &env) {
            if (m_ifFloat) {
                m_adaExec = std::make_shared<ada::Ada>(env, m_ifSim, m_pid + "_exec", m_transform, false);
            } else {
                m_adaExec = std::make_shared<ada::Ada>(env, m_ifSim, m_pid + "_exec", false);
            }
        }

        void moveToHome();

        Eigen::Isometry3d m_transform;
        Eigen::VectorXd m_homePositions;
        bool m_ifFloat;

    };

}

#endif //WECOOK_ROBOT_H
