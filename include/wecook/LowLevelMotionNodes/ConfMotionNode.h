//
// Created by hejia on 8/6/19.
//

#ifndef WECOOK_CONFMOTIONNODE_H
#define WECOOK_CONFMOTIONNODE_H

#include "MotionNode.h"

namespace wecook {

    class ConfMotionNode : public MotionNode {
    public:
        ConfMotionNode(const Eigen::VectorXd &adaPlanGoalConf,
                       const Eigen::VectorXd &adaExecGoalConf,
                       const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
                       const dart::dynamics::MetaSkeletonPtr &skeleton) : MotionNode(stateSpace,
                                                                                     skeleton),
                                                                          m_adaPlanGoalConf(adaPlanGoalConf),
                                                                          m_adaExecGoalConf(adaExecGoalConf) {
        }

        void plan(const std::shared_ptr<ada::Ada> &adaPlan, const std::shared_ptr<ada::Ada> &adaExec,
                  Result *result = nullptr);

    private:
        Eigen::VectorXd m_adaPlanGoalConf;
        Eigen::VectorXd m_adaExecGoalConf;
    };
}

#endif //WECOOK_CONFMOTIONNODE_H
