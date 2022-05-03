#!/usr/bin/env python

# mass_verde_3505

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, TagMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_fruit_jelly', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0., 1.]),
                          ObjectMsg('pot0',
                                    'package://wecook_assets/data/objects/pot.urdf',
                                    [0.35, -0.45, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('spoon1',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.15, -0.55, 0.75, 0, 0., 0, 1.]),
                          ObjectMsg('chopping_board0',
                                    'package://wecook_assets/data/objects/chopping_board.urdf',
                                    [0.4, -0.0, 0.73, 0, 0, 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.3, -0.10, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item3',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.45, -0.1, 0.75, 0., 0., 0., 1.]), ],
                         [TagMsg(0, 'None', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
                         [ContainingMsg(['chopping_board0', 'food_item0']),
                          ContainingMsg(['chopping_board0', 'food_item3'])])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'transfer', ['chopping_board0', 'pot0'], 'spoon1', ['food_item0'])],
                       [AgentMsg('p1', 'r', [0, -0.1, 0.6, 0., 0., 0.7071068, 0.7071068], True), ],
                       "",
                       "",
                       "follow",
                       "RRTConnect",
                       True)

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
