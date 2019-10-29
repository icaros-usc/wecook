#!/usr/bin/env python

# mass_verde_3505

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_fruit_jelly', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0., 1.]),
                          ObjectMsg('pan',
                                    'package://wecook_assets/data/objects/pan.urdf',
                                    [0.5, -0.3, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('pot0',
                                    'package://wecook_assets/data/objects/pot.urdf',
                                    [0.35, -0.45, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('pot1',
                                    'package://wecook_assets/data/objects/pot_l.urdf',
                                    [0.55, -0.55, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('oil',
                                    'package://wecook_assets/data/objects/oil.urdf',
                                    [0.65, -0.65, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.15, 0.55, 0.78, 0., 0, 0., 1.]),
                          ObjectMsg('knife1',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.15, -0.75, 0.78, 0., 0, 0., 1.]),
                          ObjectMsg('spoon1',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.15, -0.55, 0.75, 0, 0., 0, 1.]),
                          ObjectMsg('chopping_board0',
                                    'package://wecook_assets/data/objects/chopping_board.urdf',
                                    [0.4, -0.0, 0.73, 0, 0, 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.3, -0.10, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item1',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.35, -0., 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item3',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.45, -0.1, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('knifeHolder0',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, 0.55, 0.73, 0., 0., 0., 1]),
                          ObjectMsg('knifeHolder1',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, -0.75, 0.73, 0., 0., 0., 1]),
                          ObjectMsg('spoonHolder1',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, -0.60, 0.73, 0., 0., 0., 1])],
                         # ObjectMsg('cooker0',
                         #           'package://wecook_assets/data/objects/cooker.urdf',
                         #           [-0.55, 0, 0.45, 0., 0., 0., 1])],
                         [ContainingMsg(['chopping_board0', 'food_item0']),
                          ContainingMsg(['chopping_board0', 'food_item1']),
                          ContainingMsg(['chopping_board0', 'food_item3'])])

    task_msg = TaskMsg(scene_msg, [ActionMsg(['p2'], 'transfer', ['chopping_board0', 'pot0'], 'spoon1', ['food_item0'])])

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass