#!/usr/bin/env python

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_demo1', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0.7071063, 0.7071063]),
                          ObjectMsg('pot0',
                                    'package://wecook_assets/data/objects/pot.urdf',
                                    [0.3, -0.45, 0.70, 0., 0., 0., 1.]),
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.0, 0.06, 0.80, 0., 0, 0., 1.]),
                          ObjectMsg('chopping_board0',
                                    'package://wecook_assets/data/objects/chopping_board.urdf',
                                    [0.3, 0.35, 0.73, 0, 0, 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.3, 0.35, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item1',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.3, -0.45, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('spoon0',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.2, -0.75, 0.80, 0, 0., 0, 1.]),
                          ObjectMsg('plate0',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.3, 0.65, 0.7, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl.urdf',
                                    [0.3, 0., 0.7, 0., 0., 0., 1.])],
                         [ContainingMsg(['pot0', 'food_item1']),
                          ContainingMsg(['chopping_board0', 'food_item0'])])

    task_msg = TaskMsg(scene_msg, [ActionMsg(['p1'], 'stir', ['pot0'], 'spoon0', ['food_item1']),
                                   ActionMsg(['p2'], 'cut', ['chopping_board0'], 'knife0', ['food_item0']),
                                   ActionMsg(['p2'], 'transfer', ['chopping_board0', 'bowl0'], 'hand', ['food_item0']),
                                   ActionMsg(['p2', 'p1'], 'handover', ['air'], 'hand', ['bowl0']),
                                   ActionMsg(['p1'], 'transfer', ['bowl0', 'pot0'], 'bowl0', ['food_item0']),
                                   ActionMsg(['p1'], 'stir', ['pot0'], 'spoon0', ['food_item0', 'food_item1']),
                                   ActionMsg(['p2', 'p1'], 'holding_plate0_transfer', ['pot0', 'plate0'], 'spoon0', ['food_item0', 'food_item1'])])

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
