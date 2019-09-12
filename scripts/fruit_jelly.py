#!/usr/bin/env python

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_fruit_jelly', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0., 1.]),
                          ObjectMsg('plate1',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.3, -0.10, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('plate0',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.3, 0.35, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl.urdf',
                                    [0.50, 0., 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.0, 0.45, 0.80, 0., 0, 0., 1.]),
                          ObjectMsg('knife1',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.0, -0.25, 0.80, 0., 0, 0., 1.]),
                          ObjectMsg('spoon1',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.2, -0.30, 0.80, 0, 0., 0, 1.]),
                          ObjectMsg('food_item1',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.3, -0.10, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.3, 0.35, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('spoon0',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.2, 0.50, 0.80, 0, 0., 0, 1.])],
                         [ContainingMsg(['plate1', 'food_item1']),
                          ContainingMsg(['plate0', 'food_item0'])])

    # task_msg = TaskMsg(scene_msg, [ActionMsg(['p2'], 'cut', ['plate1'], 'knife1', ['food_item1']),
    #                                ActionMsg(['p1', 'p2'], 'holding_plate0_transfer', ['plate0', 'bowl0'], 'spoon1', ['food_item0']),
    #                                ActionMsg(['p2'], 'stir', ['bowl0'], 'spoon1', ['food_item0']),
    #                                ActionMsg(['p1', 'p2'], 'holding_plate1_transfer', ['plate1', 'bowl0'], 'spoon1', ['food_item1']),
    #                                ActionMsg(['p2'], 'stir', ['bowl0'], 'spoon1', ['food_item0', 'food_item1']),
    #                                ActionMsg(['p1'], 'stir', ['bowl0'], 'spoon0', ['food_item0', 'food_item1'])])

    task_msg = TaskMsg(scene_msg, [ActionMsg(['p1', 'p2'], 'holding_plate0_transfer', ['plate0', 'bowl0'], 'spoon1', ['food_item0']),
                                   ActionMsg(['p2'], 'stir', ['bowl0'], 'spoon1', ['food_item0']),
                                   ActionMsg(['p1', 'p2'], 'holding_plate1_transfer', ['plate1', 'bowl0'], 'spoon1', ['food_item1']),
                                   ActionMsg(['p2'], 'stir', ['bowl0'], 'spoon1', ['food_item0', 'food_item1']),
                                   ActionMsg(['p1'], 'stir', ['bowl0'], 'spoon0', ['food_item0', 'food_item1'])])

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
