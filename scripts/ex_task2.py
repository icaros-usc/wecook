#!/usr/bin/env python

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_ex_task2', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('pot0',
                                    'package://wecook_assets/data/objects/pot.urdf',
                                    [0.3, -0.45, 0.70, 0., 0., 0., 1.]),
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.0, 0.06, 0.80, 0, 0.7071063, 0, 0.7071073]),
                          ObjectMsg('chopping_board0',
                                    'package://wecook_assets/data/objects/chopping_board.urdf',
                                    [0.3, 0.35, 0.73, 0.7071063, 0, 0, 0.7071073]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/objects/food_item.urdf',
                                    [0.3, 0.35, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('food_item1',
                                    'package://wecook_assets/data/objects/food_item.urdf',
                                    [0.3, -0.45, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('spoon0',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.0, -0.60, 0.80, 0., 0.7071063, 0, 0.7071063]),
                          ObjectMsg('plate0',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.3, 0.65, 0.7, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl.urdf',
                                    [0.3, -0.05, 0.7, 0., 0., 0., 1.])])

    task_msg = TaskMsg(scene_msg, [ActionMsg(['p1'], 'stir', ['pot0'], 'spoon0', ['food_item1']),
                                   ActionMsg(['p2'], 'cut', ['chopping_board0'], 'knife0', ['food_item0'])])

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
