#!/usr/bin/env python

# rissois_vegetarianos_6519

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_fruit_jelly', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0., 1.]),
                          ObjectMsg('pot0',
                                    'package://wecook_assets/data/objects/pot_s.urdf',
                                    [0.55, -0.25, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('pan',
                                    'package://wecook_assets/data/objects/pan.urdf',
                                    [0.4, -0.45, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.4, -0.45, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('fork0',
                                    'package://wecook_assets/data/objects/fork.urdf',
                                    [0.3, -0.5, 0.78, 0., 0., 0., 1.])],
                         # ObjectMsg('cooker0',
                         #           'package://wecook_assets/data/objects/cooker.urdf',
                         #           [-0.55, 0, 0.45, 0., 0., 0., 1])],
                         [])

    task_msg = TaskMsg(scene_msg, [ActionMsg(['p2'], 'heat', [], 'fork0', ['food_item0'])])

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
