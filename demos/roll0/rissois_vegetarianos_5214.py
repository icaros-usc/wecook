#!/usr/bin/env python

# rissois_vegetarianos_5214

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, TagMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_fruit_jelly', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.3, 0.4, 0.72, 0., 0., 0., 1.]),
                          ObjectMsg('roller0',
                                    'package://wecook_assets/data/objects/roller.urdf',
                                    [0.3, 0.5, 0.83, 0., 0., 0., 1.])],
                         [TagMsg(1, 'None', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
                         [])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'roll', [], 'roller0', ['food_item0'])],
                       [AgentMsg('p1', 'r', [-0.2, 0.15, 0.7, 0., 0., 0.7071068, 0.7071068], True)],
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
