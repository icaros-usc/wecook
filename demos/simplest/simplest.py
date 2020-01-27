#!/usr/bin/env python

# simplest

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, TagMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_simplest', anonymous=True)

    scene_msg = SceneMsg([
        # ObjectMsg('table0',
        #                             'package://wecook_assets/data/furniture/table.urdf',
        #                             [-0.09, -1.0, -0.94, 0., 0., 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.40, -0.25,  0.0, 0., 0., 0., 1.]),
                          ObjectMsg('roller0',
                                    'package://wecook_assets/data/objects/box.urdf',
                                    [0.31, -0.30,  0.0, 0., 0., 0., 1.])],
                         [TagMsg(1,'None', [0, -0.045, 0.08, 0.7071068, 0, 0, 0.7071068]),
                          TagMsg(7,'roller0', [0, -0.04, 0.075, 0.7071068, 0, 0, 0.7071068])],
                         [])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'roll', [], 'roller0', ['food_item0'])],
                       [AgentMsg('p1', 'r', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])],
                       "",
                       "",
                       "follow",
                       "RRTConnect",
                       False)


    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
