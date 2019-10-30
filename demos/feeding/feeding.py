#!/usr/bin/env python

import rospy

from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_feeding', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0., 1.]),
                          ObjectMsg('oil0',
                                    'package://wecook_assets/data/objects/oil.urdf',
                                    [0.2, -0.95, 0.87, 0., 0., 0., 1.]),
                          ObjectMsg('plant0',
                                    'package://wecook_assets/data/objects/plant.urdf',
                                    [0.65, -0.75, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl.urdf',
                                    [0.4, 0.0, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('shelf0',
                                    'package://wecook_assets/data/furniture/shelf.urdf',
                                    [0., -1.10, 0.65, 0., 0., 0., 1.]),
                          ObjectMsg('cooker0',
                                    'package://wecook_assets/data/objects/cooker.urdf',
                                    [-0.8, 0., 0.86, 0., 0., 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.4, 0.0, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('spoon0',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.3, -0.35, 0.78, 0, -0.7073, 0, 0.7073883]),
                          ObjectMsg('spoonHolder0',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, -0.35, 0.73, 0., 0., 0., 1]),
                          ObjectMsg('mouth0',
                                    'package://wecook_assets/data/human/mouth.urdf',
                                    [-0.2, 0.0, 1.3, 0., 0., 0., 1.])],
                         [ContainingMsg(['bowl0', 'food_item0'])])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'transfer', ['bowl0', 'mouth0'], 'spoon0', ['food_item0'])],
                       [AgentMsg('p1', 'r', [-0.2, -0.3, 0.7, 0., 0., 0.7071068, 0.7071068])],
                       "",
                       "",
                       "follow",
                       "RRTConnect")

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
