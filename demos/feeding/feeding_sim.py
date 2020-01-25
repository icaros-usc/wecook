#!/usr/bin/env python

import rospy

from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, AgentMsg, TagMsg


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
                                    [0.4, -0.1, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('shelf0',
                                    'package://wecook_assets/data/furniture/shelf.urdf',
                                    [0., -1.10, 0.65, 0., 0., 0., 1.]),
                          ObjectMsg('cooker0',
                                    'package://wecook_assets/data/objects/cooker.urdf',
                                    [-0.8, 0., 0.86, 0., 0., 0., 1.]),
                          ObjectMsg('food_item0',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.4, -0.1, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item1',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.4, -0.1, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item2',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.4, -0.1, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item3',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.4, -0.1, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('food_item4',
                                    'package://wecook_assets/data/food/food_item1.urdf',
                                    [0.4, -0.1, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('spoon0',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.3, -0.5, 0.78, 0, -0.7073, 0, 0.7073883]),
                          ObjectMsg('spoonHolder0',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, -0.5, 0.73, 0., 0., 0., 1]),
                          ObjectMsg('mouth0',
                                    'package://wecook_assets/data/human/mouth.urdf',
                                    [0., 0.0, 1.1, 0., 0., 0., 1.])],
                         [TagMsg('baseTag','None', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
                         [ContainingMsg(['bowl0', 'food_item0']),
                          ContainingMsg(['bowl0', 'food_item1']),
                          ContainingMsg(['bowl0', 'food_item2']),
                          ContainingMsg(['bowl0', 'food_item3']),
                          ContainingMsg(['bowl0', 'food_item4'])])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'feeding', ['bowl0', 'mouth0'], 'spoon0', ['food_item0']),
                        ActionMsg(['p1'], 'feeding', ['bowl0', 'mouth0'], 'spoon0', ['food_item1']),
                        ActionMsg(['p1'], 'feeding', ['bowl0', 'mouth0'], 'spoon0', ['food_item2']),
                        ActionMsg(['p1'], 'feeding', ['bowl0', 'mouth0'], 'spoon0', ['food_item3']),
                        ActionMsg(['p1'], 'feeding', ['bowl0', 'mouth0'], 'spoon0', ['food_item4'])],
                       [AgentMsg('p1', 'r', [-0., -0.3, 0.75, 0., 0., 0.7071068, 0.7071068])],
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
