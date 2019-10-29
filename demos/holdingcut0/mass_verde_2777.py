#!/usr/bin/env python

# mass_verde_2777

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_mass_verde_2777', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('table0',
                                    'package://wecook_assets/data/furniture/table.urdf',
                                    [0.5, 0.0, 0.0, 0., 0., 0., 1.]),
                          ObjectMsg('oil0',
                                    'package://wecook_assets/data/objects/oil.urdf',
                                    [0.2, -0.95, 0.87, 0., 0., 0., 1.]),
                          ObjectMsg('shelf0',
                                    'package://wecook_assets/data/furniture/shelf.urdf',
                                    [0., -1.10, 0.65, 0., 0., 0., 1.]),
                          ObjectMsg('plant0',
                                    'package://wecook_assets/data/objects/plant.urdf',
                                    [0.65, -0.75, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl.urdf',
                                    [0.5, -0.45, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('bowl1',
                                    'package://wecook_assets/data/objects/bowl.urdf',
                                    [0.6, -0.6, 0.73, 0., 0., 0., 1.]),
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.18, 0.65, 0.78, 0., 0, 0., 1.]),
                          ObjectMsg('knifeHolder0',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, 0.65, 0.73, 0., 0., 0., 1]),
                          ObjectMsg('cooker0',
                                    'package://wecook_assets/data/objects/cooker.urdf',
                                    [-0.8, 0., 0.86, 0., 0., 0., 1.]),
                          ObjectMsg('knife1',
                                    'package://wecook_assets/data/objects/knife.urdf',
                                    [0.18, -0.55, 0.78, 0., 0, 0., 1.]),
                          ObjectMsg('knifeHolder1',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, -0.55, 0.73, 0., 0., 0., 1]),
                          ObjectMsg('spoon1',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.3, -0.4, 0.78, 0, -0.7073, 0, 0.7073883]),
                          ObjectMsg('spoonHolder1',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, -0.4, 0.73, 0., 0., 0., 1]),
                          ObjectMsg('chopping_board0',
                                    'package://wecook_assets/data/objects/chopping_board.urdf',
                                    [0.3, -0., 0.73, 0, 0, 0., 1.]),
                          ObjectMsg('food_item3',
                                    'package://wecook_assets/data/food/food_item0.urdf',
                                    [0.3, -0., 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('spoon0',
                                    'package://wecook_assets/data/objects/spoon.urdf',
                                    [0.3, 0.45, 0.78, 0, -0.7073, 0, 0.7073883]),
                          ObjectMsg('spoonHolder0',
                                    'package://wecook_assets/data/objects/holder.urdf',
                                    [0.25, 0.45, 0.73, 0., 0., 0., 1])],
                         # ObjectMsg('cooker0',
                         #           'package://wecook_assets/data/objects/cooker.urdf',
                         #           [-0.55, 0, 0.45, 0., 0., 0., 1])],
                         [ContainingMsg(['chopping_board0', 'food_item3'])])

    task_msg = TaskMsg(scene_msg, [ActionMsg(['p2', 'p1'], 'holding_chopping_board0_cut',
                                             ['chopping_board0'], 'knife0', ['food_item3'])])

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
