#!/usr/bin/env python3

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_chicken_pasta', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('wall0',
                                    'package://wecook_assets/data/furniture/wall.urdf',
                                    [0.75, 0.05, 0., 0., 0., 0., 1.]),
                          ObjectMsg('wall1',
                                    'package://wecook_assets/data/furniture/wall.urdf',
                                    [-0.85, 1.45, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('counter0',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [0.3, 0., 0., 0., 0., 0., 1.]),
                          ObjectMsg('counter1',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [0., 1.0, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('sink0',
                                    'package://wecook_assets/data/furniture/sink_counter.urdf',
                                    [-1.3, 1.05, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('shelf0',
                                    'package://wecook_assets/data/furniture/bookcase.urdf',
                                    [0.3, -1.05, 0., 0., 0., 0., 1.]),
                          ObjectMsg('pot0',
                                    'package://wecook_assets/data/objects/cooking_pot.urdf',
                                    [0.35, 1.1, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('skillet0',
                                    'package://wecook_assets/data/objects/skillet.urdf',
                                    [0.35, 0.7, 0.75, 0., 0., -0.707, .707]),
                          ObjectMsg('cutting_board0',
                                    'package://wecook_assets/data/objects/cutting_board.urdf',
                                    [0.3, -0.3, 0.75, 0., 0., 0., 1.]),         
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife_big.urdf',
                                    [0.215, -0.55, 0.775, 0., 0., 0., 1.]),
                          ObjectMsg('plate0',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.3, 0.075, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl_green.urdf',
                                    [0.45, 0.375, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('bowl1',
                                    'package://wecook_assets/data/objects/bowl_green.urdf',
                                    [0.15, 0.375, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('oil0',
                                    'package://wecook_assets/data/objects/olive_oil.urdf',
                                    [0., 1.15, 0.75, 0., 0., 0.707, 0.707]),
                          ObjectMsg('salt0',
                                    'package://wecook_assets/data/objects/salt.urdf',
                                    [0., 1.0, 0.75, 0., 0., 0.707, 0.707]),
                          ObjectMsg('pepper0',
                                    'package://wecook_assets/data/objects/black_pepper.urdf',
                                    [0., 0.9, 0.75, 0., 0., 0.707, 0.707]),
                          ObjectMsg('chicken0',
                                    'package://wecook_assets/data/food/chicken.urdf',
                                    [0.3, 0.075, 0.757, 0., 0., 0., 1.]),
                          ObjectMsg('lime0',
                                    'package://wecook_assets/data/food/lime.urdf',
                                    [0.3, -0.3, 0.757, 0., 0., 0., 1.]),
                          ObjectMsg('pasta0',
                                    'package://wecook_assets/data/food/pasta.urdf',
                                    [0.45, 0.375, 0.757, 0., 0., 0., 1.])],
                         [ContainingMsg(['plate0', 'chicken0']),
                          ContainingMsg(['bowl0', 'pasta0'])])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'cut', ['plate0'], 'knife0', ['lime0'])],
                       [AgentMsg('p1', 'r', [0., 0., 0.75, 0., 0., 0., 0.])],
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
