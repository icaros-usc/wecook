#!/usr/bin/env python3

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_chicken_pasta', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('wall0',
                                    'package://wecook_assets/data/furniture/wall.urdf',
                                    [1.21, -0.09, 0., 0., 0., 0., 1.]),
                          ObjectMsg('wall1',
                                    'package://wecook_assets/data/furniture/wall.urdf',
                                    [-0.39, 1.31, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('counter0',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [0.25, 0., 0., 0., 0., 0., 1.]),
                          ObjectMsg('counter1',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [-0.06, 0.91, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('sink0',
                                    'package://wecook_assets/data/furniture/sink_counter.urdf',
                                    [-1.28, 0.82, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('shelf0',
                                    'package://wecook_assets/data/furniture/bookcase.urdf',
                                    [0.25, -0.91, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('stove0',
                                    'package://wecook_assets/data/objects/stove.urdf',
                                    [-0.35, 0.85, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('pot0',
                                    'package://wecook_assets/data/objects/cooking_pot.urdf',
                                    [0.25, 0.91, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('skillet0',
                                    'package://wecook_assets/data/objects/skillet.urdf',
                                    [0.25, 0.61, 0.96, 0., 0., -0.707, .707]),
                          ObjectMsg('cutting_board0',
                                    'package://wecook_assets/data/objects/cutting_board.urdf',
                                    [0.21, -0.3, 0.96, 0., 0., 1., 0.]),         
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife_big.urdf',
                                    [0.07, -0.55, 0.985, 0., 0., 0., 1.]),
                          ObjectMsg('plate0',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [-0.826, 0.8, 0.86, 0., 0., 0., 1.]),
                          ObjectMsg('plate1',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.25, -0.91, 1.18, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl_green.urdf',
                                    [0.36, 0.25, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('bowl1',
                                    'package://wecook_assets/data/objects/bowl_green.urdf',
                                    [0.12, 0.25, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('oil0',
                                    'package://wecook_assets/data/objects/olive_oil.urdf',
                                    [0., 1.01, 0.96, 0., 0., 0.707, 0.707]),
                          ObjectMsg('salt0',
                                    'package://wecook_assets/data/objects/salt.urdf',
                                    [0., 0.91, 0.96, 0., 0., 0.707, 0.707]),
                          ObjectMsg('pepper0',
                                    'package://wecook_assets/data/objects/black_pepper.urdf',
                                    [0., 0.81, 0.96, 0., 0., 0.707, 0.707]),
                          ObjectMsg('chicken0',
                                    'package://wecook_assets/data/food/chicken.urdf',
                                    [0.12, 0.25, 0.967, 0., 0., 0., 1.]),
                          ObjectMsg('lime0',
                                    'package://wecook_assets/data/food/lime.urdf',
                                    [0.2, -0.32, 0.967, 0., 0., 0., 1.]),
                          ObjectMsg('pasta0',
                                    'package://wecook_assets/data/food/pasta.urdf',
                                    [0.36, 0.25, 0.967, 0., 0., 0., 1.])],
                         [ContainingMsg(['bowl0', 'pasta0']),
                          ContainingMsg(['bowl1', 'chicken0'])])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'cut', ['cutting_board0'], 'knife0', ['lime0'])],
                       [AgentMsg('p1', 'r', [0., 0., 0.97, 0., 0., -0.707, 0.707])],
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
