#!/usr/bin/env python3

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_ham_sandwich', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('wall0',
                                    'package://wecook_assets/data/furniture/wall.urdf',
                                    [1.3, 0.15, 0., 0., 0., 0., 1.]),
                          ObjectMsg('wall1',
                                    'package://wecook_assets/data/furniture/wall.urdf',
                                    [-0.3, 1.55, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('counter0',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [0.55, -0.3, 0., 0., 0., -0.707, 0.707]),
                          ObjectMsg('counter1',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [0.55, 1.1, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('sink0',
                                    'package://wecook_assets/data/furniture/sink_counter.urdf',
                                    [-0.75, 1.125, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('shelf0',
                                    'package://wecook_assets/data/furniture/bookcase.urdf',
                                    [-0.75, -0.45, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('stove0',
                                    'package://wecook_assets/data/objects/stove.urdf',
                                    [0.9, -0.3, 0.75, 0., 0., 1., 0.]),
                          ObjectMsg('toaster0',
                                    'package://wecook_assets/data/objects/toaster.urdf',
                                    [0.7, 1.25, 0.775, 0., 0., 0., 1.]),        
                          ObjectMsg('skillet0',
                                    'package://wecook_assets/data/objects/skillet.urdf',
                                    [0.9, 0.9, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('cutting_board0',
                                    'package://wecook_assets/data/objects/cutting_board.urdf',
                                    [0.3, 0.95, 0.75, 0., 0., -0.707, 0.707]),         
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife_big.urdf',
                                    [0.6, 0.85, 0.775, 0., 0., 0.707, 0.707]),
                          ObjectMsg('brush0',
                                    'package://wecook_assets/data/objects/brush.urdf',
                                    [0.525, 0.95, 0.75, 0.707, 0., 0., 0.707]),
                          ObjectMsg('plate0',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.05, -0.3, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('plate1',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.3, -0.15, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('plate2',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [0.3, -0.45, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl_green.urdf',
                                    [0.3, 1.25, 0.75, 0., 0., 0., 1.]),
                          ObjectMsg('oil0',
                                    'package://wecook_assets/data/objects/olive_oil.urdf',
                                    [0., 1.15, 0.75, 0., 0., 0.707, 0.707]),
                          ObjectMsg('salt0',
                                    'package://wecook_assets/data/objects/salt.urdf',
                                    [0., 1.0, 0.75, 0., 0., 0.707, 0.707]),
                          ObjectMsg('bread0',
                                    'package://wecook_assets/data/food/bread_slice.urdf',
                                    [0.05, -0.25, 0.765, 0., 0., 0., 1.]),
                          ObjectMsg('bread1',
                                    'package://wecook_assets/data/food/bread_slice.urdf',
                                    [0.05, -0.35, 0.765, 0., 0., 0., 1.]),
                          ObjectMsg('cheese0',
                                    'package://wecook_assets/data/food/cheese_slice.urdf',
                                    [0.25, -0.15, 0.765, 0., 0., 0., 1.]),
                          ObjectMsg('cheese1',
                                    'package://wecook_assets/data/food/cheese_slice.urdf',
                                    [0.35, -0.15, 0.765, 0., 0., 0., 1.]),
                          ObjectMsg('ham0',
                                    'package://wecook_assets/data/food/ham_slice.urdf',
                                    [0.35, -0.45, 0.765, 0., 0., 0., 1.]),
                          ObjectMsg('ham1',
                                    'package://wecook_assets/data/food/ham_slice.urdf',
                                    [0.25, -0.45, 0.765, 0., 0., 0., 1.]),
                          ObjectMsg('butter0',
                                    'package://wecook_assets/data/food/butter.urdf',
                                    [0.3, 1.25, 0.765, 0., 0., 0., 1.])],
                         [ContainingMsg(['plate0', 'bread0']),
                          ContainingMsg(['plate1', 'cheese0'])])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'stir', ['bowl0'], 'brush0', ['butter0'])],
                       [AgentMsg('p1', 'r', [0., 0.85, 0.75, 0., 0., 0., 0.])],
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
