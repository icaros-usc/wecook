#!/usr/bin/env python3

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, AgentMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_ham_sandwich', anonymous=True)

    scene_msg = SceneMsg([ObjectMsg('wall0',
                                    'package://wecook_assets/data/furniture/wall.urdf',
                                    [-0.39, 1.31, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('counter0',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [-0.06, -0.5, 0., 0., 0., -0.707, 0.707]),
                          ObjectMsg('counter1',
                                    'package://wecook_assets/data/furniture/kitchen_counter.urdf',
                                    [-0.06, 0.91, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('sink0',
                                    'package://wecook_assets/data/furniture/sink_counter.urdf',
                                    [-1.28, 0.82, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('shelf0',
                                    'package://wecook_assets/data/furniture/bookcase.urdf',
                                    [-1.28, -0.55, 0., 0., 0., 0.707, 0.707]),
                          ObjectMsg('stove0',
                                    'package://wecook_assets/data/objects/stove.urdf',
                                    [0.25, -0.5, 0.96, 0., 0., 1., 0.]),
                          ObjectMsg('toaster0',
                                    'package://wecook_assets/data/objects/toaster.urdf',
                                    [0.1, 1.05, 0.96, 0., 0., 0., 1.]),        
                          ObjectMsg('skillet0',
                                    'package://wecook_assets/data/objects/skillet.urdf',
                                    [0.3,  0.75, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('cutting_board0',
                                    'package://wecook_assets/data/objects/cutting_board.urdf',
                                    [-0.3, 0.8, 0.96, 0., 0., -0.707, 0.707]),         
                          ObjectMsg('knife0',
                                    'package://wecook_assets/data/objects/knife_big.urdf',
                                    [0.1, 0.7, 0.985, 0., 0., 0.707, 0.707]),
                          ObjectMsg('brush0',
                                    'package://wecook_assets/data/objects/brush.urdf',
                                    [0., 0.83, 0.985, 0., 0., 0., 1.0]),
                          ObjectMsg('plate0',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [-0.55, -0.5, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('plate1',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [-0.3, -0.35, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('plate2',
                                    'package://wecook_assets/data/objects/plate.urdf',
                                    [-0.3, -0.65, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('bowl0',
                                    'package://wecook_assets/data/objects/bowl_green.urdf',
                                    [-0.25, 1.05, 0.96, 0., 0., 0., 1.]),
                          ObjectMsg('oil0',
                                    'package://wecook_assets/data/objects/olive_oil.urdf',
                                    [-0.075, -0.45, 0.96, 0., 0., 0.707, 0.707]),
                          ObjectMsg('salt0',
                                    'package://wecook_assets/data/objects/salt.urdf',
                                    [-0.075, -0.55, 0.96, 0., 0., 0.707, 0.707]),
                          ObjectMsg('bread0',
                                    'package://wecook_assets/data/food/bread_slice.urdf',
                                    [-0.55, -0.45, 0.975, 0., 0., 0., 1.]),
                          ObjectMsg('bread1',
                                    'package://wecook_assets/data/food/bread_slice.urdf',
                                    [-0.55, -0.55, 0.975, 0., 0., 0., 1.]),
                          ObjectMsg('cheese0',
                                    'package://wecook_assets/data/food/cheese_slice.urdf',
                                    [-0.25, -0.35, 0.975, 0., 0., 0., 1.]),
                          ObjectMsg('cheese1',
                                    'package://wecook_assets/data/food/cheese_slice.urdf',
                                    [-0.35, -0.35, 0.975, 0., 0., 0., 1.]),
                          ObjectMsg('ham0',
                                    'package://wecook_assets/data/food/ham_slice.urdf',
                                    [-0.35, -0.7, 0.975, 0., 0., 0., 1.]),
                          ObjectMsg('ham1',
                                    'package://wecook_assets/data/food/ham_slice.urdf',
                                    [-0.25, -0.7, 0.975, 0., 0., 0., 1.]),
                          ObjectMsg('ham2',
                                    'package://wecook_assets/data/food/ham_slice.urdf',
                                    [-0.3, -0.6, 0.975, 0., 0., 0., 1.]),
                          ObjectMsg('butter0',
                                    'package://wecook_assets/data/food/butter.urdf',
                                    [-0.25, 1.05, 0.975, 0., 0., 0., 1.])],
                         [ContainingMsg(['bowl0', 'butter0'])])

    task_msg = TaskMsg(scene_msg,
                       [ActionMsg(['p1'], 'stir', ['bowl0'], 'brush0', ['butter0'])],
                       [AgentMsg('p1', 'r', [-0.62, 0.71, 0.97, 0., 0., -0.707, 0.707])],
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
