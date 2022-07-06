#!/usr/bin/env python

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg, ContainingMsg, AgentMsg, TagMsg


def validate_pose_dimension(objects, hint):
    for i, o in enumerate(objects):
        if len(o.pose) > 7:
            raise Exception(str.format("Wrong number of values for pose at index {0} in section {1}", i, hint))


def validate_tag_pose_dimension(tags, hint):
    for i, t in enumerate(tags):
        if len(t.objectPose) > 7:
            raise Exception(str.format("Wrong number of values for pose at index {0} in section {1}", i, hint))


def talker(ifSim):
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_simpler_fruit_jelly', anonymous=True)

    scene_msg = SceneMsg([
        ObjectMsg('table0',
                  'package://wecook_assets/data/furniture/table.urdf',
                  [0.2, -0.7, -0.7, 0, 0, -0.7071068, 0.7071068]),
        ObjectMsg('plate1',
                  'package://wecook_assets/data/objects/plate.urdf',
                  [0.05, -0.2, -0.01, 0, 0, -0.7071068, 0.7071068]),
        ObjectMsg('plate0',
                  'package://wecook_assets/data/objects/plate.urdf',
                  [0.4, -0.2, -0.01, 0, 0, -0.7071068, 0.7071068]),
        ObjectMsg('bowl0',
                  'package://wecook_assets/data/objects/tbowl.urdf',
                  [0.2, -0.5, 0.03, 0, 0, -0.7071068, 0.7071068]),
        ObjectMsg('spoon1',
                  'package://wecook_assets/data/objects/spoon.urdf',
                  [-0.2, -0.5, 0.08, -0.4999688, -0.4999688, -0.5000312, 0.5000312]),
        ObjectMsg('spoon0',
                  'package://wecook_assets/data/objects/spoon.urdf',
                  [0.65, -0.5, 0.15, -0.4999688, -0.4999688, -0.5000312, 0.5000312]),
        ObjectMsg('food_item1',
                  'package://wecook_assets/data/food/food_item0.urdf',
                  [0.05, -0.4, 0.05, 0, 0, -0.7071068, 0.7071068]),
        ObjectMsg('food_item0',
                  'package://wecook_assets/data/food/food_item1.urdf',
                  [0.4, -0.2, 0.05, 0, 0, -0.7071068, 0.7071068]),
        ObjectMsg('spoonHolder0',
                  'package://wecook_assets/data/objects/holder2.urdf',
                  [0.65, -0.45, -0.02, 0, 0, 0., 1.]),
    ],
        [TagMsg(0, 'None', [0.0, -0.045, 0.13, 0.7071068, 0, 0, 0.7071068]),
         ],
        [ContainingMsg(['plate1', 'food_item1']),
         ContainingMsg(['plate0', 'food_item0'])])

    task_msg = TaskMsg(scene_msg,
                       [
                           ActionMsg(['p1', 'p2'], 'holding_plate0_transfer', ['plate0', 'bowl0'], 'spoon1',
                                     ['food_item0']),
                           ActionMsg(['p1', 'p2'], 'holding_plate1_transfer', ['plate1', 'bowl0'], 'spoon1',
                                     ['food_item1']),
                           ActionMsg(['p1'], 'stir', ['bowl0'], 'spoon0', ['food_item0', 'food_item1'])
                       ],
                       [AgentMsg('p1', 'r', [0.35, 0.0, 0.025, 0., 0., 0.0, 1.0], True),
                        AgentMsg('p2', 'h', [0., 0.0, 0.0, 0., 0., 0.0, 1.0], True)],
                       "",
                       "",
                       "follow",
                       "RRTConnect",
                       ifSim)

    validate_pose_dimension(scene_msg.objects, 'ObjectMsg array')
    validate_tag_pose_dimension(scene_msg.tags, 'TagMsg array')
    validate_pose_dimension(task_msg.agents, 'AgentMsg array')

    # sleeping 10 seconds to publish
    rospy.sleep(1)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker(ifSim=False)
    except rospy.ROSInterruptException:
        pass
