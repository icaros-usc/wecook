#!/usr/bin/env python

import rospy
from wecook.msg import ActionMsg, TaskMsg, SceneMsg, ObjectMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_ex_task1', anonymous=True)
    # action_msg = ActionMsg('p1', 'stir', 'pot', 'spoon', ['milk'])
    # scene_msg = SceneMsg([ObjectMsg('')])
    scene_msg = SceneMsg([ObjectMsg('bowl0', 'package://wecook_assets/data/objects/bowl.urdf', [0.5, 0.0, 0.73, 0., 0., 0., 1.])])
    task_msg = TaskMsg(scene_msg, [ActionMsg(['p1'], 'transfer', ['bowl0', 'bowl1'], ingredients=['milk']),
                                   ActionMsg(['p2'], 'cut', ['cupboard'], 'knife', ['tomato'])])

    # sleeping 10 seconds to publish
    rospy.sleep(10)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
