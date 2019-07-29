#!/usr/bin/env python

import rospy
from wecook.msg import ActionMsg, TaskMsg


def talker():
    pub = rospy.Publisher('WeCookDispatch', TaskMsg, queue_size=10)
    rospy.init_node('wecook_ex_task1', anonymous=True)
    action_msg = ActionMsg('stir', 'pot', 'spoon', ['milk'])
    task_msg = TaskMsg([action_msg])

    # sleeping 10 seconds to publish
    rospy.sleep(10)

    pub.publish(task_msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
