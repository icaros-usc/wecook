#!/usr/bin/env python

import rospy
import simpler_fruit_jelly_common

if __name__ == '__main__':
    try:
        simpler_fruit_jelly_common.talker(ifSim=True)
    except rospy.ROSInterruptException:
        pass
