#!/usr/bin/env python3
import rospy
import subprocess
import math
from sensor_msgs.msg import BatteryState


# initialization
if __name__ == '__main__':
    # setup ros node
    rospy.init_node('slammer_actions')
    rospy.Subscriber('/unav2/status/battery', BatteryState, on_batterystate)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(2.0)
