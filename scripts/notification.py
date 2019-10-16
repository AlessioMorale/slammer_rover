#!/usr/bin/env python3
import rospy
import subprocess
import math
from sensor_msgs.msg import BatteryState
from ros_signalling.msg import blink




battery_voltage = 0
battery_current = 0
battery_charge = 0
battery_percentage = 0


front = 3
lamp = 0
rear = 1
front_count = 8
rear_count = 20



def on_batterystate(msg):
    global battery_voltage
    global battery_current
    global battery_charge
    global battery_percentage
    battery_voltage = msg.voltage
    battery_current = msg.current
    battery_charge = msg.charge
    battery_percentage = msg.percentage


# initialization
if __name__ == '__main__':
    # setup ros node
    rospy.init_node('slammer_notifications')
    rospy.Subscriber('/battery_state', BatteryState, on_batterystate)
    pub = rospy.Publisher('/signalling/leds', blink, queue_size=40)
    led = blink()
    led.led = 0
    led.msecOn = 0
    led.msecOff = 0
    led.single = False

    init = True
    # start running
    while not rospy.core.is_shutdown():
        if init:
                #front white
            led.group = front
            led.R = 64
            led.G = 64
            led.B = 0
            for i in range(front_count):
                led.led = i
                pub.publish(led)
                rospy.rostime.wallsleep(0.1)
            #Read red
            led.R = 0
            led.G = 0
            led.B = 128
            led.group = rear
            for i in range(rear_count):
                led.led = i
                pub.publish(led)
                rospy.rostime.wallsleep(0.1)

            init = False

        led.group = lamp
        led.led = 0
        led.msecOn = 50
        led.msecOff = 0
        if(battery_voltage > 10.5):
            led.R = 255
            led.G = 0
            led.B = 0
        elif battery_voltage >10.2:
            led.R = 128
            led.G = 255
            led.B = 0
        else:
            led.R = 0
            led.G = 255
            led.B = 0
        pub.publish(led) 
        rospy.rostime.wallsleep(1.0)
