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

battery_warn_threshold = 10.9
battery_critical_threshold = 10.6


front = 3
lamp = 1
front_lamps = [lamp, 1]
rear = 0
front_count = 8
rear_count = 20


# R, G, B
color_yellow = [ 0x80, 40, 00]
color_dim_red = [16, 0, 0]
color_dim_white = [16, 16, 16]
color_green = [0, 16, 0]
color_black = [0, 0, 0]
front_colour = front_count * [color_yellow]
rear_colour  = 5 * [color_green] + 6  * [color_dim_white] + 5 * [color_dim_red] + [color_green]
front_lamps_color = [16, 16, 16]
lamp_max = 32

lamp_batt_ok = [lamp_max, lamp_max, lamp_max]
lamp_batt_warn = [lamp_max // 2, lamp_max, 0]
lamp_batt_critical = [0, lamp_max, 0]

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
    rospy.Subscriber('/unav2/status/battery', BatteryState, on_batterystate)
    pub = rospy.Publisher('/signalling/leds', blink, queue_size=40)
    rospy.loginfo("Waiting for led service")
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        rospy.sleep(0.05)
    rospy.loginfo("Connected")

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
            for i in range(len(front_colour)):
                led.R = front_colour[i][0]
                led.G = front_colour[i][1]
                led.B = front_colour[i][2]
                led.led = i
                pub.publish(led)
                rospy.rostime.wallsleep(0.1)

            #front lights
            led.group = front_lamps[0]
            led.R = front_lamps_color[0]
            led.G = front_lamps_color[1]
            led.B = front_lamps_color[2]
            led.led = front_lamps[1]
            pub.publish(led)
            rospy.rostime.wallsleep(0.1)

            #Read red
            led.group = rear
            for i in range(len(rear_colour)):
                led.R = rear_colour[i][0]
                led.G = rear_colour[i][1]
                led.B = rear_colour[i][2]

                led.led = i
                pub.publish(led)
                rospy.rostime.wallsleep(0.01)

            init = False

        led.group = lamp
        led.led = 0
        led.msecOn = 50
        led.msecOff = 0
        led.single = True
        if(battery_voltage > battery_warn_threshold):
            led.R = lamp_batt_ok[0]
            led.G = lamp_batt_ok[1]
            led.B = lamp_batt_ok[2]
        elif battery_voltage > battery_critical_threshold:
            led.R = lamp_batt_warn[0]
            led.G = lamp_batt_warn[1]
            led.B = lamp_batt_warn[2]
        else:
            led.R = lamp_batt_critical[0]
            led.G = lamp_batt_critical[1]
            led.B = lamp_batt_critical[2]
        pub.publish(led) 
        rospy.rostime.wallsleep(2.0)
