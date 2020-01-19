#!/usr/bin/env python3
import rospy
import time
from os import path

#import adafruit_ssd1306
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess
import math
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState


def get_ip_address(interface):
    if get_network_interface_state(interface) == 'down':
        return None
    cmd = "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'" % interface
    return subprocess.check_output(cmd, shell=True).decode('ascii')[:-1]


def get_network_interface_state(interface):
    return subprocess.check_output('cat /sys/class/net/%s/operstate' % interface, shell=True).decode('ascii')[:-1]


user_text = None
user_text_timeout = 0


def on_user_text(msg):
	global user_text
	global user_text_timeout
	user_text = msg.data
	user_text_timeout = 10
	rospy.loginfo(rospy.get_caller_id() + ' user_text=%s', msg.data)


battery_voltage = 0
battery_current = 0
battery_charge = 0
battery_percentage = 0


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

	# 128x32 display with hardware I2C:
	# setting gpio to 1 is hack to avoid platform detection
	disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1)

	# Initialize library.
	disp.begin()

	# Clear display.
	disp.clear()
	disp.display()

	# Create blank image for drawing.
	# Make sure to create image with mode '1' for 1-bit color.
	width = disp.width
	height = disp.height
	image = Image.new('1', (width, height))

	# Get drawing object to draw on image.
	draw = ImageDraw.Draw(image)

	# Draw a black filled box to clear the image.
	draw.rectangle((0, 0, width, height), outline=0, fill=0)

	# Draw some shapes.
	# First define some constants to allow easy resizing of shapes.
	padding = -2
	top = padding
	bottom = height-padding
	# Move left to right keeping track of the current x position for drawing shapes.
	x = 0

	# Load default font.
	font = ImageFont.load_default()
	# Load a symbols font
	size = 11
	script_dir = path.dirname(__file__)
	fontSymbols = ImageFont.truetype(
	    path.join(script_dir, "./fonts/typicons.ttf"), size)

	# setup ros node
	rospy.init_node('jetbot_oled')
	rospy.Subscriber('~user_text', String, on_user_text)
	rospy.Subscriber('/battery_state', BatteryState, on_batterystate)

	counter = 0
	# start running
	while not rospy.core.is_shutdown():
		page = (math.trunc(counter / 5)) % 2
		counter += 1

		# Draw a black filled box to clear the image.
		draw.rectangle((0, 0, width, height), outline=0, fill=0)

		if not user_text is None:
			draw.text((x, top), user_text,  font=font, fill=255)
			user_text_timeout -= 1
			if(user_text_timeout < 0):
				user_text = None

		if(page == 0):
			# Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
			#cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
			#CPU = subprocess.check_output(cmd, shell = True )
			cmd = "free -m | awk 'NR==2{printf \"%s/%sMB %.0f%%\", $3,$2,$3*100/$2 }'"
			MemUsage = subprocess.check_output(cmd, shell=True)
			cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB %s\", $3,$2,$5}'"
			Disk = subprocess.check_output(cmd, shell=True)
			if user_text is None:
				draw.text((x, top), u"\ue09c",  font=fontSymbols, fill=255)
				draw.text((x + 14, top), str(get_ip_address('eth0')), font=font, fill=255)
			draw.text((x, top + 8), u"\ue146",  font=fontSymbols, fill=255)
			draw.text((x + 14, top+8), str(get_ip_address('wlan0')), font=font, fill=255)
			draw.text((x + 14, top+16), "Mem" +
			          str(MemUsage.decode('utf-8')),  font=font, fill=255)
			draw.text((x, top + 25), u"\ue07e",  font=fontSymbols, fill=255)
			draw.text((x + 14, top+25), str(Disk.decode('utf-8')),  font=font, fill=255)
		elif(page == 1):
			draw.text((x + 14, top+8), '{0:.2g}V {1:.2g}A'.format(
			    battery_voltage, battery_current),  font=font, fill=255)
			draw.text((x + 14, top+16), '{0:.1g}mAh {1:0g}%'.format(
			    battery_charge, battery_percentage * 100),  font=font, fill=255)
			draw.text((x, top + 8), u"\ue02a",  font=fontSymbols, fill=255)
		# Write two lines of text.

		# Display image.
		disp.image(image.rotate(180))
		disp.display()

		# Update ROS
		rospy.rostime.wallsleep(1.0)
