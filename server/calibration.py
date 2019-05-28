#!/usr/bin/python3
# File name   : calibration.py
# Description : Calibrate the servo motors.
# Author      : Ha Do
# Date        : 2019/5/8
from __future__ import division
import time
import turn

def turn_streering_wheel(min_angle=200, max_angle=800, step_size=20, time_step=3):
	# Turn streering wheel from min_angle to max_angle by step_size time_step seconds 
	for angle in range(min_angle, max_angle, step_size):
		print("Angle: ", angle)
		turn.pwm.set_pwm(2, 0, angle)
		time.sleep(time_step)

def check_steering_wheel():
	# Check the steering wheel at the middke, left, right angles	
	turn.middle()
	time.sleep(2)

	turn.right()
	time.sleep(2)

	turn.middle()
	time.sleep(2)

	turn.left()
	time.sleep(2)

	turn.middle()
	time.sleep(2)

def turn_head(min_angle=400, max_angle=600, step_size=20, time_step=3):
	# Turn head min_angle to max_angle by step_size time_step seconds 
	for angle in range(min_angle, max_angle, step_size):
		print("Angle: ", angle)
		turn.pwm.set_pwm(0, 0, angle)
		time.sleep(time_step)
		
#turn_streering_wheel(300, 700, 20)
## Right: 320, Left: 660, Middle: 500 
#check_steering_wheel()

#turn_head()
#turn.pwm.set_pwm(0, 0, 100)
#time.sleep(1)

#turn.pwm.set_pwm(0, 0, 800)
#time.sleep(1)
