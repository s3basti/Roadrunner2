#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
# 9 Axis Abs Orientation Sensor
from Adafruit_BNO055 import BNO055
# Shell Commands
import sys

# ----------------------------------------------------
# Controller Code ------------------------------------

# Find a connected ODrive (this will block until you connect one)
print("Finding an ODrive...")
my_drive = odrive.find_any()

print("Odrive found!")

calibrating = False

if not my_drive.axis0.motor.is_calibrated:
    print("Calibrating M0...")
    calibrating = True
    my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

if not my_drive.axis1.motor.is_calibrated:
    print("Calibrating M1...")
    calibrating = True
    my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

if calibrating:
    while (my_drive.axis0.current_state != AXIS_STATE_IDLE) or (my_drive.axis1.current_state != AXIS_STATE_IDLE):
        time.sleep(0.1)

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

my_drive.axis0.controller.current_setpoint = 0
my_drive.axis1.controller.current_setpoint = 0

my_drive.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
my_drive.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL

print("Motors Ready!")

# Main loop
while True:
    try:
        inp = input('Please enter current (in A): ')

        my_drive.axis0.controller.current_setpoint = -float(inp) # inverted
        my_drive.axis1.controller.current_setpoint = float(inp)

    except (KeyboardInterrupt):
        # Turn off the motors
        my_drive.axis0.requested_state = AXIS_STATE_IDLE
        my_drive.axis1.requested_state = AXIS_STATE_IDLE

        sys.exit()
