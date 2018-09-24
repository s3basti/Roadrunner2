#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("Finding an ODrive...")
my_drive = odrive.find_any()

print("Odrive found!")

calibrating = False

armed = False
latched = True

# ----- VELOCITY LIMIT -----
mph = 8.5 # Speed in MPH
vel_limit = mph*1.4666667/(2*3.1416*1.70833)*4.5*8192
print('Velocity limit is %i' % vel_limit)
# --------------------------

prev_speedL = 0
prev_speedR = 0
prev_speed = 0

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

armed = True

print("Motors Ready!")

while True:
    m0speed = abs(my_drive.axis0.encoder.pll_vel)
    m1speed = abs(my_drive.axis1.encoder.pll_vel)
    print('M0 Speed: %.4f, M1 Speed: %.4f' % (m0speed, m1speed))
