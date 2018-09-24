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
# IMU Code -------------------------------------------

# Init. BNO055 object
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Axis remap values
AXIS_REMAP_X                         = 0x00
AXIS_REMAP_Y                         = 0x01
AXIS_REMAP_Z                         = 0x02
AXIS_REMAP_POSITIVE                  = 0x00
AXIS_REMAP_NEGATIVE                  = 0x01

bno.set_axis_remap(AXIS_REMAP_Y,AXIS_REMAP_X,AXIS_REMAP_Z,AXIS_REMAP_POSITIVE,AXIS_REMAP_NEGATIVE,AXIS_REMAP_POSITIVE)

# Instatiate Variables
heading, roll, pitch = bno.read_euler()

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


# Convert to relative degrees
def todeg(counts):
    mod = counts % 8192
    return float(mod)/8192*360 # Degrees

def tocounts(deg):
    return float(deg)/360*8192 # Counts

def bindto360(deg):
    if deg < 360:
        return deg
    elif deg > 360:
        return deg - 360

def bindto180(deg):
    if deg < 180:
        return deg
    else:
        return deg - 360

# Calibrate the absolute position --------------------
# Keep the robot still with two feet on the ground

# Find the pitch of the body
heading, roll, pitch = bno.read_euler()

# m0pos_offset = todeg((my_drive.axis0.encoder.pos_estimate)/4.5) + (18 + pitch)
# m0pos_offset = todeg((my_drive.axis0.encoder.pos_estimate)/4.5)
# m0pos_offset = (my_drive.axis0.encoder.pos_estimate)/4.5 + tocounts(pitch)

# Find the offset of the encoders (zeros out encoders) properly scaled with
# pulley ratio
m0pos_offset = (my_drive.axis0.encoder.pos_estimate)/4.5

# Find the offset for the absolute position of the wheel
# With two feet on the ground, the absolute position of the marked wheel should
# be 18 degrees (half of the angle between spokes)
beta0 = 18 - pitch

# Main loop
while True:
    try:
        # Get current position of encoder -> wheel (raw)
        m0pos = my_drive.axis0.encoder.pos_estimate/4.5

        # Convert to degrees (for DEBUG only)
        m0pos_deg = todeg(m0pos)

        # Get current position of wheel relative to the zeroed position
        theta = todeg(m0pos - m0pos_offset) #(wheel angle)
        # theta = todeg(-m0pos) + m0pos_offset

        # Get speed (for data only)
        m0speed = my_drive.axis0.encoder.vel_estimate

        # Convert speed to deg/s from counts/s
        speed = m0speed/8192*360 # Deg/s

        # Get current pitch of body
        heading, roll, pitch = bno.read_euler()

        # Find absolute position of the wheel relative to the ground - based on
        # the marked spoke and the absolute position offset
        # Then bind the absolute position to 360 degrees then to +/- 180 degrees
        # beta_diff = pitch - theta
        beta = bindto180(bindto360(pitch + theta + beta0)) ######

        # print('Counts: %i Degrees: %.4f Theta: %.4f Pitch: %.3f Beta: %.4f Speed: %.4f' % (m0pos,m0pos_deg,theta,pitch,beta,m0speed))
        # print('Degrees: %.4f Theta: %.4f Pitch: %.3f BetaDiff: %.4f BetaSum: %.4f' % (m0pos_deg,theta,pitch,beta_diff,beta))
        print('Theta: %.4f Pitch: %.3f Beta: %.4f Speed: %.4f' % (theta,pitch,beta,speed))
    except (KeyboardInterrupt):
        sys.exit()
