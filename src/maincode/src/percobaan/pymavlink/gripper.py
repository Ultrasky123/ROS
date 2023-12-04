#!/usr/bin/env python

import time
# Import mavutil
from pymavlink import mavutil

#koneksi jika langsung komputer/nuc
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

#koneksi jika pakai companion
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

def open_gripper(servo_n, microseconds):
    master.mav.command_long_send(
    master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
        )
    for us in range(50, 1900, 1100): #min to max
        open_gripper(1, us)
        time.sleep(0.125)
    