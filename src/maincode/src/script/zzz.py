

# Import mavutil
import serial
import os
os.environ['MAVLINK20'] = ''
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
import sys
ALT_HOLD_MODE = 2


def setRcValue(channel_id, pwm=1500):
  
            if channel_id < 1 or channel_id > 18:
                print("Channel does not exist.")
                return

            # Mavlink 2 supports up to 18 channels:
            # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
            rc_channel_values = [65535 for _ in range(18)]
            rc_channel_values[channel_id - 1] = pwm
            master.mav.rc_channels_override_send(
                master.target_system,                # target_system
                master.target_component,             # target_component
                *rc_channel_values)                  # RC channel list, in microseconds.




def arm ():
     #code untuk arm disarm
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
    print("==ARM==")

def disarm ():
    #code untuk arm disarm
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
    print("==DISARM==")


def setMode(mode):
        print(mode)
        if mode not in master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(master.mode_mapping().keys()))
            sys.exit(1)


        modeId = master.mode_mapping()[mode]

        master.set_mode(modeId)

#===============================================================
# Create the connection
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
print(master.wait_heartbeat())



#================MAIN PROGRAM===============
#ARM THRUSTER
master.arducopter_arm()
print('arm')

setMode('MANUAL')

for i in range (1,8,1):
    setRcValue(i,1500)
    master.wait_heartbeat()
time.sleep(1)



master.wait_heartbeat()

setRcValue(5,1600)
time.sleep(2)
master.wait_heartbeat()
setRcValue(5,1500)


master.arducopter_disarm()
print('disarm')
