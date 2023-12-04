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

def mode_is(mode):
    try:
        return bool(master.wait_heartbeat().custom_mode == mode)
    except:
        return False

def set_target_depth(depth):
    master.mav.set_position_target_global_int_send(
        0,     
        0, 0,   
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
        0b0000111111111000,
        0,0, depth,
        0 , 0 , 0 , # x , y , z velocity in m/ s ( not used )
        0 , 0 , 0 , # x , y , z acceleration ( not supported yet , ignored in GCS Mavlink )
        0 , 0 ) # yaw , yawrate ( not supported yet , ignored in GCS Mavlink )

def set_target_attitude(roll, pitch, yaw, control_yaw=True):
    bitmask = (1<<6 | 1<<3)  if control_yaw else 1<<6

    master.mav.set_attitude_target_send(
        0,     
        0, 0,   
        bitmask,
        QuaternionBase([math.radians(roll), math.radians(pitch), math.radians(yaw)]), # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        0, #roll rate
        0, #pitch rate
        0, 0)    # yaw rate, thrust 

def getheading ():
    msg = master.recv_match(type="VFR_HUD", blocking=True)
    print(msg)
    return msg.heading


def getdepth ():
    msg = master.recv_match(type="AHRS2", blocking=True)
    print("Depth :",msg.altitude)
    return msg.altitude

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
# Create the connection
#master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
print(master.wait_heartbeat())



#================MAIN PROGRAM===============
#ARM THRUSTER
master.arducopter_arm()
print('arm')

#DEPTH HOLD
#while not mode_is(ALT_HOLD_MODE):
#    master.set_mode('ALT_HOLD')

# #get info heading&depth
#heading=getheading()
#set_target_depth(-0.4) #set depth

#SET HEADING
#pitch = yaw = roll = 0
#for i in range(4): 
#    set_target_attitude(roll, pitch, heading) #ubah nilai Heading ke current heading
#    print("set heading")
#    time.sleep(1)

#get info heading&depth
#heading,depth=getheading(),getdepth()
#set_target_depth(-0.4) #set depth

#MAJU 10 METER
setRcValue(5,1500)
print("maju")
time.sleep(1)
setRcValue(5,1500)
print("maju")

#koneksi ke serial arduino
#ser = serial.Serial('/dev/ttyACM0', 9600)

# Mengirim instruksi ke Arduino buka gripper
#ser.write(b'buka\n')  
#print("jgtf")
# Menutup koneksi serial
#ser.close()

#DISARM THRUSTER
master.arducopter_disarm()
print('disarm')
