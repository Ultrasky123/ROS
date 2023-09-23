# Import mavutil
import os
os.environ['MAVLINK20'] = ''
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
import sys
ALT_HOLD_MODE = 2


# Create the connection
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

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
    print("heading:",msg.heading)
    return msg.heading

def get_heading():
    while True:
        
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            print("heading: %s" % msg.heading)
            return(msg.heading)
            rate = 10

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
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print("disarm")

def disarm ():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
    master.motors_disarmed_wait()
    print("disarm")


#================MAIN PROGRAM================

#ARM THRUSTER
# wait until arming confirmed (can manually check with master.motors_armed())
master.arducopter_arm()
print('Armed!')

#DEPTH HOLD
while not mode_is(ALT_HOLD_MODE):
    master.set_mode('ALT_HOLD')

heading=get_heading()
set_target_depth(-0.1)

for lawn in range(2):
    #SET HEADING
    pitch = yaw = roll = 0
    for i in range(3): 
        set_target_attitude(roll, pitch, heading) #ubah nilai Headingg
        print("set heading")
        time.sleep(1)

    print(heading)
    set_target_depth(-0.5)
    # time.sleep(1)

    # #MAJU 
    setRcValue(5,1700)
    time.sleep(11)
    setRcValue(5,1500)
    print("maju")

    #get info heading
    set_target_depth(-0.5) #set depth
    time.sleep(1)

    #Belok Kanan
    for i in range (3):
        set_target_attitude(roll,pitch,heading-90)
        print("belok kanan")
        time.sleep(1)
        
    #maju sedikit
    setRcValue(5,1700)
    time.sleep(3)
    setRcValue(5,1500)
    print("maju dikit ")

    #get info heading
    heading=get_heading()
    print(heading)
    set_target_depth(-0.5) #set depth
    time.sleep(1)

    #Belok Kanan
    for i in range (1):
        set_target_attitude(roll,pitch,heading-90-80)
        print("belok kanan")
        time.sleep(1)
        
    #maju 
    setRcValue(5,1700)
    time.sleep(9)
    setRcValue(5,1500)
    print("majuu")

    #get info heading
    heading=get_heading()
    set_target_depth(-0.5) #set depth
    time.sleep(1)
    
    #Belok Kiri
    for i in range (3):
        set_target_attitude(roll,pitch,heading-90-80+90)
        print("belok kanan")
        time.sleep(1)


for i in range (3):
    roll=pitch=yaw=0
    set_target_attitude(roll,pitch,heading+70)
    print("belok kanan")
    time.sleep(1)

#SWAY
setRcValue(5,1700)
time.sleep(6)
setRcValue(5,1500)
print("geser dikit")

for i in range (3):
    set_target_attitude(roll,pitch,heading-70)
    print("belok kanan")
    time.sleep(1)

#maju 
setRcValue(5,1700)
time.sleep(3)
setRcValue(5,1500)


#===========END===============
#DISARM THRUSTER
print("adawewa")
master.arducopter_disarm()
print('Disarm')
