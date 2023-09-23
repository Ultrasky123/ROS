
import os
os.environ['MAVLINK20'] = ''
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
ALT_HOLD_MODE = 2


#koneksi companion
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#jika koneksi langsung komputer
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Wait a heartbeat before sending commands
master.wait_heartbeat()
boot_time = time.time()

def control_rov(rect, frame_width, frame_height):
    centering_zone_width = 320
    centering_zone_height = 240
    centering_zone_x = frame_width / 2 - centering_zone_width / 2
    centering_zone_y = frame_height / 2 - centering_zone_height / 2
    
    center_zone_width = 300
    center_zone_height = 450
    center_zone_x = frame_width / 2 - centering_zone_width / 2
    center_zone_y = frame_height / 2 - centering_zone_height / 2

    target_x, target_y, target_w, target_h = rect
    # jika box tepat di tengah jalan
    if (target_x >= centering_zone_x and target_x + target_w <= centering_zone_x + centering_zone_width and
        target_y >= centering_zone_y and target_y + target_h <= centering_zone_y + centering_zone_height):
        # Stop moving
        print("go maju")
    
    #jika box sudah dekat 
    elif(target_x >= center_zone_x and target_x + target_w <= center_zone_x + center_zone_width and
        target_y >= center_zone_y and target_y + target_h <= center_zone_y + center_zone_height):
        print("stop")
    else:
        if target_x < frame_width / 2-10:
            # setRcValue(6,1400)
            print("Move kiri")
        elif target_x > frame_width / 2+10 :
            # setRcValue(6,1600)
            print("Move Kanan")
        # elif target_y > 
         

def get_heading():
    #jika koneksi langsung komputer/nuc

        while True:
            
            msg = master.recv_match()
            if not msg:
                continue
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                print("depth: %s" % msg.hdg)
                return(msg.hdg)


def closeGripper(servoN, microseconds):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # first transmission of this command
            servoN + 8,  # servo instance, offset by 8 MAIN outputs
            microseconds, # PWM pulse-width
            0,0,0,0,0     # unused parameters
        )
    

def armdisarm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

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


def set_target_attitude(roll, pitch, yaw):
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

    
def disarm():
    master.arducopter_disarm()
    return True

def is_armed():
    try:
        return bool(master.wait_heartbeat().base_mode & 0b10000000)
    except:
        return False

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
    

#======RC CHANNEL PWM======
    # 1 	Pitch
    # 2 	Roll
    # 3 	Throttle
    # 4 	Yaw
    # 5 	Forward
    # 6 	Lateral
