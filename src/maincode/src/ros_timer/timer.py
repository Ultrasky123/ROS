import os
os.environ['MAVLINK20'] = ''
import rospy
from std_msgs.msg import UInt16
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
ALT_HOLD_MODE = 2

# Buat fungsi untuk mengulangi set target altitude setiap 5 detik
# Disarm
# master.arducopter_disarm() or:

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
    

def getheading ():
    msg = master.recv_match(type="VFR_HUD", blocking=True)
    print(msg)
    return msg.heading

def getdepth ():
    msg = master.recv_match(type="AHRS2", blocking=True)
    print(msg)
    return msg.altitude


# Create the connection
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
print(master.wait_heartbeat())

def main():
    rospy.init_node('Node_timer_ros', anonymous=True)
    gripper_pub = rospy.Publisher('/sensor/gripper_command', UInt16, queue_size=10)
    
    for i in range (1,8,1):
        setRcValue(i,1500)
        master.wait_heartbeat()
    time.sleep(1)

    #SET ARM
    master.arducopter_arm()

    setRcValue(5,1700)
    # setRcValue(3,1700)
    time.sleep(4)
    setRcValue(5,1500)
    setRcValue(3,1500)


    master.arducopter_disarm()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



