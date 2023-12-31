import time
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase
from PyMavlinkin import ROV

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#jika koneksi langsung komputer
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()


def main(pm: Guidance):

    #arm thruster
    pm.arm()
    

    # set the desired operating mode
    DEPTH_HOLD = 'ALT_HOLD'
    DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
    while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
        master.set_mode(DEPTH_HOLD)
    
    # go for a spin
    # (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
    roll_angle = pitch_angle = 0
    for yaw_angle in range(0, 500, 10):
        pm.setDepth(-0.5)

        pm.setRcValue(5,1600)
        print("maju")
        time.sleep(4)
        pm.setRcValue(5,1500)

    # spin the other way with 3x larger steps
    # for yaw_angle in range(0, 60, 10):
    #     pm.set_target_attitude(roll_angle, pitch_angle, 240)
    #     time.sleep(3)
    #      #maju
        
    
   

    # pm.disarm()


if __name__ == '__main__':
    pymavlink = Guidance(master)

    main(pymavlink)