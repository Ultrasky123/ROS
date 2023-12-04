#!/usr/bin/env python3

import time
import math
from pymavlink import mavutil
# import rospy
# from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import math
ALT_HOLD_MODE = 2


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
print('asdf')
master.wait_heartbeat()

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


# # Function to set target attitude
def set_target_attitude(roll, pitch, yaw):
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE |
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE |
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE |
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE,
        # Set rates to achieve desired attitude
        [roll, pitch, yaw, 0],
        0, 0, 0,  # Body rates not used
        thrust=0  # Throttle not used
    )

# # Set the desired operating mode (e.g., depth-hold mode)
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

# Set a depth target
set_target_depth(-0.5)

yaw_angle = 0

def object_heading(data):
    global yaw_angle
    # Mendapatkan data posisi objek dari subscriber
    x_obj = data.data
    print("x :",x_obj)

    # Mendapatkan posisi tengah gambar (misalnya, resolusi gambar horizontal 640 piksel)
    x_center = 320

    # Mendapatkan sudut pandang kamera (misalnya, 60 derajat)
    fov = 80

    # Menghitung sudut yaw yang diinginkan
    yaw_angle = math.degrees(math.atan((x_obj - x_center) / (320 / math.tan(math.radians(fov / 2)))))
    print(yaw_angle)
    # Mengirim perintah yaw ke ROV
    # rospy.loginfo("Nilai x : %d", data.data)

def control_rov_heading(yaw_angle):
    print('asdf')
    # Fungsi untuk mengontrol ROV menyesuaikan sudut yaw sesuai target yang dideteksi
    
    # panggil data center x dari object deteksi

    target_yaw = yaw_angle
    current_yaw = 0
    tolerance = 1  # Tolerance for stopping the rotation
    step_size = 40  # Step size for changing the yaw angle
    max_rotation_time = 4  # Maximum rotation time per step

    rate = rospy.Rate(10)
    while abs(target_yaw - current_yaw) > tolerance:
        #set yaw angle ke objek
        pitch = yaw = roll = 0
        for i in range(4): 
            set_target_attitude(roll, pitch, target_yaw) #ubah nilai Headingg
            print("set heading")
            time.sleep(1)

def object_detection_subscriber():
    # Inisialisasi ROS node
    rospy.init_node('setheadto_object', anonymous=True)
    rospy.Subscriber('center_x', Int16, object_heading) # Buat subscriber untuk menerima posisi objek dari topik "/center_x"
    control_rov_heading()

    # Loop ROS
    rospy.spin()

if __name__ == '__main__':
    object_detection_subscriber()
    

# Clean up (disarm) at the end
# master.arducopter_disarm()
# master.motors_disarmed_wait()
