#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool
import time
from pymavlink import mavutil

#jika koneksi langsung komputer/nuc
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

#koneksi jika pakai companion
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

def callbackX(data):
    rospy.loginfo('Gripper X')
    

def callbackY(data):
    rospy.loginfo('Gripper Y')
    
def main():
    def close_gripper(servo_n, microseconds):
        master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
        )
        
    for us in range(50, 1900, 1100): #max to min
        # set_servo_pwm(1, us)
        close_gripper(1, us)
        time.sleep(0.125)
    

    rospy.init_node('node_gripper', anonymous=True)
    pub = rospy.Publisher('is_success', Bool, queue_size=10)
    rospy.Subscriber("coordinate_x", Float32, callbackX)
    rospy.Subscriber("coordinate_y", Float32, callbackX)
    

    rospy.loginfo(True)
    pub.publish(True)
    
    rospy.spin()

if __name__ == '__main__':
    main()