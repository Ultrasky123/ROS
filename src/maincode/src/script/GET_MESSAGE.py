#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Int16
from pymavlink import mavutil


def publisher():
    rospy.init_node('Node_heading_altitude', anonymous=True)
    pub_heading = rospy.Publisher('heading', Int16, queue_size=10)
    pub_altitude = rospy.Publisher('altitude', Int16, queue_size=10)

    # Create the connection
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200) 

    while not rospy.is_shutdown():
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'AHRS2':
            altitude = msg.altitude
            pub_altitude.publish(altitude)
        if msg.get_type() == 'VFR_HUD':
            heading = msg.heading
            pub_heading.publish(heading)

        print("heading:", heading, "altitude:", altitude)
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
