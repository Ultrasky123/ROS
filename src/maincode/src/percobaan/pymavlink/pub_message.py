#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from pymavlink import mavutil

# Create the connection
# tanpa companion
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# pake companion
# master = mavutil.mavlink_connection('udp:0.0.0.0:14550')


def publish_message():
    # Set up publisher
    pub = rospy.Publisher('message', Float64, queue_size=10)
    rospy.init_node('message_publisher', anonymous=True)
    # rate = rospy.Rate(100) 
    while not rospy.is_shutdown():
        msg = master.recv_match()

        if not msg:
            continue
        if msg.get_type() == 'AHRS2':
            altitude = msg.altitude
            rospy.loginfo("altitude : %f",altitude)
            pub.publish(altitude)

        # elif msg.get_type() == 'GLOBAL_POSITION_INT':
        #     heading = msg.hdg
        #     rospy.loginfo("heading : %f",heading)
        #     pub.publish(heading)

        # rate.sleep()


if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
