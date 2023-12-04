#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from pymavlink import mavutil

# Create the connection
#tanpa companion
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
#pake companion
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')


def publish_heading():
    # Set up publisher
    pub = rospy.Publisher('heading', Float64, queue_size=10)
    rospy.init_node('heading_publisher', anonymous=True)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        msg = master.recv_match()
        
        if not msg:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            # print("heading: ")
            heading = msg.heading
            rospy.loginfo(heading)
            pub.publish(heading)
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_heading()
    except rospy.ROSInterruptException:
        pass
