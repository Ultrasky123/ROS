#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

heading = 0

def callbackheading(data):
    global heading
    heading = data.data
    #rospy.loginfo(rospy.get_caller_id()+'heading : %s',heading)

def main() :
    global heading, rate
    rospy.init_node('Node_Buffer_Sensor',anonymous=False)
    rospy.Subscriber('heading',Float32,callbackheading)
    pub_buffer_heading = rospy.Publisher('buffer_heading',Float32,queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub_buffer_heading.publish(heading)
        rospy.loginfo(rospy.get_caller_id() + 'data heading = %s',heading)

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
