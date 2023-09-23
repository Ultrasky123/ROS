#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Int16, String,Bool

target_heading = 0
target_depth = -20
target_forward = 1500


def FORWARD(timer , heading , depth, forward):
    dt = 0
    start_time = time.time()
    while (dt < timer ):
        current_time = time.time()
        dt = current_time- start_time
        pub_depth.publish (depth)
        pub_heading.publish (heading)
        pub_forward.publish(forward)
        rospy.loginfo(rospy.get_caller_id() + ' forward ,timer: %s  ',dt)
    pub_forward.publish(target_forward)
        
def YAW(timer, heading, depth):
    dt = 0
    start_time = time.time()
    while (dt < timer ):
        current_time = time.time()
        dt = current_time- start_time 
        pub_depth.publish (depth)
        pub_heading.publish (heading)
        rospy.loginfo(rospy.get_caller_id() + ' yaw ,timer: %s  ',dt)

def SURFACE(timer):
    dt = 0
    start_time = time.time()
    while (dt < timer ):
        current_time = time.time()
        dt = current_time - start_time
        pub_depth.publish (target_depth)
        rospy.loginfo(rospy.get_caller_id()+ " Surfacing")
    pub_arming.publish(False)

   
def GRIP(mode,duration):
    pub_grip_duration.publish(duration)
    pub_gripper.publish(mode)
    rospy.loginfo(rospy.get_caller_id()+ " GRIPPING")

def main () :
    global target_heading,target_depth,target_forward
    global pub_heading,pub_depth,pub_forward,pub_arming,pub_gripper,pub_grip_duration
    
    rospy.init_node('node_lawnmower',anonymous=False)
    pub_heading = rospy.Publisher('target_heading',Int16, queue_size=10)
    pub_depth = rospy.Publisher('target_depth',Int16, queue_size=10)
    pub_forward = rospy.Publisher('target_forward',Int16, queue_size=10)
    pub_arming = rospy.Publisher('target_arming',Bool, queue_size=10)
    pub_gripper = rospy.Publisher('target_gripper',Int16,queue_size=10)
    pub_grip_duration =rospy.Publisher('target_grip_duration',Int16,queue_size=10)
    
    #delay booting
    rospy.sleep(5)

    # while not rospy.is_shutdown() :
        # try:


    #=============MISI WAJIB============

    #FORWARD(timer,heading,TIMER,pwm)
    FORWARD(10,25,-70,1800)
    FORWARD(10,25,-70,1800)

    #YAW(timer, heading, depth)
    #YAW(4,180+95,-50)


    #GRIP(mode,duration)
    # GRIP(0,1500) #CLOSE
    # GRIP(1,1500)  #OPEN  
    
    SURFACE(2)



if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
