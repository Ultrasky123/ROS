#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from pymavlink import mavutil





def message_callback(data):
    #info isi data yang di terima
    # rospy.loginfo(data.data)
    print(data.data)

    #jika koneksi langsung komputer
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

    #koneksi jika pakai companion
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')    

    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

    def set_rc_channel_pwm(channel_id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
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
            *rc_channel_values)                   # RC channel list, in microseconds.



        if float(data.data) < -1.3:
        #     # throttle naik turun
        # control_thruster(1500)
        # print("nyala")
            set_rc_channel_pwm(3,1550)

def message_subscriber():
    rospy.init_node('sub_message', anonymous=True)
    rospy.Subscriber("message", Float64, message_callback)
    rospy.spin()

if __name__ == '__main__':
    message_subscriber()
