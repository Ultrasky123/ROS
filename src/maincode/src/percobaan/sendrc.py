"""
Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
in Ardupilot. These effectively replace the input channels (from joystick
or radio), NOT the output channels going to thrusters and servos.
"""
import os
os.environ['MAVLINK20'] = ''
from pymavlink import mavutil
import time
# Import mavutil
heading = 0
altitude = 0

setpoint = 15300
print('setpoint ',setpoint)
kp = 0.25
kd = 0.25
pwm = 0
error = 0
lastError = 0
deltaError = 0
# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print ('heartbeat confirmed')


def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(channel_id, pwm=1500):
    master.wait_heartbeat()

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
        *rc_channel_values)                  # RC channel list, in microseconds.



# 1	Pitch
# 2	Roll
# 3	Throttle
# 4	Yaw
# 5	Forward
# 6	Lateral
# 7	Camera Pan
# 8	Camera Tilt*
# 9	Lights 1 Level
# 10	Lights 2 Level
# 11	Video Switch

master.arducopter_arm()
master.motors_armed_wait()

# master.wait_heartbeat()
# print ('heartbet confirmed')
# Set some roll
# master.set_servo(3, 1550)
# # set_rc_channel_pwm(3, 1550)
# time.sleep(1)
# master.set_servo(3, 1500)
# # set_rc_channel_pwm(3, 1500)
# time.sleep(1)

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 2)

# Configure ATTITUDE message to be sent at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2)

# master.wait_heartbeat()
while True:
    msg = master.recv_match()
    if not msg:
        # set_rc_channel_pwm(4, 1500+pwm)

        continue
    if msg.get_type() == 'AHRS2':
        altitude = msg.altitude
    if msg.get_type() == 'GLOBAL_POSITION_INT':
        heading = msg.hdg
        error = (setpoint-heading)
        deltaError = (error - lastError)/5
        pwm = int(kp*error+kd*deltaError)
        lastError = error
        if pwm > 400:
            pwm = 400
        if pwm <-400:
            pwm = -400
    print("alt: ", altitude,"head: ",heading, 'PWM: ',pwm,'error: ',error,'delta: ',deltaError)
    set_rc_channel_pwm(5,1600)
    time.sleep(2)
    set_rc_channel_pwm(5,1500)
        
    # time.sleep(0.01)
    # print ()
    # if heading > 21200:
    #     master.wait_heartbeat()
    #     set_rc_channel_pwm(4, 1550)
    #     print ('1550')
    # if heading < 21200:
    #     master.wait_heartbeat()
    #     set_rc_channel_pwm(4, 1450)
    #     print ('1450')

        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("Message: %s" % msg)
        # print("\nAs dictionary: %s" % msg.to_dict())
        # # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
        # print("\nSystem status: %s" % msg.system_status)




# master.set_servo(2, 1700)
time.sleep(1)
master.arducopter_disarm()
master.motors_disarmed_wait()
# # Set some yaw

# # The camera pwm value sets the servo speed of a sweep from the current angle to
# #  the min/max camera angle. It does not set the servo position.
# # Set camera tilt to 45º (max) with full speed
# set_rc_channel_pwm(8, 1900)

# # Set channel 12 to 1500us
# # This can be used to control a device connected to a servo output by setting the
# # SERVO[N]_Function to RCIN12 (Where N is one of the PWM outputs)
# set_rc_channel_pwm(12, 1500)