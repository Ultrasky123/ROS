"""
Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
in Ardupilot. These effectively replace the input channels (from joystick
or radio), NOT the output channels going to thrusters and servos.
"""
import time
# Import mavutil
import os
os.environ['MAVLINK20'] = ''
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
# master.wait_heartbeat()
# master.arducopter_arm()
# master.motors_armed_wait()
# DEPTH_HOLD = 'STABILIZE'
# DEPTH_HOLD = 'MANUAL'

# master.wait_heartbeat()
# master.set_mode(DEPTH_HOLD)
# # time.sleep(5)
# master.wait_heartbeat()

# while True:
#     msg = master.recv_match()
#     if not msg:
#         continue
#     if msg.get_type() == 'AHRS2':
#         altitude = msg.altitude
#         print("altitude : ",altitude)
#     elif msg.get_type() == 'GLOBAL_POSITION_INT':
#         heading = msg.hdg
#         print("heading : ",heading)

# master.arducopter_disarm()
# master.motors_disarmed_wait()
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

set_rc_channel_pwm(5, 1650)
time.sleep(2)
set_rc_channel_pwm(5,1500)

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
