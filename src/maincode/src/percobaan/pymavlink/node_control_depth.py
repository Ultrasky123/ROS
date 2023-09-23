"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Create the connection
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

def control_thruster(thrust_value):
    # check if the vehicle is armed
    if master.motors_armed():
        # set the thrust value for the thruster
        master.mav.manual_control_send(
            master.target_system,  # target_system
            1,  # target_component
            0,  # x-axis (roll)
            0,  # y-axis (pitch)
            0,  # z-axis (throttle)
            thrust_value,  # r-axis (yaw)
            0,  # buttons
        )
    print("ssss")
control_thruster(1400)
