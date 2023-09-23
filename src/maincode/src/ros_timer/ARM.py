# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
#master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#jika koneksi langsung komputer
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Arm
# master.arducop5ter_arm() or:
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

