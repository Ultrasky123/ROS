
# Import mavutil
from pymavlink import mavutil

# Create the connection
# Need to provide the serial port and baudrate
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Restart the ArduSub board !
master.reboot_autopilot()
