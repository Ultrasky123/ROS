"""
Example of how to filter for specific mavlink messages coming from the
autopilot using pymavlink.

Can also filter within recv_match command - see "Read all parameters" example
"""
# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
# From topside computer
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

while True:
    msg = master.recv_match()
    
    if not msg:
        continue
    if msg.get_type() == 'AHRS2':
        print("altitude: %s" % msg.altitude)
        rate=5
        time.sleep(1/rate)
    elif msg.get_type() == 'GLOBAL_POSITION_INT':
        print("heading: %s" % msg.hdg)
        rate=15
        time.sleep(1/rate)