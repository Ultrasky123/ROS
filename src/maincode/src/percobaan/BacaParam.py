
# Import mavutil
from pymavlink import mavutil
import time
altitude = 0
heading = 0
# Create the connection;
# From topside computer
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

while True:
    # time.sleep(0.01)
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'AHRS2':
        altitude = msg.altitude
    if msg.get_type() == 'VFR_HUD':
        heading = msg.heading
        print("altitude : ", altitude,"heading : ",heading)
       