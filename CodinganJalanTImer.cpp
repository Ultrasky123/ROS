from PyMavlink import ROV
from pymavlink import mavutil
import time

#master = mavutil.mavlink_connection("/dev/ttyACM2", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

rov = ROV(master)


rov.arm()

print("altitud")
rov.setMode('ALT_HOLD')
print('Set 0.3')
rov.setDepth(-0.3)
time.sleep(10)
print('Set -0.7')
rov.setDepth(-0.7)
time.sleep(10)
print('Set 0.3')
rov.setDepth(-0.3)
time.sleep(10)
rov.setMode('MANUAL')

# print("maju")
# rov.setRcValue(5, 1700)
# time.sleep(15)
# print("mati")
# rov.setRcValue(5, 1500)
# print("Tarik Tuas")
# time.sleep(50)

# print("manual")
# rov.setDepth()
# rov.setMode('MANUAL')
# time.sleep(3)

# print("Naik")
# rov.setRcValue(3, 1600)
# time.sleep(5)
# rov.setRcValue(3, 1500)

rov.disarm()

#while True:
#    print(rov.getDataMessage('AHRS2').altitude)
#    time.sleep(0.1)
#