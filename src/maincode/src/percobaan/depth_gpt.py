#!/usr/bin/env python3

from pymavlink import mavutil
import time

# Koneksi ke Pixhawk melalui Serial 4/5
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Create the connection5
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Mengirim perintah ke Pixhawk untuk mempertahankan kedalaman tertentu
target_depth = -0.5 # kedalaman yang diinginkan dalam meter
msg = master.mav.set_position_target_local_ned_encode(
    0, # waktu dalam milidetik
    0, 0, 0, # frame koordinat, menggunakan frame inersia
    0b0000111111000111, # bit mask kontrol (posisi dan kecepatan)
    0, 0, 0, # x, y, z, posisi tidak digunakan
    0, 0, -target_depth, # x, y, z, kecepatan digunakan untuk kedalaman
    0, 0, 0, # x, y, z, akselerasi tidak digunakan
    0, 0) # yaw, yaw_rate tidak digunakan
master.mav.send(msg)
print(msg)