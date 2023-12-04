import RPi.GPIO as GPIO
import os
os.environ['MAVLINK20'] = ''
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()


master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

#set GPIO BCM
GPIO.setmode(GPIO.BCM)

# Tentukan nomor pin yang akan digunakan
button_pin = 4

# Set pin sebagai input dengan pull-up resistor
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)




try:
    while True:
        # Baca status tombol
        button_state = GPIO.input(button_pin)

        # Cek apakah tombol ditekan (LOW)
        if button_state == GPIO.LOW:
            master.motors_armed_wait()
            print("System ON!")
        
    else:
            master.motors_disarmed_wait()
            print("System OFF!")
except KeyboardInterrupt:
    GPIO.cleanup()