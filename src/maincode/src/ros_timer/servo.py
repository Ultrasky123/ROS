import time
# Import mavutil
from pymavlink import mavutil

def set_servo_pwm(servo_n, microseconds):
    
    # master.set_servo(servo_n+8, microseconds) or:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )

# Create the connection
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Wait a heartbeat before sending commands
master.wait_heartbeat()

# command servo_1 to go from min to max in steps of 50us, over 2 seconds
for us in range(50, 1900, 1100):
    set_servo_pwm(1, us)
    time.sleep(0.125)