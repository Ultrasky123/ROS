import sys
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master.wait_heartbeat()

class Guidance():
    def __init__(self, master):
        self.master = master
        self.bootTime = time.time()
        self.rcValue = [1500] * 8
        

    def armDisarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('Armed!')

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        self.master.motors_disarmed_wait()
    
    def arm(self):
        self.master.arducopter_arm()
    
    def disarm(self):
        self.master.arducopter_disarm()

    def setMode(self, mode):
        print(mode)
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)


        modeId = self.master.mode_mapping()[mode]

        self.master.set_mode(modeId)
    
    def setRcValue(self, channel, pwm):
        self.rcValue[channel - 1] = pwm

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.rcValue
        )

    def setDepth(self, depth):
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.bootTime)),
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=(
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ),
            lat_int=0, lon_int=0, alt=depth,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        )
    
    def setHeading(self, degree, loop):
        for _ in range(0, loop, 1):
            self.master.mav.set_attitude_target_send(
                int(1e3 * (time.time() - self.bootTime)),
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
                QuaternionBase([math.radians(angle) for angle in (0, 0, degree)]),
                0, 0, 0, 0
            )
    
    
    # def open_gripper(self,servo_n, microseconds):
    #     self.master.mav.command_long_send(
    #     self.master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    #         0,            # first transmission of this command
    #         servo_n + 8,  # servo instance, offset by 8 MAIN outputs
    #         microseconds, # PWM pulse-width
    #         0,0,0,0,0     # unused parameters
    #         )
        
    #     for us in range(50, 1900, 1100): #min to max
    #         # set_servo_pwm(1, us)
    #         open_gripper(1, us)
    #         time.sleep(0.125)
    
    def grip_close(self):
      
        for us in range(50, 1900, 1100): #max to min
            # set_servo_pwm(1, us)
            self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # first transmission of this command
            1 + 8,  # servo instance, offset by 8 MAIN outputs
            us, # PWM pulse-width
            0,0,0,0,0     # unused parameters
            )
            time.sleep(0.125)
    
    def msg_alt_hdg(self):
    #informasi nilai altitude dan heading
        while True:
            msg = self.master.recv_match()
            
            # if not msg:
            #     continue
            if msg.get_type() == 'AHRS2':
                print("altitude: %s" % msg.altitude)
                rate=5
                time.sleep(1/rate)
            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                print("heading: %s" % msg.hdg)
                rate=15
                time.sleep(1/rate)


    def manual_control(self):
        
        self.master.mav.manual_control_send(
            self.master.target_system,
            500,  #nilai x (roll)
            -500, #nilai y (pitch)
            250,  #nilai z (throttle)
            500,  #nilai r (yaw)
            0)    #nilai button_change

        # To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
        # It's possible to check and configure this buttons in the Joystick menu of QGC
        buttons = 1 + 1 << 3 + 1 << 7
        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            500, # 500 means neutral throttle
            0,
            buttons)