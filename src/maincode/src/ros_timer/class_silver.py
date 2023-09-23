 #!/usr/bin/env python3

import numpy as np
import sys
import time
import rospy
from pymavlink import mavutil
import os
os.environ['MAVLINK20'] = ''
from std_msgs.msg import Float32,Int16,UInt16



class Guidance:
    #ketika topik menerima data aka`n dimasukan kedalam variabel callback 
    def callback_heading(self, data):
        self.current_heading = data.data

    def callbacktargetheading (self, data):
        self.target_heading = data.data

    def callbackkpheading (self,data):
        self.kp = data.data

    def callbackkiheading (self,data):
        self.ki = data.data

    def callbackkdheading (self,data):
        self.kd = data.data


    #inisialisasi class object 
    def __init__(self, master):
        #pymavlink atribut
        self.master = master
        self.bootTime = time.time()
        self.RcValue = [1500] * 8


        #PID HEADING
        self.pid_output = 0
        self.current_heading = 0
        self.last_error = 0.0
        self.kp = 5.0
        self.ki = 0.0
        self.kd = 3.0
        self.error_sum = 0.0
        self.error=0
        self.target_heading = 0
        self.pwm_head =1500

        #PID DEPTH
        self.pid_depth = 0
        self.current_depth = 0
        self.last_error_depth = 0.0
        self.kp_depth = 12.0
        self.ki_depth = 0.01
        self.kd_depth= 3.0
        self.error_sum_depth = 0.0
        self.error_depth=0
        self.target_depth= 0
        self.pwm_depth=1500

        self.gripper = 2 #0 = tutup 1 = buka
        self.pub_gripper = rospy.Publisher('/sensor/gripper_command',UInt16,queue_size=10)
        
        #subsriber
        rospy.Subscriber("/sensor/buffer_heading", Float32, self.callback_heading)
        rospy.Subscriber("target_heading", Float32, self.callbacktargetheading)
        rospy.Subscriber("kp_heading", Int16, self.callbackkpheading)
        rospy.Subscriber("ki_heading", Int16, self.callbackkiheading)
        rospy.Subscriber("kd_heading", Int16, self.callbackkdheading)

        self.pub_pwm = rospy.Publisher("pwm_value",Int16,queue_size=10)
        

        rospy.sleep(2)

    # def set_gripper(self,mode,):
    #     if (mode != self.gripper):
    #         self.gripper = mode 
    #         self.pub_gripper.publish(self.gripper)
    #         rospy.loginfo(rospy.get_caller_id()+ 'gripper_value = %s', self.gripper)
        
    def set_gripper(self,mode,timeer):
        if (mode < 2) :
            if (mode != self.gripper):
                self.gripper = mode 
                data_grip = timeer*2 + mode
                self.pub_gripper.publish(data_grip)
                print('data grip: ',data_grip)
                rospy.loginfo(rospy.get_caller_id()+ 'gripper_value = %s', self.gripper)
        

    def setMode(self, mode):
        print(mode)
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)


        modeId = self.master.mode_mapping()[mode]

        self.master.set_mode(modeId)


    #menghitung PID yaw
    def PID_yaw(self):
        self.error = self.target_heading - self.current_heading

        if abs(self.error) < 180:
            self.error = self.error
        else:
            if self.error > 180:
                self.error -= 360
            if self.error < -180:
                self.error += 360

        # Proportional term
        p_term = self.kp * self.error

        # Integral term
        self.error_sum += self.error
        i_term = self.ki * self.error_sum

        # Derivative term
        d_term = self.kd * (self.error - self.last_error)

        # Calculate PID output
        self.pid_output = (p_term + i_term + d_term)

        # Update last error
        self.last_error = self.error

        if self.pid_output >=200:
            self.pid_output = 200
        if self.pid_output <= -200:
            self.pid_output = -200 

        self.pwm_head= int(1500+self.pid_output)
        # self.pub_pwm.publish(self.pwm_head)

        # rospy.loginfo(rospy.get_caller_id() + 'nilai heading : %s nilai pwm : %s',self.current_heading,self.pwm_head)

    def set_heading_target(self,heading):
        self.target_heading = heading

    # code khusus actuator yaw 
    def control_yaw(self):
        self.RcValue[4 - 1] = self.pwm_head

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.RcValue
        )
        rospy.loginfo(rospy.get_caller_id() + 'nilai heading : %s nilai pwm : %s error : %s',self.current_heading,self.pwm_head, self.error )

    
    def target_pid_depth(self,target):
        self.target_depth=target       

    def PID_depth(self):    

        self.error_depth = self.target_depth - self.current_depth

        # Proportional term
        p_term_depth= self.kp_depth * self.error_depth

        # Integral term
        self.error_sum_depth+= self.error_depth
        i_term_depth= self.ki_depth * self.error_sum_depth

        # Derivative term
        d_term_depth= self.kd_depth * (self.error_depth - self.last_error_depth)

        # Calculate PID output
        self.pid_depth = (p_term_depth + i_term_depth + d_term_depth)

        # Update last error
        self.last_error_depth = self.error_depth

        if self.pid_depth >=300:
            self.pid_depth = 300
        if self.pid_depth <= -300:
            self.pid_depth = -300 

        self.pwm_depth=int(1500+self.pid_depth)
        # self.pub_pwm_depth.publish(self.pwm_depth)
        print('error_depth: ',self.error_depth , 'pwm_depth :', self.pwm_depth)

        # rospy.loginfo(rospy.get_caller_id() + 'nilai heading : %s nilai pwm : %s',self.current_heading,self.pwm_head)

    def control_depth(self):
        self.RcValue[3 - 1] = self.pwm_depth

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.RcValue
        )
        # rospy.loginfo(rospy.get_caller_id() + 'nilai heading : %s nilai pwm : %s error : %s',self.current_heading,self.pwm_head, self.error )



    def control_forward(self,pwm):
        self.RcValue[5 - 1] = pwm

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.RcValue
        )
        rospy.loginfo(rospy.get_caller_id() + 'pwm_forward : %s' ,pwm)

        

    def setMode(self, mode):
        print(mode)
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)


        modeId = self.master.mode_mapping()[mode]

        self.master.set_mode(modeId)

    
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
    
    def setRcValue(self, channel, pwm):
        # RC Input & Output ArduSub
        # 1 => Roll
        # 2 => Pitch
        # 3 => Throttle
        # 4 => Yaw
        # 5 => Forward/Backward
        # 6 => Lateral

        self.RcValue[channel - 1] = pwm

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.RcValue
        )

    def resetpwm(self):
        for i in range (1,8,1):
            self.setRcValue(i,1500)
            self.master.wait_heartbeat()
            time.sleep(1)

    def set_target_depth(self,depth):
        self.master.mav.set_position_target_global_int_send(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
            0b0000111111111000,
            0,0, depth,
            0 , 0 , 0 , # x , y , z velocity in m/ s ( not used )
            0 , 0 , 0 , # x , y , z acceleration ( not supported yet , ignored in GCS Mavlink )
            0 , 0 ) # yaw , yawrate ( not supported yet , ignored in GCS Mavlink )

    def arm (self):
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
        self.master.motors_armed_wait()
        print("arm")

    def disarm (self):
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
        self.master.motors_disarmed_wait()
        print("disarm")
    
    def reboot (self):
        self.master.reboot_autopilot()

    def get_depth (self):
        msg = self.master.recv_match(type="AHRS2", blocking=True)
        print('DEPTH:', 100*(msg.altitude))
        self.current_depth=100*(msg.altitude)

    