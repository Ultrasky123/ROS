import time
from pymavlink import mavutil
from PyMavlinkzz import Guidance

#koneksi jika langsung komputer/nuc
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

#koneksi jika pakai companion
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()


# kamus set rcvalue channel
# 1     = Pitch
# 2 	= Roll
# 3 	= Throttle         (nyelam/surface)
# 4 	= Yaw              (Belok)
# 5 	= Forward          (maju)
# 6 	= Lateral          (menyamping)
# 7 	= Camera Pan       
# 8 	= Camera Tilt*
# 9 	= Lights 1 Level
# 10 	= Lights 2 Level
# 11 	= Video Switch

def main(pm: Guidance):
    
    pm.armDisarm()
    
    pm.arm()

    # setmode manual
    pm.setMode('MANUAL')

    #Robot menyelam
    pm.setRcValue(5, 1700)  
    time.sleep(3)
    pm.setRcValue(5, 1500)  

    # setmode manual
    # pm.setMode('ALT_HOLD')

    #yaw liat kanan kiri opsi 1
    # pm.setRcValue(4, 1450) #kiri    
    # time.sleep(1)
    # pm.setRcValue(4, 1550) #kiri    
    # time.sleep(1)

    #perbaiki arah gerak
    # pm.setHeading(360, 4)
    
    #yaw liat kanan kiri opsi 2
    # pm.setHeading(315, 1)
    # pm.setHeading(45,1)
    # pm.setHeading(360,4)

    # gerak maju sampe mendekati tembok
    # pm.setRcValue(5 ,1700)
    # time.sleep(4)

    # gerak menyamping
    # pm.setRcValue(6 ,1440)
    # time.sleep(1)

    # # gerak mundur sampe mendekati tembok 
    # pm.setRcValue(5 ,1300)
    # time.sleep(4)

    # # gerak menyamping
    # pm.setRcValue(6 ,1440)
    # time.sleep(1)

    # #anggaplah robot sudah mendeteksi objek
    # #gerak maju menuju objek   
    # pm.setRcValue(5,1700)
    # time.sleep(3)

    # # mundur jika robot kelewatan dari objek
    # # pm.setRcValue(5,1400)
    # # time.sleep(1)

    # # robot menyelam sedikit lebih dalam
    # pm.setRcValue(3,1450)
    # time.sleep(1)

    # lepas gripper jika posisi sudah sesuai
    # pm.grip_open()

    pm.setMode('MANUAL')

    # misi selesai
    pm.disarm()



if __name__ == '__main__':
    pymavlink = Guidance(master)

    main(pymavlink)