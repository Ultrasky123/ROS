
# Import mavutil
from pymavlink import mavutil

def get_heading():
    #jika koneksi langsung komputer/nuc
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

    #koneksi jika pakai companion
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    while True:
        
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            print("heading: %s" % msg.heading)
            return(msg.heading)
            rate = 10

if __name__ == '__main__':
    get_heading()