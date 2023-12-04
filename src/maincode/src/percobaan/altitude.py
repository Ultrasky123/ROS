
# Import mavutil
from pymavlink import mavutil

def get_altitude():
    #jika koneksi langsung komputer/nuc
    # master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

    #koneksi jika pakai companion
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    while True:
        
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'AHRS2':
            print("depth: %s" % msg.altitude)
            return(msg.altitude)
            rate = 10

if __name__ == '__main__':
    get_altitude()