import cv2
import numpy as np
import time

# Inisialisasi kamera
cap = cv2.VideoCapture(2)
cap.set(3, 640)
cap.set(4, 480)


# Membuat jendela tampilan
cv2.namedWindow('ROV')

# Inisialisasi
lower = np.load('/home/lz/catkin_ws/src/silver/src/coba/data1.npy')
upper = np.load('/home/lz/catkin_ws/src/silver/src/coba/data2.npy')


# Inisialisasi model Background Subtraction dengan metode GMM
bs = cv2.createBackgroundSubtractorMOG2()


# Fungsi untuk mengendalikan ROV berdasarkan posisi kotak terdeteksi
def control_rov(rect, frame_width, frame_height):
    centering_zone_width = 600
    centering_zone_height = 400
    centering_zone_x = frame_width / 2 - centering_zone_width / 2
    centering_zone_y = frame_height / 2 - centering_zone_height / 2
    
    target_x, target_y, target_w, target_h = rect
    # else:
    #     # setRcValue(5,1600)
    #     print("go")

    # Check if the rectangle exceeds the centering values
    if (target_x >= centering_zone_x and target_x + target_w <= centering_zone_x + centering_zone_width and
        target_y >= centering_zone_y and target_y + target_h <= centering_zone_y + centering_zone_height):
        # Stop moving
        print("Stop")
    else:
        if target_x < frame_width / 2-10:
            # setRcValue(6,1400)
            print("Move left")
        elif target_x > frame_width / 2+10 :
            # setRcValue(6,1600)
            print("Move right")
        # elif target_y > 

while True:
    
    # Membaca frame dari kamera
    ret, frame = cap.read()

    if not ret:
    # Menghentikan program jika tidak ada frame yang berhasil dibaca
        break
    
    # Mengaplikasikan Background5 Subtraction pada frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    # Menghilangkan noise dengan operasi morfologi
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Mendeteksi kotak
    # target = detect_box(frame)
     # Menggunakan metode deteksi kontur OpenCV untuk menemukan kotak
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Memilih kotak terbesar (berdasarkan luas) sebagai target
    max_area = 0
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None:
        # Menghitung pusat kotak yang terdeteksi
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            center = (center_x, center_y)
            # Menggambar lingkaran di pusat kotak yang terdeteksi
            cv2.circle(frame, center, 10, (0, 255, 0), -1)
            print(center)
            # return center
    # print(None)

    # Mengendalikan ROV berdasarkan posisi kotak terdeteksi
    # control_rov(center, frame.shape[1], frame.shape[0])

    # Menampilkan frame dengan kotak terdeteksi
    cv2.imshow('ROV', frame)
    

    # Menghentikan program jika tombol 'q' ditekan
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Menutup kamera dan menghentikan tampilan
cap.release()
cv2.destroyAllWindows()
