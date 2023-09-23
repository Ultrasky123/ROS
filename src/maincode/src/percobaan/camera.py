import cv2
import numpy as np

# Fungsi callback untuk mengubah nilai batas bawah
def set_lower(value):
    lower[0] = value

# Fungsi callback untuk mengubah nilai batas atas
def set_upper(value):
    upper[0] = value

# Inisialisasi kamera
cap = cv2.VideoCapture(0)

# Membuat jendela tampilan
cv2.namedWindow('ROV')

# Inisialisasi trackbar untuk batas bawah
lower = np.array([20, 100, 100])
cv2.createTrackbar('Lower Hue', 'ROV', lower[0], 179, set_lower)

# Inisialisasi trackbar untuk batas atas
upper = np.array([30, 255, 255])
cv2.createTrackbar('Upper Hue', 'ROV', upper[0], 179, set_upper)

# Inisialisasi model Background Subtraction dengan metode GMM
bs = cv2.createBackgroundSubtractorMOG2()


# Fungsi untuk mengendalikan ROV berdasarkan posisi kotak terdeteksi
def control_rov(target, frame_width):
    if target is not None:
        target_x = target[0]
        if target_x < frame_width/2 - 50:
            # Pindah ke kiri
            print("Move left")
        elif target_x > frame_width/2 + 50:
            # Pindah ke kanan
            print("Move right")
        else:
            # Berhenti
            print("Stop")
    else:
        # Tidak ada target terdeteksi
        print("No target")


# Inisialisasi kamera
# cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    
    # Membaca frame dari kamera
    ret, frame = cap.read()

    # if not ret:
    # # Menghentikan program jika tidak ada frame yang berhasil dibaca
    #     break
    
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
    # Mengendalikan ROV berdasarkan posisi kotak terdeteksi
    # control_rov(target, frame.shape[1])

    # Menampilkan frame dengan kotak terdeteksi
    cv2.imshow('ROV', frame)
    

    # Menghentikan program jika tombol 'q' ditekan
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Menutup kamera dan menghentikan tampilan
cap.release()
cv2.destroyAllWindows()
