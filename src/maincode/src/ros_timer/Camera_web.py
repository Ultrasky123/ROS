from flask import Flask, Response
import cv2
from time import sleep

app = Flask(__name__)

def generate_frames():
    cap = cv2.VideoCapture(1)  # 1 is the index of the external camera
    while True:
        success, frame = cap.read()  # Read the frame
        sleep(0.1)
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # frame = cv2.resize(frame, (640, 480))  # Resize the frame to 640x480
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # Concatenate frame one by one and show result

@app.route('/video')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port='5000')  # Run the Flask server