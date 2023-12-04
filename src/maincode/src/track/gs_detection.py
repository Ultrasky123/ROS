#!/usr/bin/env python3

# from controlGuidance import control_rov
import cv2
import gi
import numpy as np

import os
os.environ['MAVLINK20'] = ''
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time  

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Video():
    

    def __init__(self, port=5600):
     
        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        

        if not config:
            config = \
                [
                    
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
    
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):

        return self._new_frame is not None

    def run(self):

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK


if __name__ == '__main__':


    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    #jika koneksi langsung komputer
    # master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    boot_time = time.time()

    def setRcValue(channel_id, pwm=1500):

            if channel_id < 1 or channel_id > 18:
                print("Channel does not exist.")
                return

            # Mavlink 2 supports up to 18 channels:
            # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
            rc_channel_values = [65535 for _ in range(18)]
            rc_channel_values[channel_id - 1] = pwm
            master.mav.rc_channels_override_send(
                master.target_system,                # target_system
                master.target_component,             # target_component
                *rc_channel_values)                  # RC channel list, in microseconds.


    def control_rov(rect, frame_width, frame_height):
        centering_zone_width = 320
        centering_zone_height = 240
        centering_zone_x = frame_width / 2 - centering_zone_width / 2
        centering_zone_y = frame_height / 2 - centering_zone_height / 2
        
        center_zone_width = 300
        center_zone_height = 450
        center_zone_x = frame_width / 2 - centering_zone_width / 2
        center_zone_y = frame_height / 2 - centering_zone_height / 2

        target_x, target_y, target_w, target_h = rect
        # jika box tepat di tengah jalan
        if (target_x >= centering_zone_x and target_x + target_w <= centering_zone_x + centering_zone_width and
            target_y >= centering_zone_y and target_y + target_h <= centering_zone_y + centering_zone_height):
            # Stop moving
            setRcValue(5,1600)
            time.sleep(1)
            print("go maju")
        
        #jika box sudah dekat 
        elif(target_x >= center_zone_x and target_x + target_w <= center_zone_x + center_zone_width and
            target_y >= center_zone_y and target_y + target_h <= center_zone_y + center_zone_height):
            setRcValue(5,1500)
            time.sleep(1)
            print("stop")
        else:
            if target_x < frame_width / 2-10:
                setRcValue(6,1400)
                time.sleep(1)
                print("Move kiri")
            elif target_x > frame_width / 2+10 :
                setRcValue(6,1600)
                time.sleep(1)
                print("Move Kanan")
            # elif target_y > 


    # Load color threshold values
    lower = np.load('/home/lz/tracking/src/data1.npy')
    upper = np.load('/home/lz/tracking/src/data2.npy')

    # Background Subtraction model initialization
    bs = cv2.createBackgroundSubtractorMOG2()

    # Create the video object
    # Add port= if is necessary to use a different one
    video = Video()
    
    print('Initialising stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting streaming - press "q" to quit.')

    while True:
        # Wait for the next frame to become available
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = video.frame()
            # Apply background subtraction to the frame
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            max_area = 0
            max_contour = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    max_contour = contour

            if max_contour is not None:
                # Calculate the rectangle enclosing the target
                x, y, w, h = cv2.boundingRect(max_contour)

                # Draw the rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Control ROV based on the target rectangle
                control_rov((x, y, w, h), frame.shape[1], frame.shape[0])
            
            # Display the frame with the detected rectangle
            cv2.imshow('ROV', frame)

        # Allow frame to display, and check if user wants to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
