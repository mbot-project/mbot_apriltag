#!/usr/bin/env python3
from apriltag import apriltag
from flask import Flask, Response
import cv2
import time
import atexit
import numpy as np
from picamera2 import Picamera2 
import libcamera

"""
This script displays the video live stream with apriltag detection to browser.
The pose estimation will display as well.
visit: http://your_mbot_ip:5001
"""

class Camera:
    def __init__(self, camera_id, width, height, frame_duration):
        self.cap = Picamera2(camera_id)
        self.w = width
        self.h = height
        config = self.cap.create_preview_configuration(
            main={"size": (self.w, self.h), "format": "RGB888"},
            controls = {
                'FrameDurationLimits': (frame_duration, frame_duration) # (min, max) microseconds
            }
        )
        config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        self.cap.align_configuration(config)
        self.cap.configure(config)
        self.cap.start()

        self.detector = apriltag("tagCustom48h12", threads=1)
        self.skip_frames = 5  # Process every 5th frame for tag detection
        self.frame_count = 0
        self.detections = dict()
        calibration_data = np.load('cam_calibration_data.npz')
        self.camera_matrix = calibration_data['camera_matrix']
        self.dist_coeffs = calibration_data['dist_coeffs']
        self.tag_size = 54              # in millimeter
        self.small_tag_size = 10.8      # in millimeter
        self.object_points = np.array([
            [-self.tag_size/2,  self.tag_size/2, 0],  # Top-left corner
            [ self.tag_size/2,  self.tag_size/2, 0], # Top-right corner
            [ self.tag_size/2, -self.tag_size/2, 0], # Bottom-right corner
            [-self.tag_size/2, -self.tag_size/2, 0], # Bottom-left corner
        ], dtype=np.float32)
        self.small_object_points = np.array([
            [-self.small_tag_size/2,  self.small_tag_size/2, 0],  # Top-left corner
            [ self.small_tag_size/2,  self.small_tag_size/2, 0], # Top-right corner
            [ self.small_tag_size/2, -self.small_tag_size/2, 0], # Bottom-right corner
            [-self.small_tag_size/2, -self.small_tag_size/2, 0], # Bottom-left corner
        ], dtype=np.float32)

    def generate_frames(self):
        while True:
            self.frame_count += 1
            frame = self.cap.capture_array()

            # Process for tag detection only every 5th frame
            if self.frame_count % self.skip_frames == 0:
                # Convert frame to grayscale for detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Retry logic
                max_retries = 3
                for attempt in range(max_retries):
                    try:
                        self.detections = self.detector.detect(gray)
                        break  # Success, exit the retry loop
                    except RuntimeError as e:
                        if "Unable to create" in str(e) and attempt < max_retries - 1:
                            print(f"Detection failed due to thread creation issue, retrying... Attempt {attempt + 1}")
                            time.sleep(0.2)  # back off for a moment
                        else:
                            raise  # Re-raise the last exception if retries exhausted

            if self.detections:
                visible_tags = 0
                for detect in self.detections:
                    visible_tags += 1

                    # Draw the corners of the tag
                    corners = np.array(detect['lb-rb-rt-lt'], dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                    # Pose estimation for detected tag
                    if detect['id'] < 10: # big tag
                        image_points = np.array(detect['lb-rb-rt-lt'], dtype=np.float32)
                        retval, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    if detect['id'] >= 10: # small tag at center
                        image_points = np.array(detect['lb-rb-rt-lt'], dtype=np.float32)
                        retval, rvec, tvec = cv2.solvePnP(self.small_object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    # Convert rotation vector to a rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    # Calculate Euler angles 
                    roll, pitch, yaw = calculate_euler_angles_from_rotation_matrix(rotation_matrix)

                    pos_text = f"Tag ID {detect['id']}: x={tvec[0][0]:.2f}, y={tvec[1][0]:.2f}, z={tvec[2][0]:.2f},"
                    orientation_text = f" roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
                    vertical_pos = 40*visible_tags
                    cv2.putText(frame, pos_text+orientation_text, (10, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 0, 0), 2)

            # Encode the frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def cleanup(self):
        print("Releasing camera resources")
        if self.cap:
            self.cap.stop()

def calculate_euler_angles_from_rotation_matrix(R):
    """
    Calculate Euler angles (roll, pitch, yaw) from a rotation matrix.
    Assumes the rotation matrix uses the XYZ convention.
    """
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)  # Convert to degrees

app = Flask(__name__)
@app.route('/')
def video():
    return Response(camera.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # image width and height here should align with save_image.py
    camera_id = 0
    image_width = 1280
    image_height = 720
    fps = 10
    frame_duration = int((1./fps)*1e6)
    camera = Camera(camera_id, image_width, image_height, frame_duration) 
    atexit.register(camera.cleanup)
    app.run(host='0.0.0.0', port=5001)

