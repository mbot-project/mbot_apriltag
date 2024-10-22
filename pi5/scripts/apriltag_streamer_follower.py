#!/usr/bin/env python3
from flask import Flask, Response
import cv2
import time
import numpy as np

from utils.utils import rotation_matrix_to_euler_angles, register_signal_handlers
from utils.config import CAMERA_CONFIG
from utils.camera_handler import CameraWithAprilTag

"""
Features:
1. Displays the video live stream with apriltag detection to the browser.
2. Display the pose estimate values.
3. Follow apriltag when the tag is in sight (if follow=True).

visit: http://your_mbot_ip:5001
"""

class CameraWithAprilTagFollow(CameraWithAprilTag):
    def __init__(self, camera_id, width, height, calibration_data,
                 frame_duration, follow=False):
        super().__init__(camera_id, width, height, calibration_data,
                         frame_duration)
        self.follow = follow

    def generate_frames(self):
        while self.running:
            self.frame_count += 1
            frame = self.capture_frame()

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
                            time.sleep(0.2)  # Back off for a moment
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
                    if detect['id'] < 10:  # Big tag
                        image_points = np.array(detect['lb-rb-rt-lt'], dtype=np.float32)
                        retval, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    if detect['id'] >= 10:  # Small tag at center
                        image_points = np.array(detect['lb-rb-rt-lt'], dtype=np.float32)
                        retval, rvec, tvec = cv2.solvePnP(self.small_object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    # Convert rotation vector to a rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    # Calculate Euler angles
                    roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

                    if self.follow:
                        self.publish_velocity_command(tvec[0][0], tvec[2][0], roll, pitch, yaw)

                    pos_text = f"Tag ID {detect['id']}: x={tvec[0][0]:.2f}, y={tvec[1][0]:.2f}, z={tvec[2][0]:.2f},"
                    orientation_text = f" roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
                    vertical_pos = 40 * visible_tags
                    cv2.putText(frame, pos_text + orientation_text, (10, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 0, 0), 2)
            elif self.follow:
                self.publish_velocity_command(0, 0)

            # Encode the frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def cleanup(self):
        if self.follow:
            self.publish_velocity_command(0, 0)
        if self.cap:
            print("Releasing camera resources")
            self.cap.close()
            self.cap = None  # Avoid double cleanup

app = Flask(__name__)
@app.route('/')
def video():
    return Response(camera.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    camera_id = CAMERA_CONFIG["camera_id"]
    image_width = CAMERA_CONFIG["image_width"]
    image_height = CAMERA_CONFIG["image_height"]
    fps = CAMERA_CONFIG["fps"]

    frame_duration = int((1./fps) * 1e6)
    calibration_data = np.load('cam_calibration_data.npz')
    follow_mode = True  # Set this to False if following the tag is not needed

    camera = CameraWithAprilTagFollow(camera_id, image_width, image_height,
                                      calibration_data, frame_duration, follow=follow_mode)
    register_signal_handlers(camera.cleanup)
    app.run(host='0.0.0.0', port=5001)
