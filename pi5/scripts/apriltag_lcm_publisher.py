import cv2
import time
import numpy as np
from apriltag import apriltag
import math
import lcm
from mbot_lcm_msgs.mbot_apriltag_array_t import mbot_apriltag_array_t
from mbot_lcm_msgs.mbot_apriltag_t import mbot_apriltag_t
from picamera2 import Picamera2
import libcamera

from utils.utils import rotation_matrix_to_quaternion, rotation_matrix_to_euler_angles, register_signal_handlers
from utils.config import CAMERA_CONFIG
from utils.camera_handler import CameraWithAprilTag
"""
This script publish apriltag lcm message to MBOT_APRILTAG_ARRAY
"""

class AprilTagPublisher(CameraWithAprilTag):
    def __init__(self, camera_id, width, height, calibration_data, frame_duration=None):
        super().__init__(camera_id, width, height, calibration_data, frame_duration)

    def detect(self):
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
                            time.sleep(0.1)  # Optional: back off for a moment
                        else:
                            raise  # Re-raise the last exception if retries exhausted

                self.publish_apriltag()

    def publish_apriltag(self):
        """
        Publish the apriltag message
        """
        msg = mbot_apriltag_array_t()
        msg.array_size = len(self.detections)
        msg.detections = []
        if msg.array_size > 0:
            for detect in self.detections:
                # Pose estimation for detected tag
                image_points = np.array(detect['lb-rb-rt-lt'], dtype=np.float32)
                if detect['id'] < 10: # big tag
                    retval, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if detect['id'] >= 10: # small tag at center
                    retval, rvec, tvec = cv2.solvePnP(self.small_object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                # Convert rotation vector  to a rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # # Calculate Euler angles: roll, pitch, yaw - x, y, z in degrees
                # for apriltag, x is horizontal, y is vertical, z is outward
                roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)
                quaternion = rotation_matrix_to_quaternion(rotation_matrix)

                apriltag = mbot_apriltag_t()
                apriltag.tag_id = detect['id']
                apriltag.pose.x = tvec[0][0]
                apriltag.pose.y = tvec[1][0]
                apriltag.pose.z = tvec[2][0]
                apriltag.pose.angles_rpy = [roll, pitch, yaw]
                apriltag.pose.angles_quat = quaternion
                msg.detections.append(apriltag)

        self.lcm.publish("MBOT_APRILTAG_ARRAY", msg.encode())

if __name__ == '__main__':
    camera_id = CAMERA_CONFIG["camera_id"]
    image_width = CAMERA_CONFIG["image_width"]
    image_height = CAMERA_CONFIG["image_height"]
    fps = CAMERA_CONFIG["fps"]

    calibration_data = np.load('cam_calibration_data.npz')
    frame_duration = int((1./fps)*1e6)
    camera = AprilTagPublisher(camera_id, image_width, image_height, calibration_data, frame_duration)
    register_signal_handlers(camera.cleanup)
    camera.detect()


