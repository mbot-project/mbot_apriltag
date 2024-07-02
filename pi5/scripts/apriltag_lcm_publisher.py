import cv2
import time
import atexit
import numpy as np
from apriltag import apriltag
import math
import lcm
from mbot_lcm_msgs.mbot_apriltag_array_t import mbot_apriltag_array_t
from mbot_lcm_msgs.mbot_apriltag_t import mbot_apriltag_t
from picamera2 import Picamera2 
import libcamera
import signal

"""
This script publish apriltag lcm message to MBOT_APRILTAG_ARRAY
"""

class Camera:
    def __init__(self, camera_id, width, height, frame_duration):
        self.cap = Picamera2(camera_id)
        config = self.cap.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
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
        self.lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
        print("Initializing Publisher...")

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

    def cleanup(self):
        if self.cap:
            print("Releasing camera resources")
            self.cap.close()
            self.cap = None  # Avoid double cleanup

def rotation_matrix_to_euler_angles(R):
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

def rotation_matrix_to_quaternion(R):
    """
    Convert a rotation matrix to a quaternion.
    
    Args:
        R (numpy.ndarray): The rotation matrix.
        
    Returns:
        numpy.ndarray: The quaternion [qx, qy, qz, qw].
    """
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = R.flat
    qw = np.sqrt(max(0, 1 + m00 + m11 + m22)) / 2
    qx = np.sqrt(max(0, 1 + m00 - m11 - m22)) / 2
    qy = np.sqrt(max(0, 1 - m00 + m11 - m22)) / 2
    qz = np.sqrt(max(0, 1 - m00 - m11 + m22)) / 2
    qx = np.copysign(qx, m21 - m12)
    qy = np.copysign(qy, m02 - m20)
    qz = np.copysign(qz, m10 - m01)
    return np.array([qx, qy, qz, qw])

def signal_handler(sig, frame):
    print('Received signal to terminate')
    camera.cleanup()
    exit(0)

if __name__ == '__main__':
    # image width and height here should align with save_image.py
    camera_id = 0
    image_width = 1280
    image_height = 720
    fps = 10
    frame_duration = int((1./fps)*1e6)
    camera = Camera(camera_id, image_width, image_height, frame_duration) 

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(camera.cleanup)
    
    camera.detect()


