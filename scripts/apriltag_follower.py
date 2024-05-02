import cv2
import time
import atexit
import numpy as np
from apriltag import apriltag
import math
import lcm
from mbot_lcm_msgs.twist2D_t import twist2D_t

"""
This script allow mbot to follow apriltag
"""

class Camera:
    def __init__(self, camera_id, width, height, framerate):
        self.cap = cv2.VideoCapture(self.camera_pipeline(camera_id, width, height, framerate))
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

    def camera_pipeline(self, i, w, h, framerate):
        """
        Generates a GStreamer pipeline string for capturing video from an NVIDIA camera.

        Parameters:
        i (int): The sensor ID of the camera.
        w (int): The width of the video frame in pixels.
        h (int): The height of the video frame in pixels.
        framerate (int): The framerate of the video in frames per second.
        """
        return f"nvarguscamerasrc sensor_id={i} ! \
        video/x-raw(memory:NVMM), \
        width=1280, height=720, \
        format=(string)NV12, \
        framerate={framerate}/1 ! \
        nvvidconv \
        flip-method=0 ! \
        video/x-raw, \
        format=(string)BGRx, \
        width={w}, height={h} !\
        videoconvert ! \
        video/x-raw, \
        format=(string)BGR ! \
        appsink"

    def detect(self):
        while True:
            success, frame = self.cap.read()
            if not success:
                break
            self.frame_count += 1
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
            
                self.follow_apriltag()

    def follow_apriltag(self):
        if self.detections:
            print("Tag in sight")
            for detect in self.detections:
                # Pose estimation for detected tag
                image_points = np.array(detect['lb-rb-rt-lt'], dtype=np.float32)
                if detect['id'] < 10: # big tag
                    retval, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if detect['id'] >= 10: # small tag at center
                    retval, rvec, tvec = cv2.solvePnP(self.small_object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                self.publish_velocity_command(tvec[0][0], tvec[2][0])
        else:
            self.publish_velocity_command(0, 0)
            print("No tag in sight")

    def publish_velocity_command(self, x, z):
        """
        Publish a velocity command based on the x and z offset of the detected tag.
        """
        # Constants
        k_p = 0.002  # Proportional gain for linear velocity
        k_theta = 1.8 # Proportional gain for angular velocity
        z_target = 200  # Target distance (millimeters)

        theta = math.atan2(x, z)  # Angle to target
        wz = -k_theta * theta  # Angular velocity

        # Modified linear velocity calculation with stop condition
        if z - z_target > 0:
            vx = k_p * (z - z_target)  # Move forward if target is ahead
        else:
            vx = 0  # Stop 

        # Create the velocity command message
        command = twist2D_t()
        command.vx = vx
        command.wz = wz
        
        # Publish the velocity command
        self.lcm.publish("MBOT_VEL_CMD", command.encode())

    def cleanup(self):
        print("Releasing camera resources")
        self.publish_velocity_command(0, 0)
        if self.cap and self.cap.isOpened():
            self.cap.release()

if __name__ == '__main__':
    # image width and height here should align with save_image.py
    camera_id = 0
    image_width = 1280
    image_height = 720
    frame_rate = 10
    camera = Camera(camera_id, image_width, image_height, frame_rate) 
    atexit.register(camera.cleanup)
    camera.detect()


