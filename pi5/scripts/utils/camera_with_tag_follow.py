from picamera2 import Picamera2
import logging
import cv2, math, time
import numpy as np
from mbot_lcm_msgs.twist2D_t import twist2D_t

from utils.camera_with_apriltag import CameraWithAprilTag

class CameraWithTagFollower(CameraWithAprilTag):
    def __init__(self, camera_id, width, height, calibration_data, frame_duration=None):
        super().__init__(camera_id, width, height, calibration_data, frame_duration)

    def process_frame(self, frame):
        # Perform the same detection logic as CameraWithAprilTag
        super().process_frame(frame)

        # If detections are found, follow the tag
        if self.detections:
            for idx, detection in enumerate(self.detections):
                x, y, z, roll, pitch, yaw, quaternion = self.decode_detection(detection)
                self.publish_velocity_command(x, z, roll, pitch, yaw)
        else:
            self.publish_velocity_command(0, 0)  # Stop if no tags are detected

        return frame

    def publish_velocity_command(self, x, z, roll=0, pitch=0, yaw=0):
        """
        Publish a velocity command based on the x and z offset of the detected tag.
        """
        # Constants
        k_p_linear = 0.001  # Proportional gain for linear velocity
        k_p_angular = 2  # Proportional gain for angular velocity
        z_target = 150  # Target distance (millimeters)

        # Calculate angular velocity
        theta = math.atan2(x, z)  # Angle to target
        wz = -k_p_angular * theta  # Angular velocity

        # Calculate linear velocity
        if z - z_target > 0:
            vx = k_p_linear * (z - z_target)  # Move forward if target is ahead
        else:
            vx = 0  # Stop

        # Adjust linear velocity based on orientation
        max_pitch_angle = 20  # Maximum pitch angle (degrees)
        if abs(pitch) > max_pitch_angle:
            vx *= 0.5  # Reduce linear velocity by half if pitch angle exceeds threshold

        # Create the velocity command message
        command = twist2D_t()
        command.vx = vx
        command.wz = wz

        # Publish the velocity command
        self.lcm.publish("MBOT_VEL_CMD", command.encode())

    def cleanup(self):
        self.publish_velocity_command(0, 0)
        self.running = False
        if self.cap:
            logging.info("Releasing camera resources")
            self.cap.close()
            self.cap = None