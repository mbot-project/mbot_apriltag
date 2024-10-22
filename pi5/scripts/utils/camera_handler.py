from picamera2 import Picamera2
import libcamera
import logging
import cv2, math
from apriltag import apriltag
import numpy as np
import lcm
from mbot_lcm_msgs.twist2D_t import twist2D_t
from utils.config import TAG_CONFIG

logging.basicConfig(
    level=logging.INFO,  # Set to DEBUG for more verbosity during development
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class Camera:
    def __init__(self, camera_id, width, height, frame_duration=None):
        """
        Initializes the camera with the given parameters.
        :param camera_id: The ID of the camera to use.
        :param width: The width of the camera frame.
        :param height: The height of the camera frame.
        :param frame_duration: Optional frame duration limit for the camera.
        """
        logging.info("Initializing camera...")
        self.cap = Picamera2(camera_id)
        config = self.cap.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        if frame_duration:
            config["controls"] = {'FrameDurationLimits': (frame_duration, frame_duration)}
        config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        self.cap.align_configuration(config)
        self.cap.configure(config)
        self.cap.start()
        self.running = True
        logging.info("Camera initialized.")

    def capture_frame(self):
        """
        Captures a single frame from the camera.
        :return: The captured frame as a numpy array.
        """
        return self.cap.capture_array()

    def generate_frames(self):
        """
        Generates frames for streaming purposes.
        :return: A generator yielding encoded frames.
        """
        while self.running:
            frame = self.capture_frame()
            # Encode the frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def cleanup(self):
        """
        Cleans up the camera resources.
        """
        self.running = False
        if self.cap:
            logging.info("Releasing camera resources")
            self.cap.close()
            self.cap = None

class CameraWithAprilTag(Camera):
    def __init__(self, camera_id, width, height, calibration_data, frame_duration=None):
        super().__init__(camera_id, width, height, frame_duration)
        self.detector = apriltag(TAG_CONFIG["tag_family"], threads=1)
        self.skip_frames = 5  # Process every 5th frame for tag detection
        self.frame_count = 0
        self.detections = dict()
        # calibration_data = np.load('cam_calibration_data.npz')
        self.camera_matrix = calibration_data['camera_matrix']
        self.dist_coeffs = calibration_data['dist_coeffs']
        self.tag_size = TAG_CONFIG["tag_size"]
        self.small_tag_size = TAG_CONFIG["small_tag_size"]
        self.object_points = np.array([
            [-self.tag_size/2,  self.tag_size/2, 0],  # Top-left corner
            [ self.tag_size/2,  self.tag_size/2, 0],  # Top-right corner
            [ self.tag_size/2, -self.tag_size/2, 0],  # Bottom-right corner
            [-self.tag_size/2, -self.tag_size/2, 0],  # Bottom-left corner
        ], dtype=np.float32)
        self.small_object_points = np.array([
            [-self.small_tag_size/2,  self.small_tag_size/2, 0],  # Top-left corner
            [ self.small_tag_size/2,  self.small_tag_size/2, 0],  # Top-right corner
            [ self.small_tag_size/2, -self.small_tag_size/2, 0],  # Bottom-right corner
            [-self.small_tag_size/2, -self.small_tag_size/2, 0],  # Bottom-left corner
        ], dtype=np.float32)
        self.lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")

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