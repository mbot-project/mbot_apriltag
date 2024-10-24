from picamera2 import Picamera2
import libcamera
import logging
import cv2, math, time
from apriltag import apriltag
import numpy as np
import lcm
from mbot_lcm_msgs.twist2D_t import twist2D_t
from utils.config import TAG_CONFIG
from utils.utils import rotation_matrix_to_euler_angles

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
        self.skip_frames = TAG_CONFIG["skip_frames"]
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

    def generate_frames(self):
        while self.running:
            self.frame_count += 1
            frame = self.capture_frame()

            # Process for tag detection only every Nth frame
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
                for idx, detection in enumerate(self.detections):
                    x, y, z, roll, pitch, yaw = self.decode_detection(detection)
                    # Draw the corners of the tag
                    corners = np.array(detection['lb-rb-rt-lt'], dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                    pos_text = f"Tag ID {detection['id']}: x={x:.2f}, y={y:.2f}, z={z:.2f},"
                    orientation_text = f" roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
                    vertical_pos = 40 * (idx+1)
                    text = pos_text + orientation_text

                    # Calculate the size of the text box
                    (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    box_coords = ((10, vertical_pos - text_height - 10), (10 + text_width, vertical_pos + baseline))

                    # Draw the background rectangle
                    cv2.rectangle(frame, box_coords[0], box_coords[1], (200, 200, 200), cv2.FILLED)

                    # Put the text on top of the rectangle
                    cv2.putText(frame, text, (10, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            # Encode the frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def decode_detection(self, detect):
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

        # return x, y, z, roll, pitch, yaw
        return tvec[0][0], tvec[1][0], tvec[2][0], roll, pitch, yaw

class CameraWithAprilTagFollow(CameraWithAprilTag):
    def __init__(self, camera_id, width, height, calibration_data, frame_duration=None):
        super().__init__(camera_id, width, height, calibration_data, frame_duration)

    def generate_frames(self):
        while self.running:
            self.frame_count += 1
            frame = self.capture_frame()

            # Process for tag detection only every Nth frame
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
                for idx, detection in enumerate(self.detections):
                    x, y, z, roll, pitch, yaw = self.decode_detection(detection)
                    self.follow_tag(x, z, roll, pitch, yaw)

                    # Draw the corners of the tag
                    corners = np.array(detection['lb-rb-rt-lt'], dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                    pos_text = f"Tag ID {detection['id']}: x={x:.2f}, y={y:.2f}, z={z:.2f},"
                    orientation_text = f" roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
                    vertical_pos = 40 * (idx+1)
                    text = pos_text + orientation_text

                    # Calculate the size of the text box
                    (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    box_coords = ((10, vertical_pos - text_height - 10), (10 + text_width, vertical_pos + baseline))

                    # Draw the background rectangle
                    cv2.rectangle(frame, box_coords[0], box_coords[1], (200, 200, 200), cv2.FILLED)

                    # Put the text on top of the rectangle
                    cv2.putText(frame, text, (10, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            else:
                self.publish_velocity_command(0, 0)  # Stop if no tags are detected

            # Encode the frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def follow_tag(self, x, z, roll=0, pitch=0, yaw=0):
        if self.detections:
            self.publish_velocity_command(x, z, roll, pitch, yaw)
        else:
            self.publish_velocity_command(0, 0)  # Stop if no tags are detected

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