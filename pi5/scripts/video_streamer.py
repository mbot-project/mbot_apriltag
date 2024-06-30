from flask import Flask, Response
from picamera2 import Picamera2
import libcamera
import cv2
import atexit
import signal

"""
PI 5 Version
This script only displays the video live stream to browser.
It is mainly for testing your setup along with test.camera.py.
This is a simple program to headlessly check if the camera work and Flask is ready to use.
visit: http://your_mbot_ip:5001
"""

class Camera:
    def __init__(self, camera_id, width, height):
        self.cap = Picamera2(camera_id)
        config = self.cap.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        self.cap.align_configuration(config)
        self.cap.configure(config)
        self.cap.start()

    def generate_frames(self):
        while True:
            frame = self.cap.capture_array()

            # Encode the frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def cleanup(self):
        if self.cap:
            print("Releasing camera resources")
            self.cap.close()
            self.cap = None  # Avoid double cleanup

app = Flask(__name__)
camera = Camera(0, 1280, 720)

def signal_handler(sig, frame):
    print('Received signal to terminate')
    camera.cleanup()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
atexit.register(camera.cleanup)

@app.route('/')
def video():
    return Response(camera.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
