## Jetson nano mbot_apriltag usage
This project provides tools for camera calibration and Apriltag pose estimation, tailored for use with the MBot equipped with a **Jetson Nano**.

The scripts offer:
- Streaming live camera feeds to a browser for real-time viewing and interaction.
- Publishing and subscribing to Apriltag LCM messages.
- MBot following Apriltags.

## Installation
### Install Apriltag Library
1. Directly install from the official [github repo](https://github.com/AprilRobotics/apriltag):
    ```bash
    git clone https://github.com/AprilRobotics/apriltag.git
    ```

2. Following the install [instruction](https://github.com/AprilRobotics/apriltag) there:
    ```bash
    cd apriltag
    cmake -B build -DCMAKE_BUILD_TYPE=Release
    sudo cmake --build build --target install
    ```

### Clone this repo
```bash
git clone https://github.com/MBot-Project-Development/mbot_apriltag.git
```

### Install dependencies
```bash
pip install Flask
```

## Usage and Features
### Run scripts
> Visit `http://your_mbot_ip:5001` to see the stream video
- `python3 test_camera.py`
    - To test if your camera is functional and if the ID is correct.
- `python3 video_streamer.py`
    - Only shows the video stream, to test your camera and your dependency installation
- `python3 save_image.py`
    - Shows the video stream, we use it to save image to `/images` for camera calibration
- `python3 camera_calibration.py`
    - Use the images from `/images` and output calibration result as `cam_calibration_data.npz`. The result will be used directly by apriltag_streamer.py you don't have to modify anything.
- `python3 apriltag_streamer.py`
    - It runs apriltag detection, when tag is detected, pose estimation will be printed on the screen.
- `python3 apriltag_lcm_publisher.py`
    - Publish apriltag lcm message over `MBOT_APRILTAG_ARRAY`
- `python3 apriltag_lcm_subscriber.py`
    - Listen to `MBOT_APRILTAG_ARRAY` for apriltag lcm message
- `python3 apriltag_follower.py`
    - Allow mbot to follow the apriltag in sight
- `python3 apriltag_follower_streamer.py`
    - Allow mbot to follow the apriltag in sight, while forwaring video stream to browser

## Troubleshooting
### Camera Troubleshooting
> This section is to troubleshoot when the video_streamer.py doesn't work. Please follow the steps in order to diagnose the cause.

1. Verify the camera is connected properly and is recognized by your system. Run the following commands in your vscode terminal:
    ```bash
    ls /dev/video*
    ```
    where you should see something like this:
    ```bash
    mbot@mbot-shaw-test02:~$ ls /dev/video*
    /dev/video0
    ```

2. Verify if [nvarguscamerasrc](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/SD/CameraDevelopment/CameraSoftwareDevelopmentSolution.html) plugin is properly installed and working. Run this in your vscode terminal:
   ```bash
   gst-inspect-1.0 nvarguscamerasrc
   ```
   - Look for 'Factory Details' and 'Plugin Details' in the terminal prints to confirm its presence. If you see errors like "Command not found" or "No such element or plugin," the plugin may not be correctly installed. Note that GStreamer should already be installed on Ubuntu20. 
    - press `q` to exit

3. Verify if the `nvarguscamerasrc` is working properly by recording a video using it. Run this in your vscode terminal:
   ```bash
   gst-launch-1.0 nvarguscamerasrc num-buffers=200 ! 'video/x-raw(memory:NVMM), width=1280, height=720, framerate=20/1, format=NV12' ! omxh264enc ! qtmux ! filesink location=test.mp4
   ```
   - It tells GStreamer to capture 200 frames and then stop, and save the video `test.mp4` to the current directory.
   - To watch the video, you can directly open the `test.mp4` on vscode.
   - Up to this step, you have confirmed that both your hardware and GStreamer-related components are functioning correctly and are not the source of the issue.

4. Verify if your OpenCV installation was compiled with GStreamer support by run the following code in your vscode terminal:
```bash
# Launch Python:
$ python3
---
>>> import cv2
>>> print(cv2.getBuildInformation())
```
This command displays OpenCV's build details. In the output, locate the "Video I/O" section to confirm GStreamer support:
```bash
  Video I/O:
    DC1394:                      YES (2.2.6)
    FFMPEG:                      YES
      avcodec:                   YES (58.54.100)
      avformat:                  YES (58.29.100)
      avutil:                    YES (56.31.100)
      swscale:                   YES (5.5.100)
      avresample:                YES (4.0.0)
    GStreamer:                   YES (1.16.3)
    v4l/v4l2:                    YES (linux/videodev2.h)
```
If "GStreamer" is marked as "NO," it means GStreamer isn't enabled in your OpenCV installation. This indicates that the issue is from OpenCV.

### libapriltag.so.3
If you encounter the following runtime error: "ImportError: libapriltag.so.3: cannot open shared object file: No such file or directory," follow these steps:

1. Verify the Installation Location

    The library is typically installed in /usr/local/lib or /usr/lib. Check these locations:
    ```bash
    $ ls /usr/local/lib | grep libapriltag
    ```
    ```bash
    $ ls /usr/lib | grep libapriltag
    ```
    - If you see "libapriltag.so.3" in the output, proceed to the next step.

2. Update the Library Cache

    If the library is in a standard location but still not found, you may need to update your system's library cache. Run the following command to refresh it:

    ```bash
    $ sudo ldconfig
    ```
    - After updating, try running video_streamer.py again to check if the issue is resolved.