# mbot_apriltag

## Description
This project offers tools for camera calibration and Apriltag pose estimation, specifically tailored for use with the MBot equipped with a **Jetson Nano** and **Raspeberry Pi 5**.

The provided scripts enable users to:
- Stream live camera feeds directly to a browser, allowing for real-time viewing and interaction.
- Publish and subscribe to Apriltag LCM messages.
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

3. Add Python path
    ```bash
    $ echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.11/site-packages' >> ~/.bashrc
    ```

### Clone this repo
```bash
git clone https://github.com/mbot-project/mbot_apriltag.git
```


## Usage and Features
### Run scripts
> Visit `http://your_mbot_ip:5001` to see the stream video
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

## Authors and maintainers
- The current maintainer of this project is Shaw Sun. Please direct all questions regarding support, contributions, and issues to the maintainer.
