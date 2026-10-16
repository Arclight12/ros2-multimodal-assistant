# Setup

Install ROS 2 Jazzy on Ubuntu 24.04, source it, and install workspace dependencies:

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # once per machine; ignore the error if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

The phone must expose an OpenCV-readable HTTP/MJPEG stream. Put that URL in
`config/cameras.yaml`; put the laptop device index there too. Model paths are
configured in `perception.yaml` and `gaze.yaml`.

## Connect real cameras

The workspace camera is a phone; the gaze camera is the laptop webcam. Both
are configured in `src/assistant_bringup/config/cameras.yaml` (or launch
overrides) and should be verified before starting ROS.

### Phone (workspace camera)

1. Install an IP camera app on the phone and start its server:
   - **IP Webcam** (Android): `http://<phone-ip>:8080/video`
   - **DroidCam** (Android/iOS): `http://<phone-ip>:4747/video`
2. Put the laptop on the same Wi-Fi network and use the phone IP the app
   displays. Use the direct stream URL, not the app's browser control page.
3. Keep the app running in the background (Android: enable the app's background
   mode, disable battery optimization for it) and disable screen timeout so the
   stream does not drop mid-session. If it does drop, the workspace camera node
   reconnects automatically with backoff.

### Laptop webcam (gaze camera)

The gaze camera defaults to device index `0`. If the wrong camera is picked,
probe the available indices and configure `gaze_camera_index` (or a device
path such as `/dev/video2` via `gaze_camera_device`). On Windows prefer
backend `dshow`, on Linux `v4l2`:

```bash
python3 scripts/check_cameras.py --probe 4
```

### Verify cameras without ROS

Run the diagnostics before launching the graph; it opens each camera with the
same helpers the nodes use, prints resolution and measured fps, and saves
snapshots to inspect:

```bash
python3 scripts/check_cameras.py \
  --workspace-url http://192.168.1.10:8080/video \
  --gaze-index 0 --gaze-backend dshow
```

Note for WSL2: the phone stream is plain HTTP and works from WSL2; direct
webcam access inside WSL2 additionally requires attaching the device with
`usbipd-win`.

### Launch with real cameras

Start real camera inference without enabling motion:

```bash
ros2 launch assistant_bringup system.launch.py mock_mode:=false \
  workspace_camera_url:=http://192.168.1.10:8080/video \
  gaze_camera_index:=0 gaze_camera_backend:=dshow
```

Verify that frames are flowing and preview them:

```bash
ros2 topic hz /workspace/image_raw
ros2 topic hz /gaze/image_raw
ros2 run rqt_image_view rqt_image_view
```

Real manipulation additionally requires a valid saved calibration, trained
weights, a configured transform, the measured arm geometry and the flashed
arm firmware (see [arm.md](arm.md)).

## Raspberry Pi

Target: Raspberry Pi 4 or 5 with **Ubuntu Server 24.04 (64-bit)**. Raspberry Pi
OS and other Debian releases are not supported: ROS 2 Jazzy has no packages for
them. On a fresh Ubuntu 24.04 install, `bash scripts/setup_pi.sh` installs ROS 2
Jazzy, the Python dependencies in a venv and serial access, then
builds the workspace; it refuses to run on any other OS. Notes:

- `requirements.txt` is the runtime set. Do not install `requirements-train.txt`
  on the Pi. Ultralytics still pulls in torch as a dependency.
- Use the Linux camera backend for the laptop webcam, or a USB webcam on the Pi:
  `gaze_camera_backend:=v4l2`.
- The phone stream is unchanged: set `workspace_camera_url` as above.
- Arduino arm controller: it appears as `/dev/ttyACM0` (or `/dev/ttyUSB0`).
  Run `sudo usermod -aG dialout $USER` and log out and back in, then set
  `arm_controller_node.serial_port` in `robot.yaml`. See [arm.md](arm.md).
- Camera rates are the main load: `camera_fps` (workspace) and `gaze_camera_fps`
  set how many frames reach YOLO and gaze. Lower them before lowering anything
  else. `mock_detection_fps` and `mock_gaze_fps` only affect mock mode.
- Detector and gaze models: exported NCNN directories are not accepted, because
  `object_detector_node` loads only a single model file. ONNX is the workable
  export. Export and speed on the Pi are untested.
