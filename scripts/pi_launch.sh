#!/usr/bin/env bash
# Launch the full real-hardware graph on the Pi (phone camera + laptop webcam).
#   bash scripts/pi_launch.sh <phone-ip> <laptop-ip>
# On the laptop first:  python scripts/stream_webcam.py
# Needs NVIDIA_API_KEY exported for voice (the phone's IP Webcam also serves the mic).
set -euo pipefail
PHONE_IP=${1:?usage: pi_launch.sh <phone-ip> <laptop-ip>}
LAPTOP_IP=${2:?usage: pi_launch.sh <phone-ip> <laptop-ip>}
cd "$(dirname "$0")/.."

set +u
. /opt/ros/jazzy/setup.bash
. "$HOME/robot-venv/bin/activate"
. install/setup.bash
set -u
# ROS node scripts use the system python; expose the venv packages to them.
export PYTHONPATH="$VIRTUAL_ENV/lib/python3.12/site-packages:${PYTHONPATH:-}"

exec ros2 launch assistant_bringup system.launch.py \
  mock_mode:=false \
  workspace_camera_url:="http://${PHONE_IP}:8080/video" \
  voice_stream_url:="http://${PHONE_IP}:8080/audio.wav" \
  gaze_camera_device:="http://${LAPTOP_IP}:8081/video" \
  gaze_camera_backend:=ffmpeg
