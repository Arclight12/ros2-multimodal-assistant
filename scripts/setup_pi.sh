#!/usr/bin/env bash
# One-shot Raspberry Pi setup: ROS 2 Jazzy, Python deps, serial
# access, workspace build. Run from anywhere on Ubuntu 24.04; safe to re-run.
#
#   bash scripts/setup_pi.sh
set -euo pipefail
cd "$(dirname "$0")/.."

. /etc/os-release
if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_CODENAME:-}" != "noble" ]; then
  echo "ROS 2 Jazzy only has packages for Ubuntu 24.04 (noble); this is ${PRETTY_NAME}." >&2
  echo "Reflash the SD card with Ubuntu Server 24.04 64-bit (see docs/setup.md)." >&2
  exit 1
fi

VENV="$HOME/robot-venv"

# ROS 2 apt source. curl -f so a failed download is an error, not a junk file.
sudo apt-get update
sudo apt-get install -y software-properties-common curl unzip
sudo add-apt-repository -y universe
# Without noble-updates, -dev packages come in older than their installed libs.
grep -rqs noble-updates /etc/apt/sources.list /etc/apt/sources.list.d/ ||
  echo "deb http://ports.ubuntu.com/ubuntu-ports noble-updates main restricted universe multiverse" \
    | sudo tee /etc/apt/sources.list.d/noble-updates.list >/dev/null
sudo apt-get update
sudo apt-get -y full-upgrade
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-cv-bridge \
  ros-jazzy-launch-ros ros-jazzy-ament-index-python \
  python3-colcon-common-extensions python3-venv python3-pip \
  python3-serial python3-yaml python3-numpy python3-opencv

# Arduino serial access (takes effect after the next login).
sudo usermod -aG dialout "$USER"

# --system-site-packages keeps rclpy and cv_bridge visible inside the venv.
[ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install -r requirements.txt

# Fail here, not at launch, if pip OpenCV and apt cv_bridge do not mix.
set +u
. /opt/ros/jazzy/setup.bash
. "$VENV/bin/activate"
set -u
python - <<'EOF'
import cv2
import mediapipe
import serial
import ultralytics
from cv_bridge import CvBridge

assert hasattr(cv2, "aruco"), "this cv2 has no aruco module"
print("python imports ok, cv2", cv2.__version__)
EOF

# Built inside the venv so the node entry points use its interpreter.
colcon build --symlink-install

cat <<EOF

Setup done. Log out and back in once so the dialout group applies.
In every new shell:
  source /opt/ros/jazzy/setup.bash
  source $VENV/bin/activate
  source $PWD/install/setup.bash
EOF
