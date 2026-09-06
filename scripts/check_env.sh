#!/usr/bin/env bash
# One-shot ROS 2 availability / environment helper.
# Prints the running ROS distro and key commands, verifying the environment.
set -euo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS 2 environment is not sourced. Run:" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

echo "ROS_DISTRO=${ROS_DISTRO}"
echo "ROS_VERSION=${ROS_VERSION:-n/a}"
echo "Python: $(python3 --version)"

python3 - <<'PY'
import importlib.util
for mod in ("rclpy", "numpy", "cv2"):
    spec = importlib.util.find_spec(mod)
    print(f"  {mod}: {'OK' if spec else 'MISSING'}")
PY