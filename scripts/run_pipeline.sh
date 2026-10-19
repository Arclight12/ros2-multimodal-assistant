#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Multimodal Assistant - Pipeline Runner & Monitor
# ==============================================================================
# Usage:
#   ./scripts/run_pipeline.sh                    # Launch pipeline with default IP (192.168.1.2)
#   ./scripts/run_pipeline.sh 192.168.1.50       # Launch pipeline with custom phone IP
#   ./scripts/run_pipeline.sh launch 192.168.1.50# Launch pipeline with custom phone IP
#   ./scripts/run_pipeline.sh monitor            # Echo live object detections (/object_detections)
#
# Environment variables:
#   PHONE_IP=192.168.1.50 ./scripts/run_pipeline.sh
#   CAMERA_PORT=8080 ./scripts/run_pipeline.sh
# ==============================================================================

set -e

REPO_DIR="/home/yedhu/ros2-multimodal-assistant"
cd "$REPO_DIR"

# Source environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
fi

if [ -f "$REPO_DIR/install/setup.bash" ]; then
    source "$REPO_DIR/install/setup.bash"
fi

# Export Riva / NVIDIA API key if not already set
export NVIDIA_API_KEY="${NVIDIA_API_KEY:-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}"

# Parse arguments: mode and IP
MODE="launch"
TARGET_IP="${PHONE_IP:-192.168.1.2}"
PORT="${CAMERA_PORT:-8080}"

if [ "$1" = "monitor" ] || [ "$1" = "echo" ]; then
    MODE="monitor"
elif [ "$1" = "launch" ] || [ "$1" = "start" ]; then
    MODE="launch"
    if [ -n "$2" ]; then
        TARGET_IP="$2"
    fi
elif [ -n "$1" ]; then
    # If the first argument looks like an IP address (contains dots), use as IP
    if [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        TARGET_IP="$1"
    else
        echo "Usage: $0 [launch [PHONE_IP] | monitor | PHONE_IP]"
        echo "Examples:"
        echo "  $0"
        echo "  $0 192.168.1.50"
        echo "  $0 launch 192.168.1.50"
        echo "  $0 monitor"
        exit 1
    fi
fi

CAMERA_URL="http://${TARGET_IP}:${PORT}/video"
VOICE_URL="http://${TARGET_IP}:${PORT}/audio.wav"

case "$MODE" in
    monitor)
        echo "--> Monitoring live detections on /object_detections..."
        exec ros2 topic echo /object_detections
        ;;
    launch)
        echo "--> Launching multimodal detect & pick pipeline..."
        echo "    Phone IP:     $TARGET_IP"
        echo "    Camera URL:   $CAMERA_URL"
        echo "    Audio URL:    $VOICE_URL"
        exec ros2 launch assistant_bringup detect_pick.launch.py \
            workspace_camera_url:="$CAMERA_URL" \
            voice_stream_url:="$VOICE_URL" \
            auto_pick:=true \
            mock_mode:=false
        ;;
esac
