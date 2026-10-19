#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Multimodal Assistant - Pipeline Runner & Monitor
# ==============================================================================
# Usage:
#   ./scripts/run_pipeline.sh          # Launch full voice + vision pick pipeline
#   ./scripts/run_pipeline.sh monitor  # Echo live object detections (/object_detections)
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

MODE="${1:-launch}"

case "$MODE" in
    monitor|echo)
        echo "--> Monitoring live detections on /object_detections..."
        exec ros2 topic echo /object_detections
        ;;
    launch|start|"")
        echo "--> Launching multimodal detect & pick pipeline..."
        exec ros2 launch assistant_bringup detect_pick.launch.py \
            workspace_camera_url:=http://192.168.1.2:8080/video \
            voice_stream_url:=http://192.168.1.2:8080/audio.wav \
            auto_pick:=true \
            mock_mode:=false
        ;;
    *)
        echo "Usage: $0 [launch|monitor]"
        echo "  launch   : Run full detect_pick pipeline (default)"
        echo "  monitor  : Run ros2 topic echo /object_detections"
        exit 1
        ;;
esac
