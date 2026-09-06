#!/usr/bin/env bash
# Mirror the generated package config into the workspace so that runtime
# configuration edits take effect without a rebuild.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
BRINGUP_DIR="$WORKSPACE_ROOT/src/assistant_bringup"
INSTALL_DIR="$WORKSPACE_ROOT/install/assistant_bringup/share/assistant_bringup"

if [ ! -d "$INSTALL_DIR" ]; then
  echo "Install directory not found; have you run 'colcon build' yet?" >&2
  exit 1
fi

cp "$BRINGUP_DIR"/config/* "$INSTALL_DIR/config/"
cp "$BRINGUP_DIR"/launch/*.py "$INSTALL_DIR/launch/"

echo "Config mirrored into install tree."