#!/bin/bash
set -e

# Always source ROS in container shells.
source /opt/ros/noetic/setup.bash
# If workspace overlay exists, source it too.
if [ -f /workspace/devel/setup.bash ]; then
  source /workspace/devel/setup.bash
fi

# Optional headless display + VNC (off by default).
# Enable with: ENABLE_HEADLESS_DISPLAY=1
if [ "${ENABLE_HEADLESS_DISPLAY:-0}" = "1" ]; then
  Xvfb :1 -screen 0 1600x900x24 >/tmp/xvfb.log 2>&1 &
  x11vnc -display :1 -nopw -forever -shared -quiet >/tmp/x11vnc.log 2>&1 &
  sleep 1
  export DISPLAY=:1
fi

exec "$@"
