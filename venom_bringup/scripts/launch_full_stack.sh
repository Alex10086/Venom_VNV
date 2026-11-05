#!/bin/bash
set -e

# systemd owns the entire launch process tree, including static TF children.
if [ "${SCOUT_FULL_STACK_MANAGED:-0}" != 1 ]; then
    exec systemctl --user start scout-full-stack.service
fi

source /opt/ros/humble/setup.bash
source /home/hyq/venom_ws/install/setup.bash

if ! ip -details link show can0 | grep -q 'state ERROR-ACTIVE'; then
    echo "CAN0 is not ready. Run: sudo ip link set can0 up type can bitrate 500000" >&2
    exit 1
fi

# Display and auth auto-detection
if [ -S /tmp/.X11-unix/X0 ]; then
    export DISPLAY=:0
elif [ -S /tmp/.X11-unix/X1 ]; then
    export DISPLAY=:1
else
    export DISPLAY=:0
fi

if [ -f /run/user/1000/gdm/Xauthority ]; then
    export XAUTHORITY=/run/user/1000/gdm/Xauthority
elif [ -f /home/hyq/.Xauthority ]; then
    export XAUTHORITY=/home/hyq/.Xauthority
fi
export XDG_RUNTIME_DIR=/run/user/1000

# Do not unlock the desktop or modify the active login session.
xset dpms force on 2>/dev/null || true

echo "Starting Scout Mini Full Mapping & Navigation Stack + RViz on DISPLAY=${DISPLAY}..."
exec ros2 launch venom_bringup scout_mini_mapping.launch.py headless:=false
