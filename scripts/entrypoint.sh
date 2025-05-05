#!/bin/bash

# Start virtual X server in the background
if [[ -x "$(command -v Xvfb)" && "$DISPLAY" == ":99" ]]; then
	echo "Starting Xvfb"
	Xvfb :99 -screen 0 1600x1200x24+32 &
fi

# Check if the ROS_DISTRO is passed and use it
# to source the ROS environment
if [ -n "${ROS_DISTRO}" ]; then
	source "/opt/ros/$ROS_DISTRO/setup.bash"
else
	# Default to ROS2 Humble
	source "/opt/ros/humble/setup.bash"
fi

# Source ROS2 workspace if exists
if [ -f "/home/user/ros2_ws/install/setup.bash" ]; then
	source "/home/user/ros2_ws/install/setup.bash"
fi

# Source ROSA workspace
source "/home/user/rosa_ws/install/setup.bash"

# Start Ollama service in background
ollama serve &

# Wait for Ollama to start
sleep 5

# Try to pull Qwen3:8b if not already available
if ! ollama list | grep -q qwen3:8b; then
    echo "Pulling Qwen3:8b model..."
    ollama pull qwen3:8b || echo "Failed to pull model, continuing anyway"
fi

# Use the LOCAL_USER_ID if passed in at runtime
if [ -n "${LOCAL_USER_ID}" ]; then
	echo "Starting with UID : $LOCAL_USER_ID"
	# modify existing user's id
	usermod -u $LOCAL_USER_ID user
	# run as user
	exec gosu user "$@"
else
	exec "$@"
fi