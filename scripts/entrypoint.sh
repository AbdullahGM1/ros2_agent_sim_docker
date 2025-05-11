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

# Synchronize files from backup to shared volume if needed
if [ -d "/home/user/shared_volume" ]; then
    # Create ros2_ws/src directory structure in shared volume
    if [ ! -d "/home/user/shared_volume/ros2_ws/src" ]; then
        echo "Creating ros2_ws/src directory in shared volume..."
        mkdir -p /home/user/shared_volume/ros2_ws/src
    fi
    
    # Check if install.sh exists in shared volume
    if [ ! -f "/home/user/shared_volume/install.sh" ] && [ -f "/home/user/backup/install.sh" ]; then
        echo "Copying install.sh to shared volume..."
        cp -v /home/user/backup/install.sh /home/user/shared_volume/
        chmod +x /home/user/shared_volume/install.sh
    fi
    
    # Check if PX4_config exists in shared volume
    if [ ! -d "/home/user/shared_volume/PX4_config" ] && [ -d "/home/user/backup/PX4_config" ]; then
        echo "Copying PX4_config to shared volume..."
        cp -rv /home/user/backup/PX4_config /home/user/shared_volume/
    fi
    
    # Set proper permissions
    chown -R user:user /home/user/shared_volume
    echo "File synchronization completed."
fi

# Start Ollama service in background
ollama serve &

# Wait for Ollama to start
sleep 5

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