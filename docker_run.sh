#!/bin/bash
# Runs a docker container for Custom ROS2 Humble + Gazebo Garden + PX4 + MAVROS + ChatOllama + Qwen3 dev environment
#
# Authors: Based on Mohammed Abdelkader's script, adapted for ROS2 stack
# Maintained by: AbdullahGM1 <agm.musalami@gmail.com>

# Docker configuration
DOCKER_REPO="ros2-agent-sim:latest"
CONTAINER_NAME="ros2_agent_sim"
WORKSPACE_DIR=~/${CONTAINER_NAME}_shared_volume
CMD=""
DOCKER_OPTS=""
SETUP_MARKER="$WORKSPACE_DIR/.setup_complete"
FIRST_RUN=false

SUDO_PASSWORD="user"

# Process arguments
if [ "$1" != "" ]; then
    CONTAINER_NAME=$1
fi

WORKSPACE_DIR=~/${CONTAINER_NAME}_shared_volume
SETUP_MARKER="$WORKSPACE_DIR/.setup_complete"

# Check if there's a command parameter
if [ "$2" != "" ]; then
    CMD=$2
fi

# Create workspace directory if it doesn't exist
if [ ! -d $WORKSPACE_DIR ]; then
    mkdir -p $WORKSPACE_DIR
    FIRST_RUN=true
fi
echo "Container name: $CONTAINER_NAME WORKSPACE DIR: $WORKSPACE_DIR" 

# Get the current version of docker-ce
# Strip leading stuff before the version number so it can be compared
DOCKER_VER=$(dpkg-query -f='${Version}' --show docker-ce | sed 's/[0-9]://')
if dpkg --compare-versions 19.03 gt "$DOCKER_VER"
then
    echo "Docker version is less than 19.03, using nvidia-docker2 runtime"
    if ! dpkg --list | grep nvidia-docker2
    then
        echo "Please either update docker-ce to a version greater than 19.03 or install nvidia-docker2"
        echo "Continuing without GPU support..."
    else
        DOCKER_OPTS="$DOCKER_OPTS --runtime=nvidia"
    fi
else
    DOCKER_OPTS="$DOCKER_OPTS --gpus all"
fi
echo "GPU arguments: $DOCKER_OPTS"

# Setup X authentication
XAUTH=/tmp/.docker.xauth
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]
then
    echo XAUTH file does not exist. Creating one...
    touch $XAUTH
    chmod a+r $XAUTH
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    fi
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]
then
  echo "[$XAUTH] was not properly created. Exiting..."
  exit 1
fi

echo "Shared WORKSPACE_DIR: $WORKSPACE_DIR";

# Check if port 11434 is already in use
if lsof -Pi :11434 -sTCP:LISTEN -t >/dev/null ; then
    echo "Warning: Port 11434 is already in use. Using port 11435 instead."
    PORT_MAPPING="11435:11434"
else
    PORT_MAPPING="11434:11434"
fi

# Check if this is a first-time setup
if [ "$FIRST_RUN" = true ] || [ ! -f "$SETUP_MARKER" ]; then
    # This is the first run, so we need to copy files
    echo "First run detected or setup not yet completed."
    
    # Copy files if they don't exist
    if [ ! -f "$WORKSPACE_DIR/ros2_ws/src/install.sh" ]; then
        echo "Creating directory structure and copying install.sh..."
        mkdir -p "$WORKSPACE_DIR/ros2_ws/src"
        
        # Copy install.sh
        echo "Copying install.sh..."
        cp -v scripts/install.sh "$WORKSPACE_DIR/ros2_ws/src/" || {
            echo "Error: Failed to copy install.sh"
            echo "Current directory: $(pwd)"
            echo "Contents of scripts directory:"
            ls -la scripts/
            exit 1
        }
        
        # Copy PX4_config directory
        echo "Copying PX4_config directory..."
        cp -rv PX4_config "$WORKSPACE_DIR/ros2_ws/src/" || {
            echo "Error: Failed to copy PX4_config"
            echo "PX4_config exists? $([ -d PX4_config ] && echo 'Yes' || echo 'No')"
            exit 1
        }
        
        echo "Files copied successfully!"
        echo "Verification after copy:"
        ls -la "$WORKSPACE_DIR/ros2_ws/src/"
    fi
else
    echo "Setup already completed. Skipping file copy."
fi

# Allow X server connections from docker containers - NOT recommended for security
xhost +local:root
 
echo "Starting Container: ${CONTAINER_NAME} with image: $DOCKER_REPO"

# Define the command to run inside the container
CMD="export DEV_DIR=/home/user/shared_volume && \
    export PX4_DIR=\$DEV_DIR/PX4-Autopilot &&\
    export ROS2_WS=/home/user/ros2_ws &&\
    source /home/user/.bashrc &&\
    if [ -f "/home/user/shared_volume/ros2_ws/install/setup.bash" ]; then
        source /home/user/shared_volume/ros2_ws/install/setup.bash
    fi &&\
    /bin/bash"

if [[ -n "$SUDO_PASSWORD" ]]; then
    CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
fi

# Check if the container is already running or exists
CONTAINER_EXISTS=$(docker ps -aq -f name=${CONTAINER_NAME})
CONTAINER_RUNNING=$(docker ps -q -f name=${CONTAINER_NAME})

if [ -n "$CONTAINER_EXISTS" ]; then
    # Container exists
    if [ -z "$CONTAINER_RUNNING" ]; then
        # Container exists but is not running
        echo "Restarting the container..."
        docker start ${CONTAINER_NAME}
    fi

    # Configure sudo directly with inline commands
    echo "Configuring sudo access..."
    docker exec -u root ${CONTAINER_NAME} bash -c "echo 'user ALL=(ALL) NOPASSWD: ALL' > /etc/sudoers.d/user && chmod 440 /etc/sudoers.d/user"
    
    echo "Accessing running container..."
    docker exec --user user -it ${CONTAINER_NAME} env TERM=xterm-256color bash -c "${CMD}"
    
    # Just exit after the container session ends
    echo "Container session ended."
else
    echo "Running new container ${CONTAINER_NAME}..."
    
    # Define ROS2 specific command for first run
    CMD="export DEV_DIR=/home/user/shared_volume &&\
        export PX4_DIR=\$DEV_DIR/PX4-Autopilot &&\
        export ROS2_WS=/home/user/ros2_ws &&\
        export GZ_VERSION=garden &&\
        source /home/user/.bashrc &&\
        if [ -f "/home/user/shared_volume/ros2_ws/install/setup.bash" ]; then
            source /home/user/shared_volume/ros2_ws/install/setup.bash
        fi &&\
        /bin/bash"

    if [[ -n "$SUDO_PASSWORD" ]]; then
        CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
    fi

    # Run the container with all necessary options
    docker run -it \
        --network host \
        --env="DISPLAY=${DISPLAY}" \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e LOCAL_USER_ID="$(id -u)" \
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="$WORKSPACE_DIR:/home/user/shared_volume:rw" \
        --volume="/dev:/dev" \
        --name=${CONTAINER_NAME} \
        --privileged \
        --workdir /home/user/shared_volume \
        -p $PORT_MAPPING \
        $DOCKER_OPTS \
        ${DOCKER_REPO} \
        bash -c "${CMD}"
    
    # If this is the first run, we need to run the installation script
    if [ "$FIRST_RUN" = true ] || [ ! -f "$SETUP_MARKER" ]; then
        echo "Running initial setup..."
        
        # Configure sudo directly with inline commands
        echo "Configuring sudo access..."
        docker exec -u root ${CONTAINER_NAME} bash -c "echo 'user ALL=(ALL) NOPASSWD: ALL' > /etc/sudoers.d/user && chmod 440 /etc/sudoers.d/user"
        
        # Make install.sh executable and run it inside the container
        echo "Making install.sh executable and running it..."
        docker exec --user user ${CONTAINER_NAME} chmod +x /home/user/shared_volume/ros2_ws/src/install.sh
        docker exec --user user ${CONTAINER_NAME} /bin/bash -c "cd /home/user/shared_volume/ros2_ws/src && ./install.sh"
        
        # Check if install.sh completed successfully
        if [ $? -eq 0 ]; then
            echo "✓ All dependencies installed successfully!"
            # Mark setup as complete
            touch "$SETUP_MARKER"
        else
            echo "✗ Installation failed. Please check the install.sh script."
        fi
    fi
fi

# Cleanup X server permissions when script exits
xhost -local:root

# Optional: Print useful information for the user
echo "
=== Container Information ===
Container Name: ${CONTAINER_NAME}
Shared Volume: ${WORKSPACE_DIR}
Ollama Port: $(echo $PORT_MAPPING | cut -d':' -f1) (mapped to 11434 inside container)
Docker Image: ${DOCKER_REPO}

To stop the container:
docker stop ${CONTAINER_NAME}

To remove the container:
docker rm ${CONTAINER_NAME}
"