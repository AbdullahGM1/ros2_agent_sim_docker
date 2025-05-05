#!/bin/bash
# Runs a docker container for Custom ROS2 Humble + Gazebo Garden + PX4 + MAVROS + ROSA + ChatOllama + Qwen3 dev environment
#
# Authors: Based on Mohammed Abdelkader's script, adapted for ROS2 stack
# Maintained by: AbdullahGM1 <agm.musalami@gmail.com>

# Docker configuration
DOCKER_REPO="ros2-agent-sim:latest"
CONTAINER_NAME="ros2_agent_sim"
WORKSPACE_DIR=~/${CONTAINER_NAME}_shared_volume
CMD=""
DOCKER_OPTS=""

SUDO_PASSWORD="user"

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

# This will enable running containers with different names
# It will create a local workspace and link it to the image's workspace
if [ "$1" != "" ]; then
    CONTAINER_NAME=$1
fi
WORKSPACE_DIR=~/${CONTAINER_NAME}_shared_volume
if [ ! -d $WORKSPACE_DIR ]; then
    mkdir -p $WORKSPACE_DIR
fi
echo "Container name:$CONTAINER_NAME WORKSPACE DIR:$WORKSPACE_DIR" 

# Optional command parameter
if [ "$2" != "" ]; then
    CMD=$2
fi

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

# Allow X server connections from docker containers - NOT recommended for security
xhost +local:root
 
echo "Starting Container: ${CONTAINER_NAME} with image: $DOCKER_REPO"

# Define the command to run inside the container
CMD="export DEV_DIR=/home/user/shared_volume && \
    export PX4_DIR=\$DEV_DIR/PX4-Autopilot &&\
    export ROS2_WS=/home/user/ros2_ws &&\
    export ROSA_WS=/home/user/rosa_ws &&\
    source /home/user/.bashrc &&\
    if [ -f "/home/user/shared_volume/ros2_ws/install/setup.bash" ]; then
        source /home/user/shared_volume/ros2_ws/install/setup.bash
    fi &&\
    /bin/bash"

if [[ -n "$SUDO_PASSWORD" ]]; then
    CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
fi

if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
        # cleanup
        echo "Restarting the container..."
        docker start ${CONTAINER_NAME}
    fi

    echo "Accessing running container..."
    docker exec --user user -it ${CONTAINER_NAME} env TERM=xterm-256color bash -c "${CMD}"

else
    echo "Running container ${CONTAINER_NAME}..."
    
    # Define ROS2/ROSA specific command for first run
    CMD="export DEV_DIR=/home/user/shared_volume &&\
        export PX4_DIR=\$DEV_DIR/PX4-Autopilot &&\
        export ROS2_WS=/home/user/ros2_ws &&\
        export ROSA_WS=/home/user/rosa_ws &&\
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
        -p 11434:11434 \
        $DOCKER_OPTS \
        ${DOCKER_REPO} \
        bash -c "${CMD}"
fi

# Copy install.sh to the shared volume if it doesn't exist
if [ ! -f "$WORKSPACE_DIR/ros2_ws/src/install.sh" ]; then
    echo "Creating directory structure and copying install.sh..."
    mkdir -p "$WORKSPACE_DIR/ros2_ws/src"
    cp scripts/install.sh "$WORKSPACE_DIR/ros2_ws/src/"
    echo "install.sh copied to shared volume"
fi

# Make install.sh executable and run it inside the container
echo "Making install.sh executable and running it..."
docker exec --user user ${CONTAINER_NAME} chmod +x /home/user/shared_volume/ros2_ws/src/install.sh
docker exec --user user ${CONTAINER_NAME} /bin/bash -c "cd /home/user/shared_volume/ros2_ws/src && ./install.sh"

# Check if install.sh completed successfully
if [ $? -eq 0 ]; then
    echo "✓ All dependencies installed successfully!"
else
    echo "✗ Installation failed. Please check the install.sh script."
fi

# Cleanup X server permissions when script exits
trap 'xhost -local:root' ERR EXIT

# Optional: Print useful information for the user
echo "
=== Container Information ===
Container Name: ${CONTAINER_NAME}
Shared Volume: ${WORKSPACE_DIR}
Ollama Port: 11434 (exposed)
Docker Image: ${DOCKER_REPO}

To rebuild the image:
./build.sh

To access your container again, run:
./docker_run.sh ${CONTAINER_NAME}

To stop the container:
docker stop ${CONTAINER_NAME}

To remove the container:
docker rm ${CONTAINER_NAME}
"