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

# Process arguments
if [ "$1" != "" ]; then
    CONTAINER_NAME=$1
fi
WORKSPACE_DIR=~/${CONTAINER_NAME}_shared_volume

# Create workspace directory if it doesn't exist
if [ ! -d $WORKSPACE_DIR ]; then
    mkdir -p $WORKSPACE_DIR
    FIRST_RUN=true
fi
echo "Container name: $CONTAINER_NAME WORKSPACE DIR: $WORKSPACE_DIR" 

# Check if there's a command parameter
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

echo "GIT_USER=$GIT_USER"
echo "GIT_TOKEN=$GIT_TOKEN"

CMD="export DEV_DIR=/home/user/shared_volume && \
    export PX4_DIR=\$DEV_DIR/PX4-Autopilot &&\
    export ROS2_WS=\$DEV_DIR/ros2_ws &&\
    export OSQP_SRC=\$DEV_DIR &&\
        source /home/user/.bashrc &&\
        if [ -f "/home/user/shared_volume/ros2_ws/install/setup.bash" ]; then
            source /home/user/shared_volume/ros2_ws/install/setup.bash
        fi &&\
         /bin/bash"
if [[ -n "$GIT_TOKEN" ]] && [[ -n "$GIT_USER" ]]; then
    CMD="export GIT_USER=$GIT_USER && export GIT_TOKEN=$GIT_TOKEN && $CMD"
fi

if [[ -n "$SUDO_PASSWORD" ]]; then
    CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
fi

# echo $CMD

if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
        # cleanup
        echo "Restarting the container..."
        docker start ${CONTAINER_NAME}
    fi

    docker exec --user user -it ${CONTAINER_NAME} env TERM=xterm-256color bash -c "${CMD}"

else

CMD="export DEV_DIR=/home/user/shared_volume &&\
        export PX4_DIR=\$DEV_DIR/PX4-Autopilot &&\
        export ROS2_WS=\$DEV_DIR/ros2_ws &&\
        export OSQP_SRC=\$DEV_DIR &&\
        source /home/user/.bashrc &&\
        if [ -f "/home/user/shared_volume/ros2_ws/install/setup.bash" ]; then
            source /home/user/shared_volume/ros2_ws/install/setup.bash
        fi &&\
        /bin/bash"

    if [[ -n "$GIT_TOKEN" ]] && [[ -n "$GIT_USER" ]]; then
    CMD="export GIT_USER=$GIT_USER && export GIT_TOKEN=$GIT_TOKEN && $CMD"
    fi

    if [[ -n "$SUDO_PASSWORD" ]]; then
        CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
    fi

    echo "Running container ${CONTAINER_NAME}..."

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
        $DOCKER_OPTS \
        ${DOCKER_REPO} \
        bash -c "${CMD}"
fi