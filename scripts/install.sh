#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to run sudo commands (no password needed with sudoers setup)
run_sudo() {
    sudo $@
}

if [ -z "${DEV_DIR}" ]; then
  echo "Error: DEV_DIR environment variable is not set. Set it using export DEV_DIR=<DEV_DIR_deirectory_that_should_contain_PX4-Autopilot_and_ros2_ws>"
  exit 1
fi
echo "DEV_DIR=$DEV_DIR"
sleep 1
echo "GIT_USER=$GIT_USER"
echo "GIT_TOKEN=$GIT_TOKEN"
sleep 1

#Defining Paths 
ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src
PX4_DIR=$DEV_DIR/PX4-Autopilot

# Debug information
echo "Current environment:"
echo "DEV_DIR = $DEV_DIR"
echo "ROS2_WS = $ROS2_WS"
echo "PX4_DIR = $PX4_DIR"
echo "Current directory: $(pwd)"
echo "Content of DEV_DIR: $(ls -la $DEV_DIR)"

# Clone and build PX4-Autopilot if it doesn't exist
if [ ! -d "$PX4_DIR" ]; then
    echo "Cloning $PX4_DIR..."
    cd $DEV_DIR
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd $PX4_DIR
    git checkout v1.14.0
    make submodulesclean
    make clean
    make distclean
else
    echo "PX4_DIR=$PX4_DIR already exists"
    cd $PX4_DIR
    make submodulesclean
    make clean
    make distclean
    git checkout v1.14.0
    make submodulesclean
    make clean
    make distclean
fi

# Build px4_sitl
cd $PX4_DIR && make px4_sitl

# Copy models and worlds to PX4_DIR
echo && echo "Copying models and worlds to ${PX4_DIR}" && echo
sleep 1
cp -rv ${ROS2_SRC}/PX4_config/models/* ${PX4_DIR}/Tools/simulation/gz/models/
cp -rv ${ROS2_SRC}/PX4_config/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/

# Copy airframes file to PX4_DIR
echo && echo "Copying airframe configuration to ${PX4_DIR}" && echo
cp -v ${ROS2_SRC}/PX4_config/4022_gz_x500_lidar_camera ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/
cp -v ${ROS2_SRC}/PX4_config/4023_gz_x3_uav ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/


# Replace CMakeLists.txt in airframes directory
echo && echo "Replacing CMakeLists.txt in airframes directory" && echo
cp -v ${ROS2_SRC}/PX4_config/CMakeLists.txt ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Now try to build 
echo "Building PX4_SITL..."
cd $PX4_DIR && make px4_sitl || {
    echo "${YELLOW}Warning: PX4 build failed, but continuing with script${NC}"
}

# Verify the content of the shared volume after operations
echo "Content of DEV_DIR after operations:"
ls -la $DEV_DIR

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# 
# MAVROS SETUP 
# 
echo "Cloning mavlink package ... " && sleep 1
if [ ! -d "$ROS2_SRC/mavlink" ]; then
    cd $ROS2_SRC
    git clone  https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
    cd $ROS2_SRC/mavlink && git checkout release/humble/mavlink/2023.9.9-1
fi
# Custom mavros pkg is required to handle TF issues in multi-vehicle simulation
echo "Cloning custom mavros package ... " && sleep 1
if [ ! -d "$ROS2_SRC/mavros" ]; then
    cd $ROS2_SRC
    git clone  https://github.com/AbdullahGM1/mavros.git
    cd $ROS2_SRC/mavros && git checkout ros2_humble
fi
 
# Initialize rosdep if needed and install dependencies
echo "Running rosdep..."
cd "$ROS2_WS"
rosdep init || echo "Rosdep already initialized"
rosdep update
rosdep install --from-paths src --ignore-src -r -y || echo "WARNING: rosdep install exited with non-zero status"

# Build packages
echo "Building mavros packages..."
cd "$ROS2_WS" && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros --executor sequential || echo "WARNING: mavros build failed"
cd "$ROS2_WS" && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential || echo "WARNING: mavros_extras build failed"
cd "$ROS2_WS" && colcon build || echo "WARNING: full build failed"

echo "${GREEN}=== MAVROS SETUP COMPLETE ===${NC}"


echo -e "${GREEN}Installing Python dependencies...${NC}"
run_sudo pip3 install \
    rich \
    langchain \
    langchain-ollama \
    langchain-community \
    opencv-python \
    numpy

# Check if ollama is installed
if ! command_exists ollama; then
    echo -e "${YELLOW}Installing Ollama...${NC}"
    run_sudo bash -c "curl -fsSL https://ollama.ai/install.sh | sh"
    
    # Start ollama service
    run_sudo systemctl start ollama
    run_sudo systemctl enable ollama
else
    echo -e "${GREEN}Ollama is already installed${NC}"
fi

# Pull the default model
echo -e "${GREEN}Pulling Qwen3:8b model for LLM...${NC}"
ollama pull qwen3:8b

echo -e "${GREEN}Installation complete!${NC}"
echo
echo -e "${YELLOW}Next steps:${NC}"
echo -e "${GREEN}Installation successful!${NC}"

mv -v $ROS2_SRC/PX4_config $DEV_DIR/ 2>/dev/null || true

cd $HOME