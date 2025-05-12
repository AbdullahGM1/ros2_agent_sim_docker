#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
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
OSQP_SRC=$DEV_DIR

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
    git fetch --all --tags
    git checkout -f v1.14.0
    git reset --hard v1.14.0
    git submodule update --init --recursive
    make distclean
else
    echo "PX4_DIR=$PX4_DIR already exists"
    cd $PX4_DIR
    git fetch --all --tags
    git checkout -f v1.14.0
    git reset --hard v1.14.0
    git submodule update --init --recursive
    make distclean
fi

# Verify we're on the correct version
PX4_VERSION=$(cd $PX4_DIR && git describe --tags)
echo "Verified PX4 version: $PX4_VERSION"
if [[ "$PX4_VERSION" != "v1.14.0" ]]; then
    echo "${RED}Error: Failed to checkout PX4 version v1.14.0, got $PX4_VERSION instead${NC}"
    echo "Will try to continue anyway..."
fi

# Try building with different methods
echo "Building PX4 SITL..."
cd $PX4_DIR
export srctree=$PX4_DIR

# Try different build approaches
DONT_RUN=1 make px4_sitl || {
    echo "${YELLOW}First build method failed, trying alternative...${NC}"
    make px4_sitl || {
        echo "${YELLOW}Warning: PX4 build failed, but continuing with script${NC}"
    }
}

# Check for PX4_config location
PX4_CONFIG_DIR=""
if [ -d "${DEV_DIR}/PX4_config" ]; then
    PX4_CONFIG_DIR="${DEV_DIR}/PX4_config"
    echo "Found PX4_config in DEV_DIR"
elif [ -d "${ROS2_SRC}/PX4_config" ]; then
    PX4_CONFIG_DIR="${ROS2_SRC}/PX4_config"
    echo "Found PX4_config in ROS2_SRC"
else
    echo "${RED}Error: PX4_config directory not found in ${DEV_DIR} or ${ROS2_SRC}${NC}"
    echo "Please make sure PX4_config is available in one of these locations"
    exit 1
fi

# Copy models and worlds to PX4_DIR
echo && echo "Copying models and worlds to ${PX4_DIR}" && echo
sleep 1

# Ensure target directories exist
mkdir -p ${PX4_DIR}/Tools/simulation/gz/models/
mkdir -p ${PX4_DIR}/Tools/simulation/gz/worlds/

# Verify source directories exist
if [ ! -d "${PX4_CONFIG_DIR}/models" ]; then
    echo "${RED}Error: Models directory not found: ${PX4_CONFIG_DIR}/models${NC}"
    echo "Contents of ${PX4_CONFIG_DIR}:"
    ls -la ${PX4_CONFIG_DIR}
    exit 1
fi

if [ ! -d "${PX4_CONFIG_DIR}/worlds" ]; then
    echo "${RED}Error: Worlds directory not found: ${PX4_CONFIG_DIR}/worlds${NC}"
    echo "Contents of ${PX4_CONFIG_DIR}:"
    ls -la ${PX4_CONFIG_DIR}
    exit 1
fi

# Copy files with full error checking
cp -rv ${PX4_CONFIG_DIR}/models/* ${PX4_DIR}/Tools/simulation/gz/models/ || {
    echo "${RED}Error: Failed to copy models${NC}"
    echo "Source: ${PX4_CONFIG_DIR}/models/*"
    echo "Destination: ${PX4_DIR}/Tools/simulation/gz/models/"
    ls -la ${PX4_CONFIG_DIR}/models/
    exit 1
}

cp -rv ${PX4_CONFIG_DIR}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/ || {
    echo "${RED}Error: Failed to copy worlds${NC}"
    echo "Source: ${PX4_CONFIG_DIR}/worlds/*"
    echo "Destination: ${PX4_DIR}/Tools/simulation/gz/worlds/"
    ls -la ${PX4_CONFIG_DIR}/worlds/
    exit 1
}

# Copy airframes file to PX4_DIR
echo && echo "Copying airframe configuration to ${PX4_DIR}" && echo

# Ensure target directory exists
mkdir -p ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Verify source files exist
for file in "${PX4_CONFIG_DIR}/4022_gz_x500_lidar_camera" "${PX4_CONFIG_DIR}/4023_gz_x3_uav" "${PX4_CONFIG_DIR}/CMakeLists.txt"; do
    if [ ! -f "$file" ]; then
        echo "${RED}Error: Required file not found: $file${NC}"
        echo "Contents of ${PX4_CONFIG_DIR}:"
        ls -la ${PX4_CONFIG_DIR}
        exit 1
    fi
done

# Copy airframe files
cp -v ${PX4_CONFIG_DIR}/4022_gz_x500_lidar_camera ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/ || {
    echo "${RED}Error: Failed to copy 4022_gz_x500_lidar_camera${NC}"
    echo "Source: ${PX4_CONFIG_DIR}/4022_gz_x500_lidar_camera"
    echo "Destination: ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
    ls -la ${PX4_CONFIG_DIR}/
    exit 1
}

cp -v ${PX4_CONFIG_DIR}/4023_gz_x3_uav ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/ || {
    echo "${RED}Error: Failed to copy 4023_gz_x3_uav${NC}"
    echo "Source: ${PX4_CONFIG_DIR}/4023_gz_x3_uav"
    echo "Destination: ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
    ls -la ${PX4_CONFIG_DIR}/
    exit 1
}

# Copy worlds models
cp -rv ${PX4_CONFIG_DIR}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/ || {
    echo "${RED}Error: Failed to copy worlds directory${NC}"
    echo "Source: ${PX4_CONFIG_DIR}/worlds/*"
    echo "Destination: ${PX4_DIR}/Tools/simulation/gz/worlds/"
    ls -la ${PX4_CONFIG_DIR}/worlds/
    exit 1
}

# Copy airframe models
p -rv ${PX4_CONFIG_DIR}/models/* ${PX4_DIR}/Tools/simulation/gz/models/ || {
    echo "${RED}Error: Failed to copy worlds directory${NC}"
    echo "Source: ${PX4_CONFIG_DIR}/worlds/*"
    echo "Destination: ${PX4_DIR}/Tools/simulation/gz/worlds/"
    ls -la ${PX4_CONFIG_DIR}/models/
    exit 1
}


# Replace CMakeLists.txt in airframes directory
echo && echo "Replacing CMakeLists.txt in airframes directory" && echo
cp -v ${PX4_CONFIG_DIR}/CMakeLists.txt ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/ || {
    echo "${RED}Error: Failed to copy CMakeLists.txt${NC}"
    echo "Source: ${PX4_CONFIG_DIR}/CMakeLists.txt"
    echo "Destination: ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
    ls -la ${PX4_CONFIG_DIR}/
    exit 1
}

# Rebuild after copying files
echo "Rebuilding PX4_SITL after file copies..."
cd $PX4_DIR && make px4_sitl || {
    echo "${YELLOW}Warning: PX4 rebuild failed, but continuing with script${NC}"
}

# 
# MAVROS SETUP 
# 
echo "Cloning mavlink package ... " && sleep 1
if [ ! -d "$ROS2_SRC/mavlink" ]; then
    mkdir -p "$ROS2_SRC"
    cd $ROS2_SRC
    git clone  https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
    cd $ROS2_SRC/mavlink && git checkout release/humble/mavlink/2023.9.9-1
fi

# Custom mavros pkg is required to handle TF issues in multi-vehicle simulation
echo "Cloning custom mavros package ... " && sleep 1
if [ ! -d "$ROS2_SRC/mavros" ]; then
    mkdir -p "$ROS2_SRC"
    cd $ROS2_SRC
    git clone  https://github.com/AbdullahGM1/mavros.git
    cd $ROS2_SRC/mavros && git checkout ros2_humble
fi
 
# Initialize rosdep if needed and install dependencies
echo "Running rosdep..."
mkdir -p "$ROS2_WS"
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
pip3 install \
    rich \
    langchain \
    langchain-ollama \
    langchain-community \
    opencv-python \
    numpy

# Check if ollama is installed
if ! command_exists ollama; then
    echo -e "${YELLOW}Installing Ollama...${NC}"
    curl -fsSL https://ollama.ai/install.sh | sh
    
    # Start ollama service
    systemctl start ollama || echo "Failed to start ollama service, you may need to start it manually"
    systemctl enable ollama || echo "Failed to enable ollama service, you may need to enable it manually"
else
    echo -e "${GREEN}Ollama is already installed${NC}"
fi

# Pull the default model
echo -e "${GREEN}Pulling Qwen3:8b model for LLM...${NC}"
if command_exists ollama; then
    ollama pull qwen3:8b || echo "Failed to pull Qwen3:8b model, you may need to pull it manually"
else
    echo "Ollama not available, skipping model pull"
fi

echo -e "${GREEN}Installation complete!${NC}"
echo
echo -e "${YELLOW}Next steps:${NC}"
echo -e "${GREEN}Installation successful!${NC}"

# Move PX4_config to DEV_DIR if it's in ROS2_SRC and not already in DEV_DIR
if [ -d "${ROS2_SRC}/PX4_config" ] && [ ! -d "${DEV_DIR}/PX4_config" ]; then
    echo "Moving PX4_config from ROS2_SRC to DEV_DIR..."
    mv -v $ROS2_SRC/PX4_config $DEV_DIR/ 2>/dev/null || true
fi

cd $HOME