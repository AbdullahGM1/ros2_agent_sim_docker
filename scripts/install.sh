#!/bin/bash
echo "Installing Python dependencies and Ollama for Go2 ROS Agent..."
# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

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

# Clone and build PX4-Autopilot with a two-step approach
if [ ! -d "$PX4_DIR" ]; then
    echo "${YELLOW}PX4-Autopilot not found in shared volume, cloning...${NC}"
    
    # Step 1: Clone to a temporary directory inside the container
    TEMP_DIR="/tmp/px4_temp"
    echo "Cloning to temporary location: $TEMP_DIR"
    
    # Remove temp dir if it exists
    rm -rf $TEMP_DIR
    
    # Clone to temp directory
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git $TEMP_DIR
    cd $TEMP_DIR
    git checkout v1.14.0
    
    # Step 2: Copy to shared volume using rsync
    echo "Creating target directory: $PX4_DIR"
    mkdir -p $PX4_DIR
    
    echo "Copying from temp directory to shared volume..."
    rsync -av --progress $TEMP_DIR/ $PX4_DIR/
    
    # Clean up temp directory
    echo "Cleaning up temporary directory..."
    rm -rf $TEMP_DIR
    
    # Fix permissions
    echo "Setting correct permissions..."
    sudo chown -R $(id -u):$(id -g) $PX4_DIR
    
    # Now perform initialization in the shared volume
    cd $PX4_DIR
    echo "Running initial setup in shared volume..."
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
echo "Building PX4_SITL..."
cd $PX4_DIR && make px4_sitl

# Copy models and worlds to PX4_DIR
echo && echo "Copying models and worlds to ${PX4_DIR}" && echo
sleep 1
mkdir -p ${PX4_DIR}/Tools/simulation/gz/models/
mkdir -p ${PX4_DIR}/Tools/simulation/gz/worlds/
cp -rv ${ROS2_SRC}/PX4_config/models/* ${PX4_DIR}/Tools/simulation/gz/models/
cp -rv ${ROS2_SRC}/PX4_config/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/

# Copy airframe file to PX4_DIR
echo && echo "Copying airframe configuration to ${PX4_DIR}" && echo
mkdir -p ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/
cp -v ${ROS2_SRC}/PX4_config/4022_gz_x500_lidar_camera ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Replace CMakeLists.txt in airframes directory
echo && echo "Replacing CMakeLists.txt in airframes directory" && echo
cp -v ${ROS2_SRC}/PX4_config/CMakeLists.txt ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Force a sync to ensure files are written to disk
sync

# 
echo "Fixing CMakeLists.txt for missing airframes..."
sed -i '/1015_gazebo-classic_iris_obs_avoid/d' ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
# Rebuild after copying files
cd $PX4_DIR && make px4_sitl

# Verify the content of the shared volume after all operations
echo "Content of DEV_DIR after all operations:"
ls -la $DEV_DIR

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

#
#
# MAVROS
#
echo "Setting up MAVROS and dependencies..."

# Check and create the src directory if it doesn't exist
if [ ! -d "$ROS2_SRC" ]; then
    echo "Creating ROS2 source directory..."
    mkdir -p "$ROS2_SRC"
    sudo chown -R $(id -u):$(id -g) "$ROS2_SRC"
fi

# Clone mavlink with better error handling
echo "Cloning mavlink package..." 
if [ ! -d "$ROS2_SRC/mavlink" ]; then
    cd "$ROS2_SRC"
    echo "Current directory: $(pwd)"
    echo "Cloning from: https://github.com/ros2-gbp/mavlink-gbp-release.git"
    
    git clone https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink || {
        echo "${RED}Failed to clone mavlink repository${NC}"
        echo "Error code: $?"
        echo "Current directory: $(pwd)"
        echo "Directory contents: $(ls -la)"
    }
    
    if [ -d "$ROS2_SRC/mavlink" ]; then
        cd "$ROS2_SRC/mavlink" 
        git checkout release/humble/mavlink/2023.9.9-1
    else
        echo "${RED}Mavlink directory not created, skipping checkout${NC}"
    fi
else
    echo "${YELLOW}Mavlink directory already exists, skipping clone${NC}"
fi

# Clone custom mavros with better error handling
echo "Cloning custom mavros package..." 
if [ ! -d "$ROS2_SRC/mavros" ]; then
    cd "$ROS2_SRC"
    echo "Current directory: $(pwd)"
    echo "Cloning from: https://github.com/AbdullahGM1/mavros.git"
    
    git clone https://github.com/AbdullahGM1/mavros.git || {
        echo "${RED}Failed to clone mavros repository${NC}"
        echo "Error code: $?"
        echo "Current directory: $(pwd)"
        echo "Directory contents: $(ls -la)"
    }
    
    if [ -d "$ROS2_SRC/mavros" ]; then
        cd "$ROS2_SRC/mavros"
        git checkout ros2_humble
    else
        echo "${RED}Mavros directory not created, skipping checkout${NC}"
    fi
else
    echo "${YELLOW}Mavros directory already exists, skipping clone${NC}"
fi

# Initialize rosdep if needed and install dependencies
echo "Running rosdep..."
cd "$ROS2_WS"
rosdep init || echo "Rosdep already initialized"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build packages
echo "Building mavros packages..."
cd "$ROS2_WS" && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros --executor sequential
cd "$ROS2_WS" && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential
cd "$ROS2_WS" && colcon build

echo "DONE. Packages are built. Models and airframe config files are copied to the respective folders in the ${PX4_DIR} directory"
# Function to run commands with sudo
run_sudo() {
    sudo "$@"
}

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
echo "1. Build your package:"
echo "   colcon build --packages-select go2_ros_agent"
echo
echo "2. Run the agent:"
echo "   ros2 run go2_ros_agent go2_agent"
echo
echo -e "${GREEN}Installation successful!${NC}"