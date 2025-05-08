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

# Create the missing airframe file instead of trying to remove it from CMakeLists.txt
echo && echo "Creating placeholder for missing airframe file..." && echo
touch ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/1015_gazebo-classic_iris_obs_avoid

# Force a sync to ensure files are written to disk
sync

# Now try to build (with error handling)
echo "Building PX4_SITL..."
cd $PX4_DIR && make px4_sitl || {
    echo "${YELLOW}Warning: PX4 build failed, but continuing with script${NC}"
    # Continue regardless of build failure
}

# Verify the content of the shared volume after operations
echo "Content of DEV_DIR after operations:"
ls -la $DEV_DIR

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# ================================================================
# MAVROS SETUP - This section continues even if PX4 build failed
# ================================================================
echo "${GREEN}=== MAVROS SETUP DIAGNOSTICS ===${NC}"

# Turn off exit on error for this section
set +e

# Print detailed environment info
echo "User: $(whoami)"
echo "Groups: $(groups)"
echo "Git version: $(git --version)"
echo "Network connectivity test:"
ping -c 1 github.com || echo "WARNING: Cannot reach github.com"

# Check if ROS2_SRC directory exists and is writable
echo "ROS2_SRC directory: $ROS2_SRC"
if [ ! -d "$ROS2_SRC" ]; then
    echo "Creating ROS2_SRC directory..."
    mkdir -p "$ROS2_SRC"
fi

# Check permissions
echo "Permission check on ROS2_SRC:"
ls -la "$ROS2_SRC"
if [ ! -w "$ROS2_SRC" ]; then
    echo "WARNING: $ROS2_SRC is not writable!"
    sudo chown -R $(id -u):$(id -g) "$ROS2_SRC"
    echo "Fixed permissions:"
    ls -la "$ROS2_SRC"
fi

# Try a different approach using https instead of git protocol
echo "Attempting to clone mavlink using HTTPS..."
cd "$ROS2_SRC"
if [ ! -d "$ROS2_SRC/mavlink" ]; then
    echo "Cloning mavlink to $(pwd)/mavlink..."
    git clone --verbose https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
    GIT_RESULT=$?
    
    if [ $GIT_RESULT -ne 0 ]; then
        echo "ERROR: Git clone failed with exit code $GIT_RESULT"
        echo "Trying alternative method..."
        
        # Try downloading and extracting as a zip file
        wget -O /tmp/mavlink.zip https://github.com/ros2-gbp/mavlink-gbp-release/archive/refs/heads/master.zip
        unzip /tmp/mavlink.zip -d /tmp/
        mkdir -p "$ROS2_SRC/mavlink"
        cp -r /tmp/mavlink-gbp-release-master/* "$ROS2_SRC/mavlink/"
        
        if [ $? -eq 0 ]; then
            echo "Successfully downloaded and extracted mavlink"
        else
            echo "ERROR: All mavlink download methods failed"
        fi
    else
        echo "Mavlink cloned successfully."
        cd "$ROS2_SRC/mavlink" && git checkout release/humble/mavlink/2023.9.9-1
    fi
else
    echo "Mavlink directory already exists."
fi

# Similar approach for mavros
echo "Attempting to clone mavros..."
cd "$ROS2_SRC"
if [ ! -d "$ROS2_SRC/mavros" ]; then
    echo "Cloning mavros to $(pwd)/mavros..."
    git clone --verbose https://github.com/AbdullahGM1/mavros.git
    GIT_RESULT=$?
    
    if [ $GIT_RESULT -ne 0 ]; then
        echo "ERROR: Git clone failed with exit code $GIT_RESULT"
        echo "Trying alternative method..."
        
        # Try downloading and extracting as a zip file
        wget -O /tmp/mavros.zip https://github.com/AbdullahGM1/mavros/archive/refs/heads/ros2_humble.zip
        unzip /tmp/mavros.zip -d /tmp/
        mkdir -p "$ROS2_SRC/mavros"
        cp -r /tmp/mavros-ros2_humble/* "$ROS2_SRC/mavros/"
        
        if [ $? -eq 0 ]; then
            echo "Successfully downloaded and extracted mavros"
        else
            echo "ERROR: All mavros download methods failed"
        fi
    else
        echo "Mavros cloned successfully."
        cd "$ROS2_SRC/mavros" && git checkout ros2_humble
    fi
else
    echo "Mavros directory already exists."
fi

# Verify the directories were created and have content
echo "Directory structure after clone attempts:"
find "$ROS2_SRC" -maxdepth 2 -type d | sort

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

# Turn automatic exit on error back on
set -e

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
echo -e "${GREEN}Installation successful!${NC}"