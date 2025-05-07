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

# Clone and build PX4-Autopilot if it doesn't exist
if [ ! -d "$PX4_DIR" ]; then
    echo "Cloning $PX4_DIR to shared volume..."
    cd $DEV_DIR  # This ensures we're in the shared volume directory
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
mkdir -p ${PX4_DIR}/Tools/simulation/gz/models/
mkdir -p ${PX4_DIR}/Tools/simulation/gz/worlds/
cp -r ${ROS2_SRC}/PX4_config/models/* ${PX4_DIR}/Tools/simulation/gz/models/
cp -r ${ROS2_SRC}/PX4_config/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/

# Copy airframe file to PX4_DIR
echo && echo "Copying airframe configuration to ${PX4_DIR}" && echo
mkdir -p ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/
cp -v ${ROS2_SRC}/PX4_config/4022_gz_x500_lidar_camera ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Replace CMakeLists.txt in airframes directory
echo && echo "Replacing CMakeLists.txt in airframes directory" && echo
cp -v ${ROS2_SRC}/PX4_config/CMakeLists.txt ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Rebuild after copying files
cd $PX4_DIR && make px4_sitl

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

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