#!/bin/bash -e

# This script sets up the Ros2 Agent simulation environment

# Function to check if a command exists for Ollama
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

ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src
PX4_DIR=$DEV_DIR/PX4-Autopilot
PX4_config=$DEV_DIR/PX4_config
OSQP_SRC=$DEV_DIR

if [ ! -d "$ROS2_WS" ]; then
  echo "Creating $ROS2_SRC"
  mkdir -p $ROS2_SRC
fi

# Clone the ros2_agent_sim repository if it doesn't exist
ROS2_AGENT_SIM_URL=https://github.com/AbdullahGM1/ros2_agent_sim.git

# Clone the ros2_agent_sim if it doesn't exist
if [ ! -d "$ROS2_SRC/ros2_agent_sim" ]; then
    cd $ROS2_SRC
    git clone $ROS2_AGENT_SIM_URL ros2_agent_sim && cd $ROS2_SRC/ros2_agent_sim && git pull origin main
else
    cd $ROS2_SRC/ros2_agent_sim && git pull origin main
fi

# Clone and build PX4-Autopilot if it doesn't exist
if [ ! -d "$PX4_DIR" ]; then
    echo "Cloning $PX4_DIR..."
    cd $DEV_DIR
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd $PX4_DIR
    git fetch --all --tags
    git checkout -f v1.14.0  # Force checkout
    git reset --hard v1.14.0  # Reset any changes
    git submodule update --init --recursive  # Update submodules to match
    make distclean
else
    echo "PX4_DIR=$PX4_DIR already exists"
    cd $PX4_DIR
    git fetch --all --tags
    git checkout -f v1.14.0  # Force checkout
    git reset --hard v1.14.0  # Reset any changes
    git submodule update --init --recursive  # Update submodules to match
    make distclean
fi

# Verify PX4 version
PX4_VERSION=$(cd $PX4_DIR && git describe --tags)
echo "PX4 version: $PX4_VERSION"
if [[ "$PX4_VERSION" != "v1.14.0" ]]; then
    echo "Warning: PX4 version is not v1.14.0. Got $PX4_VERSION instead."
    echo "This may cause compatibility issues."
fi

# Build px4_sitl
cd $PX4_DIR && make px4_sitl

# Copy files to $PX4_DIR
echo && echo  "Copying files to ${PX4_DIR}" && echo
sleep 1
cp -r ${PX4_config}/models/* ${PX4_DIR}/Tools/simulation/gz/models/
cp -r ${PX4_config}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/
cp -r ${PX4_config}/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/
cd $PX4_DIR && make px4_sitl

# Install MAVROS packages from apt
echo "Installing MAVROS packages from apt..." && sleep 1
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-msgs

#
# MAVROS
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

cd $ROS2_WS && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y

cd $ROS2_WS && MAKEFLAGS='j1 -l1' colcon  build --packages-up-to mavros --executor sequential

cd $ROS2_WS && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential

cd $ROS2_WS && colcon build

echo "DONE. Pkgs are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"

# Add Python local bin to PATH
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Installing Python dependencies
echo -e "${GREEN}Installing Python dependencies...${NC}"
pip3 install \
    rich \
    langchain \
    langchain-ollama \
    langchain-community \
    opencv-python \
    numpy==1.24.3 --force-reinstall

# dependency 
pip3 install rospkg
pip3 install PyYAML==6.0.1
pip3 install langchain-community~=0.3.21

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

# Pull Qwen3: 8b model
echo -e "${GREEN}Pulling Qwen3:8b model for LLM...${NC}"
if command_exists ollama; then
    ollama pull qwen3:8b || echo "Failed to pull Qwen3:8b model, you may need to pull it manually"
else
    echo "Ollama not available, skipping model pull"
fi

echo -e "${GREEN}Installation complete!${NC}"
echo -e "${YELLOW}Next steps:${NC}"
echo -e "${GREEN}Installation successful!${NC}"

echo "DONE. Pkgs are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"

# Final message
success "INSTALLATION COMPLETE! Packages are built successfully."
success "Models and airframe config files are copied to the respective folders in the ${PX4_DIR} directory"
echo -e "${YELLOW}Next steps:${NC}"
echo -e "1. Source the workspace:  ${GREEN}source $ROS2_WS/install/setup.bash${NC}"
echo -e "2. Launch the simulation: ${GREEN}ros2 launch ros2_agent_sim drone.launch.py${NC}"
echo -e "3. Run the ROS2 agent:    ${GREEN}ros2 run ros2_agent ros2_agent_node${NC}"
cd $HOME