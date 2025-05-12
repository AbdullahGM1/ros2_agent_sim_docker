
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

# SIM_PKG_URL=''
# if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
#     SIM_PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/SMART-TRACK.git
# else
#     SIM_PKG_URL=https://github.com/mzahana/SMART-TRACK.git
# fi

# # Clone the SMART-TRACK if it doesn't exist
# if [ ! -d "$ROS2_SRC/smart_track" ]; then
#     cd $ROS2_SRC
#     git clone $SIM_PKG_URL smart_track && cd $ROS2_SRC/smart_track && git pull origin main
# else
#     cd $ROS2_SRC/smart_track && git pull origin main
# fi

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

# Installing Python dependencies
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


cd $ROS2_WS && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y

cd $ROS2_WS && MAKEFLAGS='j1 -l1' colcon  build --packages-up-to mavros --executor sequential

cd $ROS2_WS && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential

cd $ROS2_WS && colcon build

echo "DONE. Pkgs are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"
# echo "Source the ros2_ws and use <ros2 launch smart_track observer.launch.py> to run the simulation"
cd $HOME


##############################
