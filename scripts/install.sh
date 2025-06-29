#!/bin/bash -e

# This script sets up the Ros2 Agent simulation environment

# Function to check if a command exists for Ollama
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to fix ROS2 GPG key issues
fix_ros2_gpg_key() {
    # echo "Fixing ROS2 repository GPG key..."
    
    # Remove old key and add fresh one
    sudo apt-key del F42ED6FBAB17C654 2>/dev/null || true
    
    # Add the current ROS2 GPG key
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
    sudo apt-key add /tmp/ros.key
    
    # Alternative method using new keyring approach (more secure)
    sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || true
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Update the sources list to use the new keyring
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    echo "ROS2 GPG key updated successfully"
}

# Function to setup Gazebo repository
setup_gazebo_repository() {
    echo "Setting up Gazebo repository..."
    
    # Add Gazebo GPG key
    sudo rm -f /usr/share/keyrings/gazebo-keyring.gpg 2>/dev/null || true
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo gpg --dearmor -o /usr/share/keyrings/gazebo-keyring.gpg
    
    # Add Gazebo repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    echo "Gazebo repository added successfully"
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

# Fix ROS2 GPG key issues before any apt operations
fix_ros2_gpg_key

# Setup Gazebo repository
setup_gazebo_repository

# Update package lists after adding repositories
echo "Updating package lists..."
sudo apt update

# Clone the ros2_agent_sim repository if it doesn't exist
ROS2_AGENT_SIM_URL=https://github.com/AbdullahGM1/ros2_agent_sim.git

# Clone the ros2_agent_sim if it doesn't exist (using main branch with submodules)
if [ ! -d "$ROS2_SRC/ros2_agent_sim" ]; then
    echo "Cloning ros2_agent_sim (main branch with submodules)..."
    cd $ROS2_SRC
    git clone --recursive -b main $ROS2_AGENT_SIM_URL ros2_agent_sim
    cd $ROS2_SRC/ros2_agent_sim
    git pull origin main
    # Ensure submodules are up to date
    git submodule update --init --recursive
    echo "✅ ros2_agent_sim cloned with submodules successfully"
else
    echo "ros2_agent_sim already exists, updating to main branch..."
    cd $ROS2_SRC/ros2_agent_sim 
    git fetch origin
    git checkout main
    git pull origin main
    # Update submodules to match the main repo
    git submodule update --init --recursive
    echo "✅ ros2_agent_sim updated with submodules successfully"
fi

# Check and install simulation and ROS 2 dependencies
echo && echo "Checking and installing simulation and ROS 2 packages..." && echo

REQUIRED_APT_PACKAGES=(
  libgz-physics7
  libgz-physics7-dartsim
  ros-humble-xacro
  ros-humble-robot-localization
  ros-humble-velodyne
  ros-humble-velodyne-description
)

for pkg in "${REQUIRED_APT_PACKAGES[@]}"; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    echo "✅ $pkg is already installed"
  else
    echo "⏳ Installing $pkg..."
    sudo apt install -y "$pkg"
  fi
done

# Set Gazebo version for ROS-GZ Bridge
export GZ_VERSION=garden
echo "✅ GZ_VERSION set to 'garden'"

# Run rosdep update and install
echo "⏳ Running rosdep update and install..."
cd $ROS2_WS
rosdep update
rosdep install -r --from-paths src -i -y --rosdistro humble

# Source the install setup file (will work later after full build too)
if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source $ROS2_WS/install/setup.bash
  echo "✅ Sourced workspace: $ROS2_WS/install/setup.bash"
else
  echo "⚠️  setup.bash not found yet (will be available after colcon build)"
fi

# Verify submodule is properly loaded
if [ -d "$ROS2_SRC/ros2_agent_sim/unitree_go2_ros2" ]; then
    echo "✅ Unitree Go2 submodule found"
    # Check if submodule has content (not empty)
    if [ "$(ls -A $ROS2_SRC/ros2_agent_sim/unitree_go2_ros2)" ]; then
        echo "✅ Unitree Go2 submodule has content"
    else
        echo "⚠️  Unitree Go2 submodule is empty, updating..."
        cd $ROS2_SRC/ros2_agent_sim
        git submodule update --init --recursive --force
    fi
else
    echo "❌ Unitree Go2 submodule NOT found - checking submodule setup..."
    cd $ROS2_SRC/ros2_agent_sim
    git submodule status
    echo "Attempting to fix submodule..."
    git submodule update --init --recursive --force
fi

# Ensure submodules are on correct branches (not detached HEAD)
echo "⏳ Ensuring submodules are on correct branches..."
cd $ROS2_SRC/ros2_agent_sim

# gz_ros2_control humble branch
if [ -d "gz_ros2_control" ]; then
    echo "Checking out gz_ros2_control to humble branch..."
    cd gz_ros2_control
    git checkout humble
    git pull origin humble
    cd ..
    echo "✅ gz_ros2_control is now on humble branch"
else
    echo "⚠️  gz_ros2_control directory not found"
fi

# Install dependencies for gz_ros2_control submodule
echo "⏳ Installing dependencies for gz_ros2_control submodule..."
if [ -d "$ROS2_SRC/ros2_agent_sim/gz_ros2_control" ]; then
    cd $ROS2_SRC/ros2_agent_sim/gz_ros2_control
    rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
    echo "✅ gz_ros2_control dependencies installed"
else
    echo "⚠️  gz_ros2_control submodule directory not found"
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

# Install MAVROS packages from apt (with updated GPG key)
echo "Installing MAVROS packages from apt..." && sleep 1
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-msgs

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

cd $ROS2_WS && colcon build --allow-overriding gz_ros2_control gz_ros2_control_demos

echo "DONE. Pkgs are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"

# Add Python local bin to PATH and make it available in this session
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# Installing Python dependencies - use --no-warn-script-location to suppress warnings
echo "Installing Python dependencies..."

# Install numpy at the required version to satisfy all dependencies
pip3 install --no-warn-script-location numpy==1.26.4

# Install rospkg which is needed for rosinstall-generator
pip3 install --no-warn-script-location rospkg

# Install core packages
pip3 install --no-warn-script-location \
    rich \
    langchain \
    langchain-ollama \
    langchain-community==0.3.21 \
    opencv-python \
    PyYAML==6.0.1

# Check if ollama is installed
if ! command_exists ollama; then
    echo "Installing Ollama..."
    curl -fsSL https://ollama.ai/install.sh | sh
    
    # Start ollama service
    systemctl start ollama || echo "Failed to start ollama service, you may need to start it manually"
    systemctl enable ollama || echo "Failed to enable ollama service, you may need to enable it manually"
else
    echo "Ollama is already installed"
fi

# Pull Qwen3: 8b model
echo "Pulling Qwen3:8b model for LLM..."
if command_exists ollama; then
    ollama pull qwen3:8b || echo "Failed to pull Qwen3:8b model, you may need to pull it manually"
else
    echo "Ollama not available, skipping model pull"
fi

echo "Installation complete!"
echo "Next steps:"
echo "Installation successful!"

echo "DONE. Pkgs are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"

# Source the ws
source $ROS2_WS/install/setup.bash
# Add to .bashrc for future sessions
echo "source $ROS2_WS/install/setup.bash" >> ~/.bashrc

# Final message
echo "INSTALLATION COMPLETE! Packages are built successfully."
echo "Models and airframe config files are copied to the respective folders in the ${PX4_DIR} directory"
echo "Next steps:"
echo "1. Source the workspace:  source $ROS2_WS/install/setup.bash"
echo "2. Launch the simulation: ros2 launch drone_sim drone.launch.py"
echo "3. Run the ROS2 agent:    ros2 run ros2_agent ros2_agent_node"
cd $HOME