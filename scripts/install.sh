#!/bin/bash -e

# This script sets up the ROS2 Agent simulation environment for ROS2 Jazzy + Gazebo Harmonic

# Run rosdep update and install for Jazzy
# echo "⏳ Running rosdep update and install for ROS2 Jazzy..."
# cd $ROS2_WS
# rosdep update
#rosdep install --from-paths src --ignore-src -r -y

# # Function to check if a command exists for Ollama
# command_exists() {
#     command -v "$1" >/dev/null 2>&1
# }

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

# # Install ROS2 Jazzy packages 
echo "⏳ Installing ROS2 Jazzy packages..."

# Install core ROS2 Jazzy packages
echo "📦 Installing ROS2 Jazzy core packages..."

sudo apt install -y \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-xacro \
    ros-jazzy-robot-localization \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-velodyne \
    ros-jazzy-velodyne-description \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-eigen3-cmake-module

echo "✅ ROS2 Jazzy package installation step completed"


# Clone and build PX4-Autopilot from AbdullahGM1's fork using 'navsat_callback' branch
if [ ! -d "$PX4_DIR" ]; then
    cd "$DEV_DIR"
    git clone --recursive https://github.com/AbdullahGM1/PX4-Autopilot.git
fi

cd "$PX4_DIR"
git checkout navsat_callback
make submodulesclean
make distclean
make clean

# Build px4_sitl with error handling
echo "🔨 Building PX4 SITL..."
cd "$PX4_DIR"
if make px4_sitl; then
    echo "✅ PX4 SITL build succeeded"
else
    echo "❌ PX4 SITL build failed"
fi
    
# Copy files to $PX4_DIR
echo && echo "📁 Copying configuration files to ${PX4_DIR}" && echo
sleep 1

# Ensure target directories exist
mkdir -p ${PX4_DIR}/Tools/simulation/gz/models/
mkdir -p ${PX4_DIR}/Tools/simulation/gz/worlds/
mkdir -p ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Copy files with error checking

# Copy the models

if [ -d "${PX4_config}/models" ]; then
    cp -r ${PX4_config}/models/* ${PX4_DIR}/Tools/simulation/gz/models/
    echo "✅ Models copied successfully"
else
    echo "⚠️  Models directory not found in PX4_config"
fi

# Copy the worlds

if [ -d "${PX4_config}/worlds" ]; then
    cp -r ${PX4_config}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/
    echo "✅ Worlds copied successfully"
else
    echo "⚠️  Worlds directory not found in PX4_config"
fi

# Copy the ariframes

if [ -d "${PX4_config}/px4" ]; then
    cp -r ${PX4_config}/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/
    echo "✅ PX4 airframe configs copied successfully"
else
    echo "⚠️  PX4 configs directory not found in PX4_config"
fi

 # Re-Build px4_sitl with error handling

echo "🔨 Re-Building PX4 SITL..."
cd "$PX4_DIR"
if make px4_sitl; then
    echo "✅ PX4 SITL build succeeded"
else
    echo "❌ PX4 SITL build failed"
fi

# # Install MAVROS packages from apt for Jazzy
# echo "Installing ROS2 Jazzy MAVROS packages from apt..." && sleep 1
# sudo apt update
# sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs

# #
# # MAVROS - Custom packages for multi-vehicle support
# #
# echo "Cloning mavlink package for Jazzy... " && sleep 1
# if [ ! -d "$ROS2_SRC/mavlink" ]; then
#     cd $ROS2_SRC
#     git clone https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
#     cd $ROS2_SRC/mavlink 
#     # Try to find a jazzy compatible branch, fallback to latest
#     if git branch -r | grep -q "release/jazzy"; then
#         git checkout release/jazzy/mavlink/2024.7.8-1 || git checkout $(git branch -r | grep "release/jazzy" | head -1 | sed 's/origin\///')
#     else
#         echo "⚠️  No Jazzy-specific mavlink branch found, using latest release"
#         git checkout $(git tag | grep -E "^2024\." | sort -V | tail -1) || git checkout $(git branch -r | grep "release" | tail -1 | sed 's/origin\///')
#     fi
# fi

# # Custom mavros pkg is required to handle TF issues in multi-vehicle simulation
# echo "Cloning custom mavros package for Jazzy... " && sleep 1
# if [ ! -d "$ROS2_SRC/mavros" ]; then
#     cd $ROS2_SRC
#     git clone https://github.com/AbdullahGM1/mavros.git
#     cd $ROS2_SRC/mavros 
#     # Check if there's a jazzy branch, otherwise use ros2_humble branch
#     if git branch -r | grep -q "ros2_jazzy"; then
#         git checkout ros2_jazzy
#         echo "✅ Using ros2_jazzy branch"
#     else
#         echo "⚠️  No ros2_jazzy branch found, using ros2_humble branch"
#         echo "⚠️  This may require additional dependencies for Jazzy compatibility"
#         git checkout ros2_humble
        
#         # Install additional dependencies that might be missing for Jazzy
#         sudo apt install -y \
#             ros-jazzy-tf2-eigen \
#             ros-jazzy-tf2-geometry-msgs \
#             ros-jazzy-eigen3-cmake-module \
#             ros-jazzy-geographic-msgs \
#             ros-jazzy-sensor-msgs \
#             ros-jazzy-geometry-msgs \
#             ros-jazzy-std-srvs
#     fi
# fi

# # Initialize and update rosdep for Jazzy
# cd $ROS2_WS 
# rosdep init || echo "rosdep already initialized"
# rosdep update 
# rosdep install --from-paths src --ignore-src -r -y

# # Build MAVROS packages with Jazzy (with error handling)
# echo "🔨 Building MAVROS packages for Jazzy..."
# cd $ROS2_WS 

# # Try building mavros with better error handling
# if MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros --executor sequential; then
#     echo "✅ MAVROS built successfully"
    
#     # Build mavros_extras
#     if MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential; then
#         echo "✅ MAVROS extras built successfully"
#     else
#         echo "⚠️  MAVROS extras build failed, but core MAVROS succeeded"
#     fi
# else
#     echo "❌ Custom MAVROS build failed. Trying fallback options..."
    
#     # Option 1: Remove custom mavros and use system packages
#     echo "🔄 Removing custom MAVROS and using system packages..."
#     rm -rf $ROS2_SRC/mavros $ROS2_SRC/mavlink
    
#     # Install system MAVROS packages (already installed earlier, but ensure they're there)
#     sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-mavros-extras
    
#     echo "✅ Using system MAVROS packages as fallback"
# fi

# # Set Gazebo version for Harmonic compatibility
# export GZ_VERSION=harmonic
# echo "✅ GZ_VERSION set to 'harmonic' for Gazebo Harmonic compatibility"

# # Build remaining packages
# cd $ROS2_WS && colcon build

# echo "DONE. Packages are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"

# # Add Python local bin to PATH and make it available in this session
# export PATH="$HOME/.local/bin:$PATH"
# echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# # Installing Python dependencies - use --break-system-packages for Ubuntu 24.04
# echo "Installing Python dependencies..."

# # Install numpy at the required version to satisfy all dependencies
# pip3 install --break-system-packages --no-warn-script-location numpy==1.26.4

# # Install rospkg which is needed for rosinstall-generator
# pip3 install --break-system-packages --no-warn-script-location rospkg

# # Install core packages
# pip3 install --break-system-packages --no-warn-script-location \
#     rich \
#     langchain \
#     langchain-ollama \
#     langchain-community==0.3.21 \
#     opencv-python \
#     PyYAML==6.0.1 \
#     rosa

# # Check if ollama is installed
# if ! command_exists ollama; then
#     echo "Installing Ollama..."
#     curl -fsSL https://ollama.ai/install.sh | sh
    
#     # Start ollama service
#     systemctl start ollama || echo "Failed to start ollama service, you may need to start it manually"
#     systemctl enable ollama || echo "Failed to enable ollama service, you may need to enable it manually"
# else
#     echo "Ollama is already installed"
# fi

# # Pull Qwen3:8b model
# echo "Pulling Qwen3:8b model for LLM..."
# if command_exists ollama; then
#     ollama pull qwen3:8b || echo "Failed to pull Qwen3:8b model, you may need to pull it manually"
# else
#     echo "Ollama not available, skipping model pull"
# fi

# echo "Installation complete!"

# # Source the workspace
# source $ROS2_WS/install/setup.bash
# # Add to .bashrc for future sessions
# echo "source $ROS2_WS/install/setup.bash" >> ~/.bashrc

# # Final message
# echo "INSTALLATION COMPLETE! Packages are built successfully for ROS2 Jazzy + Gazebo Harmonic."
# echo "Models and airframe config files are copied to the respective folders in the ${PX4_DIR} directory"
# echo "Next steps:"
# echo "1. Source the workspace:  source $ROS2_WS/install/setup.bash"
# echo "2. Launch the simulation: ros2 launch drone_sim drone.launch.py"
# echo "3. Run the ROS2 agent:    ros2 run ros2_agent ros2_agent_node"
# cd $HOME