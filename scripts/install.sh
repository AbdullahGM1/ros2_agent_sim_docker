#!/bin/bash -e

# This script sets up the ROS2 Agent simulation environment for ROS2 Jazzy + Gazebo Harmonic

# Function to check if a command exists for Ollama
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to clean up deprecated apt-key entries
cleanup_deprecated_keys() {
    echo "Cleaning up any deprecated apt-key entries..."
    
    # Remove any old ROS keys that might be causing warnings
    sudo apt-key del F42ED6FBAB17C654 2>/dev/null || true
    sudo apt-key del C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 2>/dev/null || true
    
    # Clean up apt-key list
    sudo apt-key list 2>/dev/null | grep -i "warning" && echo "⚠️  Some deprecated keys still exist" || echo "✅ No deprecated key warnings"
}

# Function to setup ROS2 repository
setup_ros2_repository() {
    echo "Setting up ROS2 repository with modern keyring approach..."
    
    # Remove old keyring files if they exist
    sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || true
    
    # Download and add the ROS2 GPG key using the modern keyring approach
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Update the sources list to use the new keyring
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    echo "✅ ROS2 repository configured with modern keyring approach"
}

# Function to setup Gazebo repository
setup_gazebo_repository() {
    echo "Setting up Gazebo repository with modern keyring approach..."
    
    # Remove old Gazebo keyring files if they exist
    sudo rm -f /usr/share/keyrings/gazebo-keyring.gpg 2>/dev/null || true
    
    # Add Gazebo GPG key using modern keyring approach
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo gpg --dearmor -o /usr/share/keyrings/gazebo-keyring.gpg
    
    # Add Gazebo repository with keyring
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    echo "✅ Gazebo repository configured with modern keyring approach"
}

if [ -z "${DEV_DIR}" ]; then
  echo "Error: DEV_DIR environment variable is not set. Set it using export DEV_DIR=<DEV_DIR_directory_that_should_contain_PX4-Autopilot_and_ros2_ws>"
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

# Clean up any deprecated apt-key entries first
cleanup_deprecated_keys

# Setup ROS2 repository before any apt operations
setup_ros2_repository

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

# Install ONLY verified ROS2 Jazzy packages (NO PROBLEMATIC GAZEBO PACKAGES)
echo "⏳ Installing verified ROS2 Jazzy packages..."
sudo apt update

# Install packages ONE BY ONE to catch any issues
declare -a packages=(
    "ros-jazzy-gz-ros2-control"
    "ros-jazzy-gz-ros2-control-demos" 
    "ros-jazzy-xacro"
    "ros-jazzy-robot-localization"
    "ros-jazzy-ros2-controllers"
    "ros-jazzy-ros2-control"
    "ros-jazzy-velodyne"
    "ros-jazzy-velodyne-description"
    "ros-jazzy-tf2-eigen"
    "ros-jazzy-tf2-geometry-msgs"
    "ros-jazzy-eigen3-cmake-module"
)

for pkg in "${packages[@]}"; do
    echo "📦 Installing $pkg..."
    if sudo apt install -y "$pkg"; then
        echo "✅ $pkg installed successfully"
    else
        echo "❌ Failed to install $pkg, continuing..."
    fi
done

echo "✅ Core ROS2 Jazzy packages installation completed"

# Additional required packages for Jazzy + Harmonic compatibility  
echo "⏳ Installing Gazebo Harmonic physics libraries..."
PHYSICS_PACKAGES=(
  "libgz-physics8-dev"
  "libgz-physics8-dartsim"
  "libgz-physics8"
)

for pkg in "${PHYSICS_PACKAGES[@]}"; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    echo "✅ $pkg is already installed"
  else
    echo "⏳ Installing $pkg..."
    if sudo apt install -y "$pkg"; then
        echo "✅ $pkg installed successfully"
    else
        echo "❌ Failed to install $pkg, continuing..."
    fi
  fi
done

# Run rosdep update and install for Jazzy
echo "⏳ Running rosdep update and install for ROS2 Jazzy..."
cd $ROS2_WS
rosdep update

# Fix gz_sim rosdep issue for Unitree packages
echo "🔧 Fixing gz_sim rosdep definitions for Unitree packages..."
if [ -d "$ROS2_SRC/ros2_agent_sim/unitree_go2_ros2" ]; then
    echo "📝 Removing problematic gz_sim dependencies from Unitree packages..."
    
    # REMOVE gz_sim dependencies entirely to avoid rosdep issues
    find $ROS2_SRC/ros2_agent_sim/unitree_go2_ros2 -name "package.xml" -exec sed -i '/<.*depend>gz_sim<\/.*depend>/d' {} \;
    find $ROS2_SRC/ros2_agent_sim/unitree_go2_ros2 -name "package.xml" -exec sed -i '/<.*depend>gz-sim<\/.*depend>/d' {} \;
    
    echo "✅ Removed gz_sim dependencies from Unitree packages (will use system Gazebo)"
    echo "📋 Modified package.xml files:"
    find $ROS2_SRC/ros2_agent_sim/unitree_go2_ros2 -name "package.xml" -exec echo "   - {}" \;
else
    echo "⚠️  Unitree Go2 packages not found, skipping rosdep fix"
fi

# Install dependencies with error handling and debugging
echo "⏳ Running rosdep install with detailed output..."
rosdep install -r --from-paths src -i -y --rosdistro jazzy --verbose || {
    echo "❌ Rosdep install failed. Checking individual packages..."
    echo "📋 Checking each package for dependency issues:"
    
    for pkg_dir in src/*/; do
        if [ -d "$pkg_dir" ]; then
            pkg_name=$(basename "$pkg_dir")
            echo "🔍 Checking package: $pkg_name"
            rosdep check --from-paths "$pkg_dir" --ignore-src --rosdistro jazzy || {
                echo "❌ $pkg_name has dependency issues"
                echo "📄 package.xml contents for $pkg_name:"
                find "$pkg_dir" -name "package.xml" -exec cat {} \; | grep -E "<.*depend>" || echo "No dependencies found"
            }
        fi
    done
    
    echo "⚠️  Continuing with partial rosdep installation..."
}

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

# Clone and build PX4-Autopilot from AbdullahGM1's fork using main branch
if [ -d "$PX4_DIR" ]; then
    echo "⚠️  PX4_DIR=$PX4_DIR already exists. Removing for clean installation..."
    rm -rf "$PX4_DIR"
fi

echo "🔄 Cloning fresh PX4-Autopilot from AbdullahGM1/PX4-Autopilot (main branch)..."
cd $DEV_DIR
git clone https://github.com/AbdullahGM1/PX4-Autopilot.git --recursive --depth=1
cd $PX4_DIR

# Verify we're on the main branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [[ "$CURRENT_BRANCH" != "main" ]]; then
    echo "🔄 Switching to main branch..."
    git checkout main
    git submodule update --init --recursive
fi

echo "✅ Fresh PX4-Autopilot clone completed"

# Verify PX4 version and branch
PX4_BRANCH=$(cd $PX4_DIR && git rev-parse --abbrev-ref HEAD)
PX4_COMMIT=$(cd $PX4_DIR && git rev-parse --short HEAD)
PX4_REMOTE=$(cd $PX4_DIR && git remote get-url origin)
echo "✅ PX4 Repository: $PX4_REMOTE"
echo "✅ PX4 Branch: $PX4_BRANCH"  
echo "✅ PX4 Commit: $PX4_COMMIT"

# Clean any previous build artifacts
echo "🧹 Cleaning previous build artifacts..."
cd $PX4_DIR 
rm -rf build/
make distclean || echo "⚠️  make distclean failed (this is okay for fresh clone)"

# Build px4_sitl with error handling
echo "🔨 Building PX4 SITL..."
cd $PX4_DIR 
if ! make px4_sitl; then
    echo "❌ PX4 SITL build failed. Trying alternative build approach..."
    
    # Try clearing cmake cache and rebuilding
    rm -rf build/
    mkdir -p build/px4_sitl_default
    cd build/px4_sitl_default
    
    if cmake ../.. -DCONFIG=px4_sitl_default; then
        if make; then
            echo "✅ PX4 SITL build succeeded with manual cmake"
        else
            echo "❌ PX4 SITL build failed even with manual cmake"
            echo "⚠️  Continuing with installation, but PX4 may not work properly"
        fi
    else
        echo "❌ CMake configuration failed"
        echo "⚠️  Continuing with installation, but PX4 may not work properly"
    fi
    cd $PX4_DIR
else
    echo "✅ PX4 SITL build succeeded"
fi

# Copy files to $PX4_DIR
echo && echo "📁 Copying configuration files to ${PX4_DIR}" && echo
sleep 1

# Ensure target directories exist
mkdir -p ${PX4_DIR}/Tools/simulation/gz/models/
mkdir -p ${PX4_DIR}/Tools/simulation/gz/worlds/
mkdir -p ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Copy files with error checking
if [ -d "${PX4_config}/models" ]; then
    cp -r ${PX4_config}/models/* ${PX4_DIR}/Tools/simulation/gz/models/
    echo "✅ Models copied successfully"
else
    echo "⚠️  Models directory not found in PX4_config"
fi

if [ -d "${PX4_config}/worlds" ]; then
    cp -r ${PX4_config}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/
    echo "✅ Worlds copied successfully"
else
    echo "⚠️  Worlds directory not found in PX4_config"
fi

if [ -d "${PX4_config}/px4" ]; then
    cp -r ${PX4_config}/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/
    echo "✅ PX4 airframe configs copied successfully"
else
    echo "⚠️  PX4 configs directory not found in PX4_config"
fi

# Try building again after copying configs
echo "🔨 Final PX4 SITL build with new configurations..."
cd $PX4_DIR 
make px4_sitl || echo "⚠️  Final PX4 build had issues, but continuing..."

# Install MAVROS packages from apt for Jazzy
echo "Installing ROS2 Jazzy MAVROS packages from apt..." && sleep 1
sudo apt update
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs

#
# MAVROS - Custom packages for multi-vehicle support
#
echo "Cloning mavlink package for Jazzy... " && sleep 1
if [ ! -d "$ROS2_SRC/mavlink" ]; then
    cd $ROS2_SRC
    git clone https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
    cd $ROS2_SRC/mavlink 
    # Try to find a jazzy compatible branch, fallback to latest
    if git branch -r | grep -q "release/jazzy"; then
        git checkout release/jazzy/mavlink/2024.7.8-1 || git checkout $(git branch -r | grep "release/jazzy" | head -1 | sed 's/origin\///')
    else
        echo "⚠️  No Jazzy-specific mavlink branch found, using latest release"
        git checkout $(git tag | grep -E "^2024\." | sort -V | tail -1) || git checkout $(git branch -r | grep "release" | tail -1 | sed 's/origin\///')
    fi
fi

# Custom mavros pkg is required to handle TF issues in multi-vehicle simulation
echo "Cloning custom mavros package for Jazzy... " && sleep 1
if [ ! -d "$ROS2_SRC/mavros" ]; then
    cd $ROS2_SRC
    git clone https://github.com/AbdullahGM1/mavros.git
    cd $ROS2_SRC/mavros 
    # Check if there's a jazzy branch, otherwise use ros2_humble branch
    if git branch -r | grep -q "ros2_jazzy"; then
        git checkout ros2_jazzy
        echo "✅ Using ros2_jazzy branch"
    else
        echo "⚠️  No ros2_jazzy branch found, using ros2_humble branch"
        echo "⚠️  This may require additional dependencies for Jazzy compatibility"
        git checkout ros2_humble
        
        # Install additional dependencies that might be missing for Jazzy
        sudo apt install -y \
            ros-jazzy-tf2-eigen \
            ros-jazzy-tf2-geometry-msgs \
            ros-jazzy-eigen3-cmake-module \
            ros-jazzy-geographic-msgs \
            ros-jazzy-sensor-msgs \
            ros-jazzy-geometry-msgs \
            ros-jazzy-std-srvs
    fi
fi

# Initialize and update rosdep for Jazzy
cd $ROS2_WS 
rosdep init || echo "rosdep already initialized"
rosdep update 
rosdep install --from-paths src --ignore-src -r -y

# Build MAVROS packages with Jazzy (with error handling)
echo "🔨 Building MAVROS packages for Jazzy..."
cd $ROS2_WS 

# Try building mavros with better error handling
if MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros --executor sequential; then
    echo "✅ MAVROS built successfully"
    
    # Build mavros_extras
    if MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential; then
        echo "✅ MAVROS extras built successfully"
    else
        echo "⚠️  MAVROS extras build failed, but core MAVROS succeeded"
    fi
else
    echo "❌ Custom MAVROS build failed. Trying fallback options..."
    
    # Option 1: Remove custom mavros and use system packages
    echo "🔄 Removing custom MAVROS and using system packages..."
    rm -rf $ROS2_SRC/mavros $ROS2_SRC/mavlink
    
    # Install system MAVROS packages (already installed earlier, but ensure they're there)
    sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-mavros-extras
    
    echo "✅ Using system MAVROS packages as fallback"
fi

# Set Gazebo version for Harmonic compatibility
export GZ_VERSION=harmonic
echo "✅ GZ_VERSION set to 'harmonic' for Gazebo Harmonic compatibility"

# Build remaining packages
cd $ROS2_WS && colcon build

echo "DONE. Packages are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"

# Add Python local bin to PATH and make it available in this session
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# Installing Python dependencies - use --break-system-packages for Ubuntu 24.04
echo "Installing Python dependencies..."

# Install numpy at the required version to satisfy all dependencies
pip3 install --break-system-packages --no-warn-script-location numpy==1.26.4

# Install rospkg which is needed for rosinstall-generator
pip3 install --break-system-packages --no-warn-script-location rospkg

# Install core packages
pip3 install --break-system-packages --no-warn-script-location \
    rich \
    langchain \
    langchain-ollama \
    langchain-community==0.3.21 \
    opencv-python \
    PyYAML==6.0.1 \
    rosa

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

# Pull Qwen3:8b model
echo "Pulling Qwen3:8b model for LLM..."
if command_exists ollama; then
    ollama pull qwen3:8b || echo "Failed to pull Qwen3:8b model, you may need to pull it manually"
else
    echo "Ollama not available, skipping model pull"
fi

echo "Installation complete!"

# Source the workspace
source $ROS2_WS/install/setup.bash
# Add to .bashrc for future sessions
echo "source $ROS2_WS/install/setup.bash" >> ~/.bashrc

# Final message
echo "INSTALLATION COMPLETE! Packages are built successfully for ROS2 Jazzy + Gazebo Harmonic."
echo "Models and airframe config files are copied to the respective folders in the ${PX4_DIR} directory"
echo "Next steps:"
echo "1. Source the workspace:  source $ROS2_WS/install/setup.bash"
echo "2. Launch the simulation: ros2 launch drone_sim drone.launch.py"
echo "3. Run the ROS2 agent:    ros2 run ros2_agent ros2_agent_node"
cd $HOME