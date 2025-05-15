#!/bin/bash -e

# This script sets up the Ros2 Agent simulation environment

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to display progress message
progress() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Function to display success message
success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# Function to display warning message
warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Function to display error message
error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running in a container
in_container() {
    [ -f /.dockerenv ] || grep -q '/docker/' /proc/1/cgroup || grep -q '/docker/' /proc/self/cgroup
}

# Check environment variables
if [ -z "${DEV_DIR}" ]; then
  error "DEV_DIR environment variable is not set. Set it using export DEV_DIR=<DEV_DIR_directory_that_should_contain_PX4-Autopilot_and_ros2_ws>"
  exit 1
fi
progress "Using DEV_DIR=$DEV_DIR"
sleep 1

# Hide credentials from console output
if [ -n "$GIT_USER" ] && [ -n "$GIT_TOKEN" ]; then
    progress "GIT_USER and GIT_TOKEN are set"
else 
    warning "GIT_USER and/or GIT_TOKEN are not set (not required for this script)"
fi
sleep 1

# Define directory paths
ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src
PX4_DIR=$DEV_DIR/PX4-Autopilot
PX4_config=$DEV_DIR/PX4_config
OSQP_SRC=$DEV_DIR

# Create workspace directories if they don't exist
if [ ! -d "$ROS2_WS" ]; then
  progress "Creating $ROS2_SRC"
  mkdir -p $ROS2_SRC
fi

# Clone the ros2_agent_sim repository if it doesn't exist
progress "Setting up ros2_agent_sim repository"
ROS2_AGENT_SIM_URL=https://github.com/AbdullahGM1/ros2_agent_sim.git

# Clone the ros2_agent_sim if it doesn't exist
if [ ! -d "$ROS2_SRC/ros2_agent_sim" ]; then
    cd $ROS2_SRC
    progress "Cloning ros2_agent_sim..."
    git clone $ROS2_AGENT_SIM_URL ros2_agent_sim && cd $ROS2_SRC/ros2_agent_sim && git pull origin main
    success "ros2_agent_sim cloned successfully"
else
    progress "ros2_agent_sim already exists, updating..."
    cd $ROS2_SRC/ros2_agent_sim && git pull origin main
    success "ros2_agent_sim updated successfully"
fi

# Clone and build PX4-Autopilot if it doesn't exist
progress "Setting up PX4-Autopilot"
if [ ! -d "$PX4_DIR" ]; then
    progress "Cloning $PX4_DIR..."
    cd $DEV_DIR
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd $PX4_DIR
    git fetch --all --tags
    git checkout -f v1.14.0  # Force checkout
    git reset --hard v1.14.0  # Reset any changes
    git submodule update --init --recursive  # Update submodules to match
    make distclean
    success "PX4-Autopilot cloned successfully"
else
    progress "PX4_DIR=$PX4_DIR already exists, updating to v1.14.0..."
    cd $PX4_DIR
    git fetch --all --tags
    git checkout -f v1.14.0  # Force checkout
    git reset --hard v1.14.0  # Reset any changes
    git submodule update --init --recursive  # Update submodules to match
    make distclean
    success "PX4-Autopilot updated to v1.14.0"
fi

# Verify PX4 version
PX4_VERSION=$(cd $PX4_DIR && git describe --tags)
progress "PX4 version: $PX4_VERSION"
if [[ "$PX4_VERSION" != "v1.14.0" ]]; then
    warning "PX4 version is not v1.14.0. Got $PX4_VERSION instead."
    warning "This may cause compatibility issues."
fi

# Build px4_sitl
progress "Building PX4 SITL..."
cd $PX4_DIR && make px4_sitl
success "PX4 SITL built successfully"

# Copy files to $PX4_DIR
progress "Copying configuration files to ${PX4_DIR}"
sleep 1
cp -r ${PX4_config}/models/* ${PX4_DIR}/Tools/simulation/gz/models/
cp -r ${PX4_config}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/
cp -r ${PX4_config}/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/
cd $PX4_DIR && make px4_sitl
success "Configuration files copied successfully"

# Install MAVROS packages from apt
progress "Installing MAVROS packages from apt..."
sleep 1
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-msgs
success "MAVROS packages installed"

# MAVROS setup
progress "Setting up MAVROS and related packages..."
progress "Cloning mavlink package..."
if [ ! -d "$ROS2_SRC/mavlink" ]; then
    cd $ROS2_SRC
    git clone https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
    cd $ROS2_SRC/mavlink && git checkout release/humble/mavlink/2023.9.9-1
    success "mavlink package cloned successfully"
else
    progress "mavlink package already exists"
    cd $ROS2_SRC/mavlink && git checkout release/humble/mavlink/2023.9.9-1
fi

# Custom mavros pkg is required to handle TF issues in multi-vehicle simulation
progress "Cloning custom mavros package..."
if [ ! -d "$ROS2_SRC/mavros" ]; then
    cd $ROS2_SRC
    git clone https://github.com/AbdullahGM1/mavros.git
    cd $ROS2_SRC/mavros
    
    # Try to find the correct branch
    progress "Checking available branches in mavros repository"
    git fetch --all
    
    # List of possible branch names to try
    BRANCH_OPTIONS=("ros2" "humble" "main" "master")
    
    # Try to checkout the first available branch from our options
    BRANCH_FOUND=false
    for branch in "${BRANCH_OPTIONS[@]}"; do
        if git ls-remote --heads origin | grep -q "refs/heads/$branch"; then
            progress "Found branch: $branch"
            git checkout $branch
            BRANCH_FOUND=true
            break
        fi
    done
    
    if [ "$BRANCH_FOUND" = false ]; then
        warning "None of the expected branches found. Using default branch."
    fi
    
    success "Custom mavros package cloned successfully"
else
    progress "Custom mavros package already exists"
    cd $ROS2_SRC/mavros
    git fetch --all
    
    # Try to find the correct branch
    BRANCH_OPTIONS=("ros2" "humble" "main" "master")
    BRANCH_FOUND=false
    for branch in "${BRANCH_OPTIONS[@]}"; do
        if git ls-remote --heads origin | grep -q "refs/heads/$branch"; then
            progress "Found branch: $branch"
            git checkout $branch
            git pull origin $branch
            BRANCH_FOUND=true
            break
        fi
    done
    
    if [ "$BRANCH_FOUND" = false ]; then
        warning "None of the expected branches found. Using current branch."
    fi
fi

# Add ~/.local/bin to PATH if not already there
if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
    progress "Adding ~/.local/bin to PATH"
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    export PATH="$HOME/.local/bin:$PATH"
fi

# Installing Python dependencies with specific versions to resolve conflicts
progress "Installing Python dependencies..."
pip3 install \
    rich \
    rospkg \
    PyYAML==6.0.1 \
    numpy==1.26.4 \
    langchain-community==0.3.21 \
    langchain \
    langchain-ollama \
    opencv-python \
    --force-reinstall

success "Python dependencies installed"

# Check if ollama is installed
if ! command_exists ollama; then
    progress "Installing Ollama..."
    curl -fsSL https://ollama.ai/install.sh | sh
    
    # Start ollama service (check if in container first)
    if ! in_container; then
        progress "Starting Ollama service..."
        systemctl start ollama || warning "Failed to start ollama service, you may need to start it manually"
        systemctl enable ollama || warning "Failed to enable ollama service, you may need to enable it manually"
    else
        warning "Running in container - systemctl commands skipped"
        warning "You may need to start Ollama manually"
    fi
    success "Ollama installed"
else
    success "Ollama is already installed"
fi

# Pull Qwen3: 8b model
progress "Pulling Qwen3:8b model for LLM..."
if command_exists ollama; then
    ollama pull qwen3:8b || warning "Failed to pull Qwen3:8b model, you may need to pull it manually"
    success "Qwen3:8b model pulled successfully"
else
    warning "Ollama not available, skipping model pull"
fi

# Setup ROS workspace
progress "Setting up ROS2 workspace..."
cd $ROS2_WS && rosdep init || true  # Won't fail if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
success "ROS2 dependencies installed"

progress "Building ROS2 workspace (this may take a while)..."
cd $ROS2_WS && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros --executor sequential
success "mavros package built"

cd $ROS2_WS && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential
success "mavros_extras package built"

cd $ROS2_WS && colcon build
success "All ROS2 packages built successfully"

# Final message
success "INSTALLATION COMPLETE! Packages are built successfully."
success "Models and airframe config files are copied to the respective folders in the ${PX4_DIR} directory"
echo -e "${YELLOW}Next steps:${NC}"
echo -e "1. Source your updated PATH:  ${GREEN}source ~/.bashrc${NC}"
echo -e "2. Source the workspace:      ${GREEN}source $ROS2_WS/install/setup.bash${NC}"
echo -e "3. Launch the simulation:     ${GREEN}ros2 launch ros2_agent_sim drone.launch.py${NC}"
echo -e "4. Run the ROS2 agent:        ${GREEN}ros2 run ros2_agent ros2_agent_node${NC}"
cd $HOME

##############################