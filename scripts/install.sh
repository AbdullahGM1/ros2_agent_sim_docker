#!/bin/bash -e

# ============================================================================
# ROS2 Jazzy + Gazebo Harmonic + PX4 + MAVROS + ROSA Installation Script
# Enhanced with error handling, logging, and progress tracking
# ============================================================================

# Colors and formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Logging setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="${SCRIPT_DIR}/install_$(date +%Y%m%d_%H%M%S).log"
STEP_COUNTER=0
TOTAL_STEPS=12

# Enhanced logging functions
log() {
    echo -e "$1" | tee -a "$LOG_FILE"
}

print_header() {
    echo -e "\n${PURPLE}═══════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}${CYAN}$1${NC}"
    echo -e "${PURPLE}═══════════════════════════════════════════════════════════════════════════════${NC}\n"
}

print_step() {
    STEP_COUNTER=$((STEP_COUNTER + 1))
    echo -e "\n${BLUE}[${STEP_COUNTER}/${TOTAL_STEPS}]${NC} ${BOLD}$1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_success() { log "${GREEN}✅ $1${NC}"; }
print_warning() { log "${YELLOW}⚠️  $1${NC}"; }
print_error() { log "${RED}❌ $1${NC}"; }
print_info() { log "${CYAN}ℹ️  $1${NC}"; }

# Enhanced error handling
cleanup_on_error() {
    print_error "Installation failed at step ${STEP_COUNTER}/${TOTAL_STEPS}"
    print_info "Check log file: ${LOG_FILE}"
    print_info "Cleaning up partial installations..."
    
    # Optional: Add cleanup commands here
    # rm -rf partially_installed_directories
    
    exit 1
}

trap cleanup_on_error ERR

# Performance tracking
start_time=$(date +%s)
track_time() {
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    print_info "⏱️  Step completed in ${duration}s"
    start_time=$(date +%s)
}

# System requirements check
check_system_requirements() {
    print_step "Checking System Requirements"
    
    # Check available disk space (minimum 20GB)
    local available_space=$(df /home | tail -1 | awk '{print $4}')
    if [ "$available_space" -lt 20971520 ]; then
        print_error "Insufficient disk space. Need at least 20GB free."
        exit 1
    fi
    
    # Check RAM (minimum 8GB recommended)
    local ram_gb=$(free -g | grep "Mem:" | awk '{print $2}')
    if [ "$ram_gb" -lt 8 ]; then
        print_warning "Less than 8GB RAM detected. Build process may be slow."
    fi
    
    # Check Ubuntu version
    local ubuntu_version=$(lsb_release -rs)
    if [[ "$ubuntu_version" != "24.04" ]]; then
        print_warning "Optimized for Ubuntu 24.04. Current: $ubuntu_version"
    fi
    
    print_success "System requirements check completed"
    track_time
}

# Environment validation
validate_environment() {
    print_step "Validating Environment Variables"
    
    if [ -z "${DEV_DIR}" ]; then
        print_error "DEV_DIR environment variable is not set"
        print_info "Set it using: export DEV_DIR=/path/to/your/dev/directory"
        exit 1
    fi
    
    print_info "DEV_DIR: ${DEV_DIR}"
    print_info "GIT_USER: ${GIT_USER:-'Not set'}"
    print_info "GIT_TOKEN: ${GIT_TOKEN:+Set}"
    
    # Set all paths
    export ROS2_WS="$DEV_DIR/ros2_ws"
    export ROS2_SRC="$DEV_DIR/ros2_ws/src"
    export PX4_DIR="$DEV_DIR/PX4-Autopilot"
    export PX4_config="$DEV_DIR/PX4_config"
    export OSQP_SRC="$DEV_DIR"
    
    # Create necessary directories
    mkdir -p "$ROS2_SRC"
    
    print_success "Environment validation completed"
    track_time
}

# Enhanced package installation with retry mechanism
install_packages_with_retry() {
    local packages=("$@")
    local max_retries=3
    local retry_count=0
    
    while [ $retry_count -lt $max_retries ]; do
        if sudo apt install -y "${packages[@]}"; then
            return 0
        else
            retry_count=$((retry_count + 1))
            print_warning "Package installation failed. Retry ${retry_count}/${max_retries}"
            sudo apt update
            sleep 2
        fi
    done
    
    print_error "Failed to install packages after ${max_retries} attempts"
    return 1
}

# Enhanced repository cloning with validation
clone_repository() {
    local repo_url="$1"
    local target_dir="$2"
    local branch="$3"
    local repo_name=$(basename "$repo_url" .git)
    
    print_info "Cloning ${repo_name} (${branch} branch)..."
    
    if [ -d "$target_dir" ]; then
        print_warning "${repo_name} exists, updating..."
        cd "$target_dir"
        git fetch origin
        git checkout "$branch"
        git pull origin "$branch"
        
        # Verify branch
        local current_branch=$(git rev-parse --abbrev-ref HEAD)
        if [ "$current_branch" != "$branch" ]; then
            print_error "Failed to checkout ${branch} branch"
            return 1
        fi
    else
        cd "$(dirname "$target_dir")"
        git clone "$repo_url" "$(basename "$target_dir")"
        cd "$target_dir"
        git checkout "$branch"
    fi
    
    print_success "${repo_name} ready ($(git rev-parse --short HEAD))"
    return 0
}

# Main installation steps
setup_ros2_agent_sim() {
    print_step "Setting up ROS2 Agent Simulation"
    
    clone_repository \
        "https://github.com/AbdullahGM1/ros2_agent_sim.git" \
        "$ROS2_SRC/ros2_agent_sim" \
        "main"
    
    # Update submodules
    cd "$ROS2_SRC/ros2_agent_sim"
    git submodule update --init --recursive
    
    # Verify submodules
    if [ -d "$ROS2_SRC/ros2_agent_sim/unitree_go2_ros2" ] && [ "$(ls -A "$ROS2_SRC/ros2_agent_sim/unitree_go2_ros2")" ]; then
        print_success "Unitree Go2 submodule verified"
    else
        print_warning "Unitree Go2 submodule issue detected, fixing..."
        git submodule update --init --recursive --force
    fi
    
    track_time
}

install_ros2_packages() {
    print_step "Installing ROS2 Jazzy Packages"
    
    local packages=(
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
        "ros-jazzy-geographic-msgs"
    )
    
    # Install packages with progress tracking
    local total_packages=${#packages[@]}
    local current_package=0
    
    for pkg in "${packages[@]}"; do
        current_package=$((current_package + 1))
        print_info "[${current_package}/${total_packages}] Installing ${pkg}..."
        
        if ! install_packages_with_retry "$pkg"; then
            print_error "Failed to install ${pkg}"
            return 1
        fi
    done
    
    print_success "All ROS2 Jazzy packages installed"
    track_time
}

install_python_dependencies() {
    print_step "Installing Python Dependencies"
    
    # Install with proper error handling
    local python_packages=(
        "rich"
        "langchain"
        "langchain-ollama"
        "langchain-community==0.3.21"
        "opencv-python"
        "PyYAML==6.0.1"
        "rosa"
        "symforce"
        "numpy==1.26.4"
        "rospkg"
    )
    
    print_info "Installing Python packages..."
    for pkg in "${python_packages[@]}"; do
        print_info "Installing ${pkg}..."
        if ! pip3 install --break-system-packages --no-warn-script-location "$pkg"; then
            print_error "Failed to install ${pkg}"
            return 1
        fi
    done
    
    print_success "Python dependencies installed"
    track_time
}

setup_px4_autopilot() {
    print_step "Setting up PX4 Autopilot"
    
    # Clean installation
    if [ -d "$PX4_DIR" ]; then
        print_warning "Removing existing PX4 installation for clean build"
        rm -rf "$PX4_DIR"
    fi
    
    # Clone PX4
    clone_repository \
        "https://github.com/AbdullahGM1/PX4-Autopilot.git" \
        "$PX4_DIR" \
        "navsat_callback"
    
    # Clean and build
    cd "$PX4_DIR"
    print_info "Cleaning build artifacts..."
    rm -rf build/ || true
    make distclean || true
    
    # Build with progress indication
    print_info "Building PX4 SITL (this may take several minutes)..."
    export CMAKE_ARGS="-Wno-dev"
    
    if ! make px4_sitl; then
        print_error "PX4 SITL build failed"
        return 1
    fi
    
    print_success "PX4 SITL built successfully"
    track_time
}

copy_px4_configurations() {
    print_step "Copying PX4 Configuration Files"
    
    # Ensure target directories exist
    mkdir -p "${PX4_DIR}/Tools/simulation/gz/models/"
    mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds/"
    mkdir -p "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
    
    # Copy with verification
    local config_types=("models" "worlds" "px4")
    local target_dirs=(
        "${PX4_DIR}/Tools/simulation/gz/models/"
        "${PX4_DIR}/Tools/simulation/gz/worlds/"
        "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
    )
    
    for i in "${!config_types[@]}"; do
        local config_type="${config_types[$i]}"
        local target_dir="${target_dirs[$i]}"
        
        if [ -d "${PX4_config}/${config_type}" ]; then
            cp -r "${PX4_config}/${config_type}/"* "$target_dir"
            print_success "${config_type} configuration copied"
        else
            print_warning "${config_type} configuration not found"
        fi
    done
    
    # Rebuild PX4 with new configurations
    print_info "Rebuilding PX4 with new configurations..."
    cd "$PX4_DIR"
    make px4_sitl
    
    track_time
}

setup_mavros_and_mavlink() {
    print_step "Setting up MAVROS and MAVLink"
    
    # Clone mavlink
    clone_repository \
        "https://github.com/ros2-gbp/mavlink-gbp-release.git" \
        "$ROS2_SRC/mavlink" \
        "release/jazzy/mavlink"
    
    # Clone MAVROS
    clone_repository \
        "https://github.com/AbdullahGM1/mavros.git" \
        "$ROS2_SRC/mavros" \
        "ros2"
    
    print_success "MAVROS and MAVLink setup completed"
    track_time
}

handle_dependencies() {
    print_step "Resolving ROS2 Dependencies"
    
    cd "$ROS2_WS"
    
    # Initialize rosdep if needed
    if ! rosdep init 2>/dev/null; then
        print_info "rosdep already initialized"
    fi
    
    # Update rosdep
    rosdep update
    
    # Fix gz_sim rosdep issue for Unitree packages
    if [ -d "$ROS2_SRC/ros2_agent_sim/unitree_go2_ros2" ]; then
        print_info "Fixing gz_sim dependencies in Unitree packages..."
        find "$ROS2_SRC/ros2_agent_sim/unitree_go2_ros2" -name "package.xml" -exec sed -i '/<.*depend>gz_sim<\/.*depend>/d' {} \;
        find "$ROS2_SRC/ros2_agent_sim/unitree_go2_ros2" -name "package.xml" -exec sed -i '/<.*depend>gz-sim<\/.*depend>/d' {} \;
    fi
    
    # Install dependencies
    print_info "Installing ROS2 dependencies..."
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || {
        print_warning "Some dependencies failed to install, continuing..."
    }
    
    track_time
}

build_workspace() {
    print_step "Building ROS2 Workspace"
    
    cd "$ROS2_WS"
    
    # Set Gazebo version
    export GZ_VERSION=harmonic
    
    # Build with progress indication
    print_info "Building workspace (this may take several minutes)..."
    
    if ! colcon build --executor sequential --event-handlers console_direct+; then
        print_error "Workspace build failed"
        return 1
    fi
    
    print_success "Workspace built successfully"
    track_time
}

finalize_installation() {
    print_step "Finalizing Installation"
    
    # Source the workspace
    cd "$ROS2_WS"
    source install/setup.bash
    
    # FIXED: Check if .bashrc exists and is writable
    if [ ! -f ~/.bashrc ]; then
        print_warning ".bashrc not found, creating basic version..."
        touch ~/.bashrc
    fi
    
    # Make a backup of current .bashrc
    cp ~/.bashrc ~/.bashrc.backup.$(date +%s)
    print_info "Backed up existing .bashrc"
    
    # FIXED: Add workspace sourcing with correct path validation
    local workspace_source_line="# Auto-added by install script"
    local workspace_command="if [ -f \"$ROS2_WS/install/setup.bash\" ]; then source \"$ROS2_WS/install/setup.bash\"; fi"
    
    if ! grep -q "$workspace_source_line" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "$workspace_source_line" >> ~/.bashrc
        echo "$workspace_command" >> ~/.bashrc
        print_success "Added conditional workspace sourcing to .bashrc"
    else
        print_info "Workspace sourcing already configured in .bashrc"
    fi
    
    # FIXED: Add Python path with conditional check
    local python_path_comment="# Python local path - auto-added by install script"
    local python_path_command='if [ -d "$HOME/.local/bin" ]; then export PATH="$HOME/.local/bin:$PATH"; fi'
    
    if ! grep -q "$python_path_comment" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "$python_path_comment" >> ~/.bashrc
        echo "$python_path_command" >> ~/.bashrc
        print_success "Added conditional Python path to .bashrc"
    else
        print_info "Python path already configured in .bashrc"
    fi
    
    # FIXED: Add environment variables with validation
    local env_comment="# ROS2 Agent Sim environment - auto-added by install script"
    if ! grep -q "$env_comment" ~/.bashrc; then
        cat >> ~/.bashrc << EOF

$env_comment
export DEV_DIR="$DEV_DIR"
export PX4_DIR="$PX4_DIR"
export ROS2_WS="$ROS2_WS"
export OSQP_SRC="$OSQP_SRC"
export GZ_VERSION="harmonic"

# Convenient aliases
alias cd_ws='cd \$ROS2_WS'
alias cd_dev='cd \$DEV_DIR'
alias source_ws='if [ -f "\$ROS2_WS/install/setup.bash" ]; then source "\$ROS2_WS/install/setup.bash"; fi'
alias rosdep_install='rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy'
alias colcon_build='colcon build --executor sequential --event-handlers console_direct+'

EOF
        print_success "Added environment variables and aliases to .bashrc"
    else
        print_info "Environment variables already configured in .bashrc"
    fi
    
    # Verify the .bashrc is valid by testing it
    print_info "Validating .bashrc syntax..."
    if bash -n ~/.bashrc; then
        print_success ".bashrc syntax is valid"
    else
        print_error ".bashrc has syntax errors! Restoring backup..."
        cp ~/.bashrc.backup.$(date +%s) ~/.bashrc
        return 1
    fi
    
    # Test sourcing the .bashrc
    print_info "Testing .bashrc sourcing..."
    if bash -c "source ~/.bashrc && echo 'Bashrc sourced successfully'"; then
        print_success ".bashrc sources without errors"
    else
        print_warning ".bashrc has sourcing issues, but continuing..."
    fi
    
    track_time
}

# Main execution
main() {
    print_header "ROS2 Jazzy + Gazebo Harmonic + PX4 Installation"
    print_info "Log file: ${LOG_FILE}"
    print_info "Installation started at $(date)"
    
    # Run installation steps
    check_system_requirements
    validate_environment
    sudo apt update
    setup_ros2_agent_sim
    install_ros2_packages
    install_python_dependencies
    setup_px4_autopilot
    copy_px4_configurations
    setup_mavros_and_mavlink
    handle_dependencies
    build_workspace
    finalize_installation
    
    # Final summary
    local total_time=$(( $(date +%s) - start_time ))
    print_header "Installation Completed Successfully!"
    print_success "Total installation time: ${total_time}s"
    print_info "Log file saved: ${LOG_FILE}"
    
    echo -e "\n${GREEN}🎉 Next steps:${NC}"
    echo -e "${CYAN}1. Source the workspace:${NC}  source $ROS2_WS/install/setup.bash"
    echo -e "${CYAN}2. Launch the simulation:${NC} ros2 launch drone_sim drone.launch.py"
    echo -e "${CYAN}3. Run the ROS2 agent:${NC}   ros2 run ros2_agent ros2_agent_node"
    
    cd "$HOME"
}

# Run main function
main "$@"