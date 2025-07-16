#!/bin/bash -e

# ============================================================================
# ROS2 Agent Sim - Simplified Runtime Installation Script
# Only handles runtime-specific tasks (Docker handles build-time setup)
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
TOTAL_STEPS=6  # Reduced from 12 since Docker handles most setup

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

# Environment validation (runtime-specific)
validate_environment() {
    print_step "Validating Runtime Environment"
    
    if [ -z "${DEV_DIR}" ]; then
        print_error "DEV_DIR environment variable is not set"
        print_info "Set it using: export DEV_DIR=/home/user/shared_volume"
        exit 1
    fi
    
    print_info "DEV_DIR: ${DEV_DIR}"
    
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

# Copy pre-cloned repositories from container to shared volume
setup_repositories() {
    print_step "Setting up Repositories in Shared Volume"
    
    # Copy ros2_agent_sim if not exists
    if [ ! -d "$ROS2_SRC/ros2_agent_sim" ]; then
        print_info "Copying ros2_agent_sim from container..."
        cp -r /home/user/ros2_ws/src/ros2_agent_sim "$ROS2_SRC/"
        print_success "ros2_agent_sim copied"
    else
        print_info "ros2_agent_sim already exists in shared volume"
    fi
    
    # Copy mavlink if not exists
    if [ ! -d "$ROS2_SRC/mavlink" ]; then
        print_info "Copying mavlink from container..."
        cp -r /home/user/ros2_ws/src/mavlink "$ROS2_SRC/"
        print_success "mavlink copied"
    else
        print_info "mavlink already exists in shared volume"
    fi
    
    # Copy mavros if not exists
    if [ ! -d "$ROS2_SRC/mavros" ]; then
        print_info "Copying mavros from container..."
        cp -r /home/user/ros2_ws/src/mavros "$ROS2_SRC/"
        print_success "mavros copied"
    else
        print_info "mavros already exists in shared volume"
    fi
    
    # Copy PX4-Autopilot if not exists
    if [ ! -d "$PX4_DIR" ]; then
        print_info "Copying PX4-Autopilot from container..."
        cp -r /tmp/PX4-Autopilot "$PX4_DIR"
        print_success "PX4-Autopilot copied"
    else
        print_info "PX4-Autopilot already exists in shared volume"
    fi
    
    track_time
}

# Build PX4 with shared volume configurations
setup_px4_autopilot() {
    print_step "Setting up Pre-built PX4 Autopilot"
    
    # Check if PX4 already exists in shared volume
    if [ -d "$PX4_DIR" ]; then
        print_info "PX4 already exists in shared volume, checking integrity..."
        
        # Check if it's properly built (binary exists)
        if [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
            print_warning "PX4 binary missing, will replace with pre-built version"
            rm -rf "$PX4_DIR"
        else
            print_success "PX4 appears to be properly built"
            track_time
            return 0
        fi
    fi
    
    # Copy pre-built PX4 from container to shared volume
    if [ -d "/tmp/PX4-Autopilot" ]; then
        print_info "Copying pre-built PX4 from container to shared volume..."
        cp -r /tmp/PX4-Autopilot "$PX4_DIR"
        
        # Apply custom configurations if available
        if [ -d "$PX4_config" ]; then
            print_info "Applying custom PX4 configurations..."
            
            # Copy models
            if [ -d "$PX4_config/models" ]; then
                mkdir -p "${PX4_DIR}/Tools/simulation/gz/models/"
                cp -r "$PX4_config/models/"* "${PX4_DIR}/Tools/simulation/gz/models/"
                print_success "Models configuration applied"
            fi
            
            # Copy worlds
            if [ -d "$PX4_config/worlds" ]; then
                mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds/"
                cp -r "$PX4_config/worlds/"* "${PX4_DIR}/Tools/simulation/gz/worlds/"
                print_success "Worlds configuration applied"
            fi
            
            # Copy airframes
            if [ -d "$PX4_config/px4" ]; then
                mkdir -p "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
                cp -r "$PX4_config/px4/"* "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
                print_success "Airframes configuration applied"
            fi
            
            # Rebuild with new configurations
            print_info "Rebuilding PX4 with custom configurations..."
            cd "$PX4_DIR"
            export CMAKE_ARGS="-Wno-dev"
            if make px4_sitl; then
                print_success "PX4 rebuilt with custom configurations"
            else
                print_warning "PX4 rebuild failed, but pre-built version should still work"
            fi
        fi
        
        print_success "Pre-built PX4 setup completed successfully"
    else
        print_error "Pre-built PX4 not found in container at /tmp/PX4-Autopilot"
        print_info "This suggests the Docker build didn't complete PX4 setup properly"
        exit 1
    fi
    
    track_time
}

# Handle ROS2 dependencies (runtime-specific)
handle_dependencies() {
    print_step "Resolving ROS2 Dependencies"
    
    cd "$ROS2_WS"
    
    # Install missing Python packages first
    print_info "Installing missing Python dependencies..."
    pip install --no-cache-dir "lark>=1.1.0" "lark-parser>=0.12.0" || {
        print_warning "Failed to install some Python packages"
    }
    
    # Initialize rosdep if needed
    if ! rosdep init 2>/dev/null; then
        print_info "rosdep already initialized"
    fi
    
    # Update rosdep
    rosdep update
    
    # Install dependencies with error handling
    print_info "Installing ROS2 dependencies..."
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || {
        print_warning "Some rosdep dependencies failed to install"
        
        # Try installing critical packages manually
        print_info "Attempting to install critical packages manually..."
        sudo apt-get update || true
        sudo apt-get install -y \
            python3-future \
            ros-jazzy-joint-state-publisher \
            ros-jazzy-joint-state-publisher-gui \
            ros-jazzy-rosidl-generator-py \
            ros-jazzy-rosidl-runtime-py \
            2>/dev/null || print_warning "Some manual installations failed"
    }
    
    track_time
}

# Build workspace (runtime-specific)
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

# Finalize installation (runtime-specific bashrc setup)
finalize_installation() {
    print_step "Finalizing Installation"
    
    # Source the workspace
    cd "$ROS2_WS"
    source install/setup.bash
    
    # Check if .bashrc exists
    if [ ! -f ~/.bashrc ]; then
        print_warning ".bashrc not found, creating basic version..."
        touch ~/.bashrc
    fi
    
    # Make a backup of current .bashrc
    cp ~/.bashrc ~/.bashrc.backup.$(date +%s)
    print_info "Backed up existing .bashrc"
    
    # Add workspace sourcing
    local workspace_source_line="# Auto-added by install script"
    local workspace_command="if [ -f \"$ROS2_WS/install/setup.bash\" ]; then source \"$ROS2_WS/install/setup.bash\"; fi"
    
    if ! grep -q "$workspace_source_line" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "$workspace_source_line" >> ~/.bashrc
        echo "$workspace_command" >> ~/.bashrc
        print_success "Added workspace sourcing to .bashrc"
    else
        print_info "Workspace sourcing already configured"
    fi
    
    # Add environment variables
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
        print_info "Environment variables already configured"
    fi
    
    # Validate .bashrc
    print_info "Validating .bashrc syntax..."
    if bash -n ~/.bashrc; then
        print_success ".bashrc syntax is valid"
    else
        print_error ".bashrc has syntax errors! Restoring backup..."
        cp ~/.bashrc.backup.$(date +%s) ~/.bashrc
        return 1
    fi
    
    track_time
}

# Main execution
main() {
    print_header "ROS2 Agent Sim - Runtime Setup"
    print_info "Log file: ${LOG_FILE}"
    print_info "Installation started at $(date)"
    print_info "Note: Docker image provides all packages and tools"
    
    # Run runtime-specific installation steps
    validate_environment
    setup_repositories
    setup_px4_autopilot
    handle_dependencies
    build_workspace
    finalize_installation
    
    # Final summary
    local total_time=$(( $(date +%s) - start_time ))
    print_header "Runtime Setup Completed Successfully!"
    print_success "Total setup time: ${total_time}s"
    print_info "Log file saved: ${LOG_FILE}"
    
    echo -e "\n${GREEN}🎉 Next steps:${NC}"
    echo -e "${CYAN}1. Source the workspace:${NC}  source $ROS2_WS/install/setup.bash"
    echo -e "${CYAN}2. Launch the simulation:${NC} ros2 launch drone_sim drone.launch.py"
    echo -e "${CYAN}3. Run the ROS2 agent:${NC}   ros2 run ros2_agent ros2_agent_node"
    
    cd "$HOME"
}

# Run main function
main "$@"