#!/bin/bash -e

# ============================================================================
# ROS2 Agent Sim - FIXED Runtime Installation Script (Ubuntu 24.04)
# Handles runtime-specific tasks with comprehensive graphics and Qt6 support
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
TOTAL_STEPS=8  # Updated for Ubuntu 24.04 verification step

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
    print_info "For graphics issues, try: diagnose_graphics"
    print_info "For Ubuntu 24.04 package issues, try: check_packages"
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

# CRITICAL FIX: Ubuntu 24.04 package verification
verify_ubuntu_packages() {
    print_step "Verifying Ubuntu 24.04 Package Compatibility"
    
    UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
    print_info "Detected Ubuntu version: $UBUNTU_VERSION"
    
    if [ "$UBUNTU_VERSION" = "24.04" ]; then
        print_success "Ubuntu 24.04 confirmed"
    else
        print_warning "Ubuntu version may not be 24.04 - proceeding with compatibility mode"
    fi
    
    # Check for critical Ubuntu 24.04 packages
    print_info "Checking Ubuntu 24.04 package availability..."
    
    MISSING_PACKAGES=""
    PACKAGE_WARNINGS=""
    
    # Check libgl1 (replaces libgl1-mesa-glx)
    if ! dpkg -l | grep -q "^ii.*libgl1[^-]"; then
        MISSING_PACKAGES="$MISSING_PACKAGES libgl1"
    else
        print_success "libgl1 package found (Ubuntu 24.04 compatible)"
    fi
    
    # Check libglx-mesa0 (part of libgl1-mesa-glx replacement)
    if ! dpkg -l | grep -q "^ii.*libglx-mesa0"; then
        MISSING_PACKAGES="$MISSING_PACKAGES libglx-mesa0"
    else
        print_success "libglx-mesa0 package found (Ubuntu 24.04 compatible)"
    fi
    
    # Check libglut3.12 (replaces freeglut3)
    if ! dpkg -l | grep -q "^ii.*libglut3.12"; then
        MISSING_PACKAGES="$MISSING_PACKAGES libglut3.12"
    else
        print_success "libglut3.12 package found (Ubuntu 24.04 compatible)"
    fi
    
    # Check Qt6 packages
    if ! dpkg -l | grep -q "^ii.*qt6-base"; then
        MISSING_PACKAGES="$MISSING_PACKAGES qt6-base-dev"
    else
        print_success "Qt6 packages found (Ubuntu 24.04 compatible)"
    fi
    
    # Check for deprecated packages that shouldn't be present
    if dpkg -l | grep -q "^ii.*libgl1-mesa-glx"; then
        PACKAGE_WARNINGS="$PACKAGE_WARNINGS libgl1-mesa-glx(deprecated)"
    fi
    
    if dpkg -l | grep -q "^ii.*freeglut3[^-]"; then
        PACKAGE_WARNINGS="$PACKAGE_WARNINGS freeglut3(deprecated)"
    fi
    
    # Report results
    if [ -n "$MISSING_PACKAGES" ]; then
        print_warning "Missing Ubuntu 24.04 packages: $MISSING_PACKAGES"
        print_info "These packages should have been installed during Docker build"
    fi
    
    if [ -n "$PACKAGE_WARNINGS" ]; then
        print_warning "Deprecated packages detected: $PACKAGE_WARNINGS"
        print_info "These may cause conflicts but fallbacks are configured"
    fi
    
    if [ -z "$MISSING_PACKAGES" ] && [ -z "$PACKAGE_WARNINGS" ]; then
        print_success "All Ubuntu 24.04 packages properly configured"
    fi
    
    track_time
}

# CRITICAL FIX: Graphics environment validation (Ubuntu 24.04)
validate_graphics_environment() {
    print_step "Validating Graphics Environment (Ubuntu 24.04)"
    
    print_info "Testing comprehensive graphics stack for Ubuntu 24.04..."
    
    # Test X11 connection
    if timeout 5 xset q >/dev/null 2>&1; then
        X11_DISPLAY=$(echo $DISPLAY)
        print_success "X11 working on $X11_DISPLAY"
    else
        print_warning "X11 not working on $DISPLAY, checking alternatives..."
        
        # Try to find working display
        for disp in :1 :0 :10 :2; do
            if DISPLAY="$disp" timeout 3 xset q >/dev/null 2>&1; then
                export DISPLAY="$disp"
                print_success "Found working display: $DISPLAY"
                break
            fi
        done
        
        if ! timeout 3 xset q >/dev/null 2>&1; then
            print_warning "No working X11 display found - GUI applications will use software fallbacks"
        fi
    fi
    
    # Test OpenGL capabilities (Ubuntu 24.04 aware)
    if command -v glxinfo >/dev/null 2>&1; then
        print_info "Testing OpenGL support (Ubuntu 24.04)..."
        
        if timeout 10 glxinfo -B >/dev/null 2>&1; then
            RENDERER=$(glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Hardware")
            GL_VERSION=$(glxinfo -B 2>/dev/null | grep "OpenGL version" | cut -d: -f2 | xargs || echo "Unknown")
            print_success "OpenGL working: $RENDERER"
            print_info "OpenGL Version: $GL_VERSION"
            
            # Check if Mesa software rendering
            if echo "$RENDERER" | grep -qi "llvmpipe\|softpipe\|swrast"; then
                print_info "Mesa software rendering detected (normal for containers)"
            fi
        else
            print_info "Hardware OpenGL failed, testing software rendering..."
            if LIBGL_ALWAYS_SOFTWARE=1 timeout 10 glxinfo -B >/dev/null 2>&1; then
                SOFTWARE_RENDERER=$(LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Software")
                print_success "Software OpenGL working: $SOFTWARE_RENDERER"
            else
                print_warning "Both hardware and software OpenGL tests failed"
                print_info "Gazebo will attempt to use fallback rendering"
            fi
        fi
    else
        print_warning "glxinfo not available for OpenGL testing"
    fi
    
    # Test Qt6 environment (Ubuntu 24.04 specific)
    print_info "Verifying Qt6 environment (Ubuntu 24.04)..."
    
    # Check Qt6 platform plugins
    QT6_PLATFORMS_DIR="/usr/lib/x86_64-linux-gnu/qt6/plugins/platforms"
    if [ -d "$QT6_PLATFORMS_DIR" ]; then
        AVAILABLE_PLATFORMS=$(ls "$QT6_PLATFORMS_DIR" 2>/dev/null | grep -E "\.(so|dylib)$" | sed 's/lib//g' | sed 's/\.so.*//g' | tr '\n' ' ')
        print_success "Qt6 platforms available: $AVAILABLE_PLATFORMS"
        
        # Verify xcb platform specifically
        if [ -f "$QT6_PLATFORMS_DIR/libqxcb.so" ]; then
            print_success "Qt6 XCB platform plugin found"
        else
            print_warning "Qt6 XCB platform plugin missing"
        fi
    else
        print_warning "Qt6 platform plugins directory not found"
    fi
    
    # Test Gazebo command availability
    if command -v gz >/dev/null 2>&1; then
        GZ_VERSION_OUTPUT=$(gz --version 2>/dev/null | head -1 || echo "Gazebo available")
        print_success "Gazebo command available: $GZ_VERSION_OUTPUT"
    else
        print_error "Gazebo command not found"
        return 1
    fi
    
    print_success "Graphics environment validation completed for Ubuntu 24.04"
    track_time
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
    
    # CRITICAL FIX: Set graphics environment for build process (Ubuntu 24.04)
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_INDIRECT=0
    export MESA_GL_VERSION_OVERRIDE="4.5"
    export GALLIUM_DRIVER="llvmpipe"
    
    print_success "Environment validation completed with Ubuntu 24.04 graphics support"
    track_time
}

# Copy pre-cloned repositories from container to shared volume
setup_repositories() {
    print_step "Setting up Repositories in Shared Volume"
    
    # Copy ros2_agent_sim if not exists
    if [ ! -d "$ROS2_SRC/ros2_agent_sim" ]; then
        print_info "Copying ros2_agent_sim from container..."
        if [ -d "/home/user/ros2_ws/src/ros2_agent_sim" ]; then
            cp -r /home/user/ros2_ws/src/ros2_agent_sim "$ROS2_SRC/"
            print_success "ros2_agent_sim copied"
        else
            print_warning "ros2_agent_sim not found in container, will need manual installation"
        fi
    else
        print_info "ros2_agent_sim already exists in shared volume"
    fi
    
    # Copy mavlink if not exists
    if [ ! -d "$ROS2_SRC/mavlink" ]; then
        print_info "Copying mavlink from container..."
        if [ -d "/home/user/ros2_ws/src/mavlink" ]; then
            cp -r /home/user/ros2_ws/src/mavlink "$ROS2_SRC/"
            print_success "mavlink copied"
        else
            print_warning "mavlink not found in container"
        fi
    else
        print_info "mavlink already exists in shared volume"
    fi
    
    # Copy mavros if not exists
    if [ ! -d "$ROS2_SRC/mavros" ]; then
        print_info "Copying mavros from container..."
        if [ -d "/home/user/ros2_ws/src/mavros" ]; then
            cp -r /home/user/ros2_ws/src/mavros "$ROS2_SRC/"
            print_success "mavros copied"
        else
            print_warning "mavros not found in container"
        fi
    else
        print_info "mavros already exists in shared volume"
    fi
    
    # Copy PX4-Autopilot 
    if [ ! -d "$PX4_DIR" ]; then
        print_info "Copying PX4-Autopilot from container..."
        if [ -d "/opt/px4-source" ]; then
            cp -r /opt/px4-source "$PX4_DIR"
            print_success "PX4-Autopilot copied from /opt/px4-source"
        else
            print_error "PX4 source not found at /opt/px4-source"
            print_info "Available directories:"
            ls -la /opt/ | grep px4 || echo "No px4 directories found in /opt/"
            exit 1
        fi
    else
        print_info "PX4-Autopilot already exists in shared volume"
    fi
    
    track_time
}

# Build PX4 with shared volume configurations (Ubuntu 24.04 compatible)
setup_px4_autopilot() {
    print_step "Setting up PX4 Autopilot (Runtime Build - Ubuntu 24.04)"
    
    # Check if PX4 already exists and is properly built
    if [ -d "$PX4_DIR" ] && [ -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
        print_info "PX4 already built in shared volume"
        
        # Verify it's working
        if "$PX4_DIR/build/px4_sitl_default/bin/px4" --help >/dev/null 2>&1; then
            print_success "PX4 binary verified as working"
            track_time
            return 0
        else
            print_warning "PX4 binary exists but not working, rebuilding..."
        fi
    fi
    
    # Verify PX4 directory exists
    if [ ! -d "$PX4_DIR" ]; then
        print_error "PX4 directory not found at $PX4_DIR"
        print_info "Run setup_repositories first"
        exit 1
    fi
    
    # Apply custom configurations
    if [ -d "$PX4_config" ]; then
        print_info "Applying custom PX4 configurations..."
        
        if [ -d "$PX4_config/models" ]; then
            mkdir -p "${PX4_DIR}/Tools/simulation/gz/models/"
            cp -r "$PX4_config/models/"* "${PX4_DIR}/Tools/simulation/gz/models/"
            print_success "Models configuration applied"
        fi
        
        if [ -d "$PX4_config/worlds" ]; then
            mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds/"
            cp -r "$PX4_config/worlds/"* "${PX4_DIR}/Tools/simulation/gz/worlds/"
            print_success "Worlds configuration applied"
        fi
        
        if [ -d "$PX4_config/px4" ]; then
            mkdir -p "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
            cp -r "$PX4_config/px4/"* "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
            print_success "Airframes configuration applied"
        fi
    fi
    
    # Build PX4 in the final location
    print_info "Building PX4 in final location (this may take 10-15 minutes)..."
    cd "$PX4_DIR"
    
    # CRITICAL FIX: Set build environment variables for Ubuntu 24.04
    export CMAKE_ARGS="-Wno-dev"
    export MAKEFLAGS="-j$(nproc)"
    
    # Clean any previous build attempts
    if [ -d "build" ]; then
        print_info "Cleaning previous build..."
        rm -rf build/
    fi
    
    # CRITICAL FIX: Build with graphics environment set for Ubuntu 24.04
    print_info "Building with Ubuntu 24.04 graphics-aware environment..."
    
    if make px4_sitl; then
        print_success "PX4 built successfully in final location"
        
        # Test the binary
        if [ -f "build/px4_sitl_default/bin/px4" ]; then
            PX4_SIZE=$(ls -lh build/px4_sitl_default/bin/px4 | awk '{print $5}')
            print_success "PX4 binary verified: $PX4_SIZE"
            
            # Quick functionality test
            if timeout 10 ./build/px4_sitl_default/bin/px4 --help >/dev/null 2>&1; then
                print_success "PX4 binary functionality verified"
            else
                print_warning "PX4 binary built but functionality test failed"
            fi
        else
            print_error "PX4 binary not found after build"
            return 1
        fi
    else
        print_error "PX4 build failed"
        print_info "Check build logs for details"
        return 1
    fi
    
    track_time
}

# Handle ROS2 dependencies (Ubuntu 24.04 specific)
handle_dependencies() {
    print_step "Resolving ROS2 Dependencies (Ubuntu 24.04)"
    
    cd "$ROS2_WS"
    
    print_info "Installing missing Python dependencies for Ubuntu 24.04..."
    pip3 install --break-system-packages --no-cache-dir \
        "lark>=1.1.0" "lark-parser>=0.12.0" \
        2>/dev/null || {
        print_warning "Some Python packages failed to install (may not be critical)"
    }
    
    # Initialize rosdep if needed
    if ! rosdep init 2>/dev/null; then
        print_info "rosdep already initialized"
    fi
    
    # Update rosdep
    rosdep update
    
    # Install dependencies with enhanced error handling for Ubuntu 24.04
    print_info "Installing ROS2 dependencies (Ubuntu 24.04 mode)..."
    if rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy; then
        print_success "ROS2 dependencies installed successfully"
    else
        print_warning "Some rosdep dependencies failed, trying manual installation..."
        
        # Install critical packages manually (Ubuntu 24.04 compatible)
        print_info "Installing critical packages manually for Ubuntu 24.04..."
        sudo apt-get update 2>/dev/null || true
        
        # CRITICAL FIX: Install Ubuntu 24.04 compatible packages
        sudo apt-get install -y \
            python3-future \
            ros-jazzy-joint-state-publisher \
            ros-jazzy-joint-state-publisher-gui \
            ros-jazzy-rosidl-generator-py \
            ros-jazzy-rosidl-runtime-py \
            qt6-base-dev \
            libqt6opengl6-dev \
            qt6-qpa-plugins \
            mesa-utils \
            libgl1 \
            libglx-mesa0 \
            libglut3.12 \
            libglut-dev \
            libosmesa6 \
            2>/dev/null || print_warning "Some manual installations failed"
    fi
    
    track_time
}

# Build workspace (Ubuntu 24.04 compatible)
build_workspace() {
    print_step "Building ROS2 Workspace (Ubuntu 24.04)"
    
    cd "$ROS2_WS"
    
    # CRITICAL FIX: Set comprehensive build environment for Ubuntu 24.04
    export GZ_VERSION=harmonic
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_INDIRECT=0
    
    # Build with progress indication and enhanced error handling
    print_info "Building workspace with Ubuntu 24.04 graphics support (this may take several minutes)..."
    
    # Try sequential build first (more reliable)
    if colcon build --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo; then
        print_success "Workspace built successfully with sequential executor"
    else
        print_warning "Sequential build failed, trying parallel build..."
        
        # Clean and try parallel build
        rm -rf build/ install/ log/ 2>/dev/null || true
        
        if colcon build --executor parallel --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo; then
            print_success "Workspace built successfully with parallel executor"
        else
            print_error "Both sequential and parallel builds failed"
            print_info "Common issues for Ubuntu 24.04:"
            print_info "  1. Check for Qt6/Qt5 conflicts"
            print_info "  2. Verify graphics environment"
            print_info "  3. Check available memory"
            print_info "  4. Run 'check_packages' to verify Ubuntu 24.04 packages"
            return 1
        fi
    fi
    
    # Verify build artifacts
    if [ -f "install/setup.bash" ]; then
        print_success "Build artifacts verified"
        
        # Test sourcing the workspace
        if source install/setup.bash 2>/dev/null; then
            print_success "Workspace sourcing test passed"
        else
            print_warning "Workspace built but sourcing test failed"
        fi
    else
        print_error "Build completed but install/setup.bash not found"
        return 1
    fi
    
    track_time
}

# Finalize installation with enhanced graphics support (Ubuntu 24.04)
finalize_installation() {
    print_step "Finalizing Installation (Ubuntu 24.04)"
    
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
    local workspace_source_line="# Auto-added by install script (FIXED VERSION - Ubuntu 24.04)"
    local workspace_command="if [ -f \"$ROS2_WS/install/setup.bash\" ]; then source \"$ROS2_WS/install/setup.bash\"; fi"
    
    if ! grep -q "$workspace_source_line" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "$workspace_source_line" >> ~/.bashrc
        echo "$workspace_command" >> ~/.bashrc
        print_success "Added workspace sourcing to .bashrc"
    else
        print_info "Workspace sourcing already configured"
    fi
    
    # CRITICAL FIX: Add comprehensive environment variables for Ubuntu 24.04
    local env_comment="# ROS2 Agent Sim environment - FIXED VERSION (Ubuntu 24.04)"
    if ! grep -q "$env_comment" ~/.bashrc; then
        cat >> ~/.bashrc << EOF

$env_comment
export DEV_DIR="$DEV_DIR"
export PX4_DIR="$PX4_DIR"
export ROS2_WS="$ROS2_WS"
export OSQP_SRC="$OSQP_SRC"
export GZ_VERSION="harmonic"

# CRITICAL FIX: Graphics environment (Ubuntu 24.04)
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export MESA_GL_VERSION_OVERRIDE="4.5"
export GALLIUM_DRIVER="llvmpipe"

# Convenient aliases
alias cd_ws='cd \$ROS2_WS'
alias cd_dev='cd \$DEV_DIR'
alias source_ws='if [ -f "\$ROS2_WS/install/setup.bash" ]; then source "\$ROS2_WS/install/setup.bash"; fi'
alias rosdep_install='rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy'
alias colcon_build='colcon build --executor sequential --event-handlers console_direct+'

# CRITICAL FIX: Enhanced graphics aliases (Ubuntu 24.04)
alias gazebo_start='QT_QPA_PLATFORM=xcb gz sim'
alias gazebo_software='LIBGL_ALWAYS_SOFTWARE=1 gz sim'
alias rviz_start='QT_QPA_PLATFORM=xcb rviz2'
alias graphics_test='echo "X11: \$(timeout 3 xset q >/dev/null 2>&1 && echo OK || echo FAIL)"; echo "OpenGL: \$(timeout 5 glxinfo -B >/dev/null 2>&1 && echo OK || echo FAIL)"; echo "Qt6: \$(find /usr -name "*qt6*" -name "*platforms*" 2>/dev/null | grep -q . && echo OK || echo FAIL)"'

# Ubuntu 24.04 specific aliases
alias check_packages_ubuntu='echo "Ubuntu 24.04 packages:"; dpkg -l | grep -E "(libgl1|libglx-mesa0|libglut3.12|qt6-base)" | awk "{print \$2,\$3}"'
alias check_deprecated='echo "Deprecated packages (should not be present):"; dpkg -l | grep -E "(libgl1-mesa-glx|freeglut3)" | awk "{print \$2,\$3}" || echo "None found (good)"'

EOF
        print_success "Added comprehensive environment variables and aliases to .bashrc for Ubuntu 24.04"
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
    
    # CRITICAL FIX: Test graphics environment after installation (Ubuntu 24.04)
    print_info "Testing graphics environment after installation (Ubuntu 24.04)..."
    
    # Test X11
    if timeout 3 xset q >/dev/null 2>&1; then
        print_success "X11 test passed"
    else
        print_warning "X11 test failed - GUI apps will use software fallbacks"
    fi
    
    # Test ROS2 workspace
    if ros2 pkg list >/dev/null 2>&1; then
        PKG_COUNT=$(ros2 pkg list | wc -l)
        print_success "ROS2 workspace functional ($PKG_COUNT packages available)"
    else
        print_warning "ROS2 workspace test failed"
    fi
    
    track_time
}

# Main execution
main() {
    print_header "ROS2 Agent Sim - Runtime Setup (Ubuntu 24.04)"
    print_info "Log file: ${LOG_FILE}"
    print_info "Installation started at $(date)"
    print_info "FIXES: Ubuntu 24.04 compatibility, Qt6 conflicts, graphics support, enhanced error handling"
    
    # Run installation steps with Ubuntu 24.04 validation
    verify_ubuntu_packages        # NEW: Verify Ubuntu 24.04 packages first
    validate_graphics_environment # Enhanced: Ubuntu 24.04 graphics validation
    validate_environment
    setup_repositories
    setup_px4_autopilot
    handle_dependencies
    build_workspace
    finalize_installation
    
    # Final summary
    local total_time=$(( $(date +%s) - start_time ))
    print_header "Runtime Setup Completed Successfully (Ubuntu 24.04)!"
    print_success "Total setup time: ${total_time}s"
    print_info "Log file saved: ${LOG_FILE}"
    
    echo -e "\n${GREEN}🎉 Next steps:${NC}"
    echo -e "${CYAN}3. Build workspace:${NC}      colcon build"
    echo -e "${CYAN}3. Source workspace:${NC}      source $ROS2_WS/install/setup.bash"
    echo -e "${CYAN}4. Launch simulation:${NC}     ros2 launch drone_sim drone.launch.py"
    echo -e "${CYAN}5. Run ROS2 agent:${NC}        ros2 run ros2_agent ros2_agent_node"
    
    print_success "🎯 Installation completed with comprehensive Ubuntu 24.04 support!"
    
    cd "$HOME"
}

# Run main function
main "$@"