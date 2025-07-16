#!/bin/bash -e

# ============================================================================
# ROS2 Agent Sim - FIXED Runtime Installation Script
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
TOTAL_STEPS=7  # Updated for graphics verification step

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

# CRITICAL FIX: Graphics environment validation
validate_graphics_environment() {
    print_step "Validating Graphics Environment"
    
    print_info "Testing comprehensive graphics stack..."
    
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
    
    # Test OpenGL capabilities
    if command -v glxinfo >/dev/null 2>&1; then
        print_info "Testing OpenGL support..."
        
        if timeout 10 glxinfo -B >/dev/null 2>&1; then
            RENDERER=$(glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Hardware")
            GL_VERSION=$(glxinfo -B 2>/dev/null | grep "OpenGL version" | cut -d: -f2 | xargs || echo "Unknown")
            print_success "OpenGL working: $RENDERER"
            print_info "OpenGL Version: $GL_VERSION"
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
    
    # Test Qt6 environment
    print_info "Verifying Qt6 environment..."
    
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
    
    print_success "Graphics environment validation completed"
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
    
    # CRITICAL FIX: Set graphics environment for build process
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_INDIRECT=0
    export MESA_GL_VERSION_OVERRIDE="4.5"
    export GALLIUM_DRIVER="llvmpipe"
    
    print_success "Environment validation completed with graphics support"
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

# Build PX4 with shared volume configurations
setup_px4_autopilot() {
    print_step "Setting up PX4 Autopilot (Runtime Build)"
    
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
    
    # CRITICAL FIX: Set build environment variables
    export CMAKE_ARGS="-Wno-dev"
    export MAKEFLAGS="-j$(nproc)"
    
    # Clean any previous build attempts
    if [ -d "build" ]; then
        print_info "Cleaning previous build..."
        rm -rf build/
    fi
    
    # CRITICAL FIX: Build with graphics environment set
    print_info "Building with graphics-aware environment..."
    
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

# Handle ROS2 dependencies (runtime-specific)
handle_dependencies() {
    print_step "Resolving ROS2 Dependencies"
    
    cd "$ROS2_WS"
    
    # Install missing Python dependencies
    print_info "Installing missing Python dependencies..."
    pip install --no-cache-dir \
        "lark>=1.1.0" \
        "lark-parser>=0.12.0" \
        2>/dev/null || {
        print_warning "Some Python packages failed to install (may not be critical)"
    }
    
    # Initialize rosdep if needed
    if ! rosdep init 2>/dev/null; then
        print_info "rosdep already initialized"
    fi
    
    # Update rosdep
    rosdep update
    
    # Install dependencies with enhanced error handling
    print_info "Installing ROS2 dependencies..."
    if rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy; then
        print_success "ROS2 dependencies installed successfully"
    else
        print_warning "Some rosdep dependencies failed, trying manual installation..."
        
        # Install critical packages manually
        print_info "Installing critical packages manually..."
        sudo apt-get update 2>/dev/null || true
        
        # Install packages that definitely exist in Ubuntu 24.04
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
            libgl1-mesa-dev \
            libosmesa6 \
            2>/dev/null || print_warning "Some manual installations failed"
    fi
    
    track_time
}

# Build workspace (runtime-specific)
build_workspace() {
    print_step "Building ROS2 Workspace"
    
    cd "$ROS2_WS"
    
    # CRITICAL FIX: Set comprehensive build environment
    export GZ_VERSION=harmonic
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_INDIRECT=0
    
    # Build with progress indication and enhanced error handling
    print_info "Building workspace with graphics support (this may take several minutes)..."
    
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
            print_info "Common issues:"
            print_info "  1. Check for Qt6/Qt5 conflicts"
            print_info "  2. Verify graphics environment"
            print_info "  3. Check available memory"
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

# Finalize installation with enhanced graphics support
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
    local workspace_source_line="# Auto-added by install script (FIXED VERSION)"
    local workspace_command="if [ -f \"$ROS2_WS/install/setup.bash\" ]; then source \"$ROS2_WS/install/setup.bash\"; fi"
    
    if ! grep -q "$workspace_source_line" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "$workspace_source_line" >> ~/.bashrc
        echo "$workspace_command" >> ~/.bashrc
        print_success "Added workspace sourcing to .bashrc"
    else
        print_info "Workspace sourcing already configured"
    fi
    
    # CRITICAL FIX: Add comprehensive environment variables
    local env_comment="# ROS2 Agent Sim environment - FIXED VERSION"
    if ! grep -q "$env_comment" ~/.bashrc; then
        cat >> ~/.bashrc << EOF

$env_comment
export DEV_DIR="$DEV_DIR"
export PX4_DIR="$PX4_DIR"
export ROS2_WS="$ROS2_WS"
export OSQP_SRC="$OSQP_SRC"
export GZ_VERSION="harmonic"

# CRITICAL FIX: Graphics environment
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

# CRITICAL FIX: Enhanced graphics aliases
alias gazebo_start='QT_QPA_PLATFORM=xcb gz sim'
alias gazebo_software='LIBGL_ALWAYS_SOFTWARE=1 gz sim'
alias rviz_start='QT_QPA_PLATFORM=xcb rviz2'
alias graphics_test='echo "X11: \$(timeout 3 xset q >/dev/null 2>&1 && echo OK || echo FAIL)"; echo "OpenGL: \$(timeout 5 glxinfo -B >/dev/null 2>&1 && echo OK || echo FAIL)"; echo "Qt6: \$(find /usr -name "*qt6*" -name "*platforms*" 2>/dev/null | grep -q . && echo OK || echo FAIL)"'

EOF
        print_success "Added comprehensive environment variables and aliases to .bashrc"
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
    
    # CRITICAL FIX: Test graphics environment after installation
    print_info "Testing graphics environment after installation..."
    
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

# CRITICAL FIX: Graphics verification step
verify_graphics_installation() {
    print_step "Verifying Graphics Installation"
    
    print_info "Running comprehensive graphics verification..."
    
    # Test Gazebo specifically
    print_info "Testing Gazebo Harmonic..."
    if timeout 10 gz --version >/dev/null 2>&1; then
        GZ_VERSION_FULL=$(gz --version 2>/dev/null | head -1)
        print_success "Gazebo verified: $GZ_VERSION_FULL"
    else
        print_error "Gazebo command test failed"
        return 1
    fi
    
    # Test critical ROS2 packages
    print_info "Testing ROS2 packages..."
    CRITICAL_PACKAGES="ros2_agent drone_sim"
    for pkg in $CRITICAL_PACKAGES; do
        if ros2 pkg list | grep -q "$pkg" 2>/dev/null; then
            print_success "Package verified: $pkg"
        else
            print_warning "Package not found: $pkg (may need manual installation)"
        fi
    done
    
    # Create test scripts for user
    print_info "Creating test scripts..."
    
    cat > "$DEV_DIR/test_graphics.sh" << 'EOF'
#!/bin/bash
# Graphics test script - automatically generated

echo "🔍 Graphics Environment Test"
echo "=========================="

echo -n "X11 Test: "
if timeout 5 xset q >/dev/null 2>&1; then
    echo "✅ PASS (Display: $DISPLAY)"
else
    echo "❌ FAIL"
fi

echo -n "OpenGL Test: "
if timeout 10 glxinfo -B >/dev/null 2>&1; then
    RENDERER=$(glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs)
    echo "✅ PASS ($RENDERER)"
elif LIBGL_ALWAYS_SOFTWARE=1 timeout 10 glxinfo -B >/dev/null 2>&1; then
    echo "⚠️ SOFTWARE ONLY"
else
    echo "❌ FAIL"
fi

echo -n "Qt6 Test: "
if find /usr -name "*qt6*" -name "*platforms*" 2>/dev/null | grep -q .; then
    echo "✅ PASS"
else
    echo "❌ FAIL"
fi

echo -n "Gazebo Test: "
if timeout 10 gz --version >/dev/null 2>&1; then
    echo "✅ PASS"
else
    echo "❌ FAIL"
fi

echo -n "ROS2 Test: "
if ros2 pkg list >/dev/null 2>&1; then
    echo "✅ PASS"
else
    echo "❌ FAIL"
fi

echo ""
echo "Environment Variables:"
echo "  DISPLAY: $DISPLAY"
echo "  QT_QPA_PLATFORM: $QT_QPA_PLATFORM"
echo "  LIBGL_ALWAYS_SOFTWARE: ${LIBGL_ALWAYS_SOFTWARE:-0}"
echo "  MESA_GL_VERSION_OVERRIDE: $MESA_GL_VERSION_OVERRIDE"
EOF
    
    chmod +x "$DEV_DIR/test_graphics.sh"
    chown $(whoami):$(whoami) "$DEV_DIR/test_graphics.sh" 2>/dev/null || true
    
    print_success "Test script created: $DEV_DIR/test_graphics.sh"
    
    track_time
}

# Main execution
main() {
    print_header "ROS2 Agent Sim - FIXED Runtime Setup"
    print_info "Log file: ${LOG_FILE}"
    print_info "Installation started at $(date)"
    print_info "FIXES: Qt6 conflicts, graphics support, enhanced error handling"
    
    # Run installation steps with graphics validation
    validate_graphics_environment  # NEW: Validate graphics first
    validate_environment
    setup_repositories
    setup_px4_autopilot
    handle_dependencies
    build_workspace
    finalize_installation
    verify_graphics_installation   # NEW: Final verification
    
    # Final summary
    local total_time=$(( $(date +%s) - start_time ))
    print_header "FIXED Runtime Setup Completed Successfully!"
    print_success "Total setup time: ${total_time}s"
    print_info "Log file saved: ${LOG_FILE}"
    
    echo -e "\n${GREEN}🎉 Next steps (FIXED VERSION):${NC}"
    echo -e "${CYAN}1. Test graphics:${NC}         ./test_graphics.sh"
    echo -e "${CYAN}2. Source workspace:${NC}      source $ROS2_WS/install/setup.bash"
    echo -e "${CYAN}3. Launch simulation:${NC}     ros2 launch drone_sim drone.launch.py"
    echo -e "${CYAN}4. Run ROS2 agent:${NC}        ros2 run ros2_agent ros2_agent_node"
    echo -e "${CYAN}5. Start Gazebo manually:${NC}  gz sim (or gazebo_start alias)"
    
    echo -e "\n${YELLOW}🔧 Troubleshooting commands:${NC}"
    echo -e "${CYAN}- Graphics issues:${NC}        diagnose_graphics"
    echo -e "${CYAN}- Software rendering:${NC}     gazebo_software"
    echo -e "${CYAN}- Qt6 debug:${NC}              QT_DEBUG_PLUGINS=1 gz sim"
    echo -e "${CYAN}- Test environment:${NC}       graphics_test"
    
    print_success "🎯 Installation completed with comprehensive graphics support!"
    
    cd "$HOME"
}

# Run main function
main "$@"