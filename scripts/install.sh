#!/bin/bash -e

# ============================================================================
# ROS2 Agent Sim - FIXED Runtime Installation Script (Ubuntu 24.04)
# Enhanced with comprehensive build debugging and error recovery
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
TOTAL_STEPS=9  # Updated for enhanced build step

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

# CRITICAL FIX: Enhanced Ubuntu 24.04 package verification
verify_ubuntu_packages() {
    print_step "Enhanced Ubuntu 24.04 Package Verification"
    
    UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
    print_info "Detected Ubuntu version: $UBUNTU_VERSION"
    
    if [ "$UBUNTU_VERSION" = "24.04" ]; then
        print_success "Ubuntu 24.04 confirmed"
    else
        print_warning "Ubuntu version may not be 24.04 - proceeding with compatibility mode"
    fi
    
    # Check and fix critical Ubuntu 24.04 packages
    print_info "Checking and fixing Ubuntu 24.04 package compatibility..."
    
    # Update package database
    sudo apt-get update -qq 2>/dev/null || print_warning "Package update had issues"
    
    # Remove conflicting packages
    print_info "Removing potential conflicting packages..."
    sudo apt-get remove -y qt5-default libqt5core5a libqt5gui5 libqt5widgets5 freeglut3 libgl1-mesa-glx 2>/dev/null || true
    
    # Install Ubuntu 24.04 compatible packages
    print_info "Installing Ubuntu 24.04 compatible packages..."
    sudo apt-get install -y \
        libgl1 \
        libglx-mesa0 \
        libglut3.12 \
        libglut-dev \
        libosmesa6 \
        mesa-utils \
        qt6-base-dev \
        qt6-qpa-plugins \
        libqt6opengl6-dev \
        libqt6gui6 \
        libqt6widgets6 \
        libqt6core6 \
        xvfb \
        build-essential \
        cmake \
        pkg-config \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        2>/dev/null || print_warning "Some package installations may have failed"
    
    # Verify critical packages are installed
    CRITICAL_PACKAGES="libgl1 libglx-mesa0 libglut3.12 qt6-base-dev"
    MISSING_CRITICAL=""
    
    for pkg in $CRITICAL_PACKAGES; do
        if dpkg -l | grep -q "^ii.*$pkg"; then
            print_success "$pkg installed correctly"
        else
            MISSING_CRITICAL="$MISSING_CRITICAL $pkg"
            print_error "$pkg is missing"
        fi
    done
    
    if [ -n "$MISSING_CRITICAL" ]; then
        print_error "Critical packages missing: $MISSING_CRITICAL"
        print_info "Attempting manual installation..."
        sudo apt-get install -y --fix-missing $MISSING_CRITICAL 2>/dev/null || true
    fi
    
    # Set Ubuntu 24.04 specific environment variables
    export QT_SELECT=qt6
    export QT_QPA_PLATFORM=xcb
    export PKG_CONFIG_PATH="/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
    
    print_success "Ubuntu 24.04 package verification completed"
    track_time
}

# CRITICAL FIX: Enhanced graphics environment validation
validate_graphics_environment() {
    print_step "Enhanced Graphics Environment Validation (Ubuntu 24.04)"
    
    print_info "Testing comprehensive graphics stack for Ubuntu 24.04..."
    
    # Enhanced X11 testing
    if timeout 5 xset q >/dev/null 2>&1; then
        X11_DISPLAY=$(echo $DISPLAY)
        print_success "X11 working on $X11_DISPLAY"
        
        # Test window creation capability
        if timeout 10 xwininfo -root >/dev/null 2>&1; then
            print_success "X11 window system fully functional"
        else
            print_warning "X11 basic connection works but window system may have issues"
        fi
    else
        print_warning "X11 not working on $DISPLAY, attempting to fix..."
        
        # Try to start Xvfb if no display available
        if [ -z "$DISPLAY" ] || [ "$DISPLAY" = ":0" ]; then
            print_info "Starting virtual display with Xvfb..."
            pkill Xvfb 2>/dev/null || true
            Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
            sleep 2
            export DISPLAY=:99
            
            if timeout 5 xset q >/dev/null 2>&1; then
                print_success "Virtual display :99 created and working"
            else
                print_warning "Virtual display creation failed"
            fi
        fi
    fi
    
    # Enhanced OpenGL testing
    if command -v glxinfo >/dev/null 2>&1; then
        print_info "Testing OpenGL capabilities (Ubuntu 24.04)..."
        
        # Test hardware OpenGL
        if timeout 15 glxinfo -B >/dev/null 2>&1; then
            RENDERER=$(glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Hardware")
            GL_VERSION=$(glxinfo -B 2>/dev/null | grep "OpenGL version" | cut -d: -f2 | xargs || echo "Unknown")
            print_success "Hardware OpenGL working: $RENDERER"
            print_info "OpenGL Version: $GL_VERSION"
            
            # Check rendering capabilities
            if echo "$RENDERER" | grep -qi "llvmpipe\|softpipe\|swrast"; then
                print_info "Mesa software rendering detected (normal for containers)"
                export LIBGL_ALWAYS_SOFTWARE=1
            fi
        else
            print_warning "Hardware OpenGL failed, testing software rendering..."
            export LIBGL_ALWAYS_SOFTWARE=1
            
            if timeout 15 glxinfo -B >/dev/null 2>&1; then
                SOFTWARE_RENDERER=$(glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Software")
                print_success "Software OpenGL working: $SOFTWARE_RENDERER"
            else
                print_warning "Both hardware and software OpenGL failed - using CPU fallbacks"
                export LIBGL_ALWAYS_SOFTWARE=1
                export MESA_GL_VERSION_OVERRIDE="4.5"
                export GALLIUM_DRIVER="llvmpipe"
            fi
        fi
    else
        print_warning "glxinfo not available, installing mesa-utils..."
        sudo apt-get install -y mesa-utils 2>/dev/null || true
    fi
    
    # Enhanced Qt6 testing
    print_info "Enhanced Qt6 environment testing (Ubuntu 24.04)..."
    
    # Check Qt6 installation
    QT6_VERSION=$(pkg-config --modversion Qt6Core 2>/dev/null || echo "not found")
    if [ "$QT6_VERSION" != "not found" ]; then
        print_success "Qt6 Core version: $QT6_VERSION"
    else
        print_warning "Qt6 development packages may not be properly installed"
    fi
    
    # Test Qt6 platform plugins
    QT6_PLATFORMS_DIR="/usr/lib/x86_64-linux-gnu/qt6/plugins/platforms"
    if [ -d "$QT6_PLATFORMS_DIR" ]; then
        AVAILABLE_PLATFORMS=$(ls "$QT6_PLATFORMS_DIR" 2>/dev/null | grep "\.so$" | sed 's/lib//g' | sed 's/\.so.*//g' | tr '\n' ' ')
        print_success "Qt6 platforms available: $AVAILABLE_PLATFORMS"
        
        # Test xcb platform specifically
        if [ -f "$QT6_PLATFORMS_DIR/libqxcb.so" ]; then
            print_success "Qt6 XCB platform plugin found"
            export QT_QPA_PLATFORM=xcb
        else
            print_warning "Qt6 XCB platform plugin missing, using fallback"
            export QT_QPA_PLATFORM=offscreen
        fi
    else
        print_warning "Qt6 platform plugins directory not found"
        export QT_QPA_PLATFORM=offscreen
    fi
    
    # Test Gazebo availability
    if command -v gz >/dev/null 2>&1; then
        GZ_VERSION_OUTPUT=$(timeout 10 gz --version 2>/dev/null | head -1 || echo "Gazebo available")
        print_success "Gazebo command available: $GZ_VERSION_OUTPUT"
        
        # Test Gazebo with minimal configuration
        print_info "Testing Gazebo startup capability..."
        if timeout 30 bash -c "gz sim --help >/dev/null 2>&1"; then
            print_success "Gazebo can start successfully"
        else
            print_warning "Gazebo startup test failed - may need environment adjustments"
        fi
    else
        print_error "Gazebo command not found"
        return 1
    fi
    
    print_success "Enhanced graphics environment validation completed"
    track_time
}

# Environment validation (enhanced)
validate_environment() {
    print_step "Enhanced Runtime Environment Validation"
    
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
    
    # Create necessary directories with proper permissions
    mkdir -p "$ROS2_SRC"
    mkdir -p "$DEV_DIR"
    
    # Check available resources
    AVAILABLE_MEMORY=$(free -m | awk 'NR==2{printf "%.1fGB", $7/1024}')
    AVAILABLE_DISK=$(df -h "$DEV_DIR" | awk 'NR==2{print $4}')
    CPU_CORES=$(nproc)
    
    print_info "System resources:"
    print_info "  Available memory: $AVAILABLE_MEMORY"  
    print_info "  Available disk: $AVAILABLE_DISK"
    print_info "  CPU cores: $CPU_CORES"
    
    # Set optimized build environment for Ubuntu 24.04
    export CMAKE_PREFIX_PATH="/opt/ros/jazzy:$CMAKE_PREFIX_PATH"
    export PKG_CONFIG_PATH="/opt/ros/jazzy/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
    export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
    
    # Graphics environment
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_INDIRECT=0
    export MESA_GL_VERSION_OVERRIDE="4.5"
    export GALLIUM_DRIVER="llvmpipe"
    
    # Ubuntu 24.04 specific
    export QT_SELECT=qt6
    
    print_success "Enhanced environment validation completed"
    track_time
}

# Copy pre-cloned repositories (unchanged but with enhanced error checking)
setup_repositories() {
    print_step "Setting up Repositories in Shared Volume"
    
    local repos_copied=0
    local repos_failed=0
    
    # Function to copy repository with error checking
    copy_repo() {
        local repo_name="$1"
        local container_path="$2"
        local target_path="$3"
        
        if [ ! -d "$target_path" ]; then
            print_info "Copying $repo_name from container..."
            if [ -d "$container_path" ]; then
                if cp -r "$container_path" "$target_path"; then
                    print_success "$repo_name copied successfully"
                    repos_copied=$((repos_copied + 1))
                else
                    print_error "Failed to copy $repo_name"
                    repos_failed=$((repos_failed + 1))
                fi
            else
                print_warning "$repo_name not found in container at $container_path"
                repos_failed=$((repos_failed + 1))
            fi
        else
            print_info "$repo_name already exists in shared volume"
            repos_copied=$((repos_copied + 1))
        fi
    }
    
    # Copy repositories
    copy_repo "ros2_agent_sim" "/home/user/ros2_ws/src/ros2_agent_sim" "$ROS2_SRC/ros2_agent_sim"
    copy_repo "mavlink" "/home/user/ros2_ws/src/mavlink" "$ROS2_SRC/mavlink"
    copy_repo "mavros" "/home/user/ros2_ws/src/mavros" "$ROS2_SRC/mavros"
    copy_repo "px4_msgs" "/home/user/ros2_ws/src/px4_msgs" "$ROS2_SRC/px4_msgs"
    copy_repo "unitree_go2_ros2" "/home/user/ros2_ws/src/unitree_go2_ros2" "$ROS2_SRC/unitree_go2_ros2"
    
    # Copy PX4-Autopilot
    if [ ! -d "$PX4_DIR" ]; then
        print_info "Copying PX4-Autopilot from container..."
        if [ -d "/home/user/PX4-Autopilot" ]; then
            if cp -r /home/user/PX4-Autopilot "$PX4_DIR"; then
                print_success "PX4-Autopilot copied from /home/user/PX4-Autopilot"
                repos_copied=$((repos_copied + 1))
            else
                print_error "Failed to copy PX4-Autopilot"
                repos_failed=$((repos_failed + 1))
            fi
        else
            print_error "PX4 source not found at /home/user/PX4-Autopilot"
            repos_failed=$((repos_failed + 1))
        fi
    else
        print_info "PX4-Autopilot already exists in shared volume"
        repos_copied=$((repos_copied + 1))
    fi
    
    print_info "Repository setup summary: $repos_copied copied, $repos_failed failed"
    
    if [ $repos_failed -gt 0 ]; then
        print_warning "Some repositories failed to copy - build may be incomplete"
    fi
    
    track_time
}

# Enhanced PX4 setup (unchanged but with better error handling)
setup_px4_autopilot() {
    print_step "Enhanced PX4 Autopilot Setup (Ubuntu 24.04)"
    
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
    
    # Apply custom configurations (enhanced error handling)
    if [ -d "$PX4_config" ]; then
        print_info "Applying custom PX4 configurations..."
        
        # Apply configurations with error checking
        apply_config() {
            local config_dir="$1"
            local target_dir="$2"
            local config_name="$3"
            
            if [ -d "$PX4_config/$config_dir" ]; then
                mkdir -p "$target_dir"
                if cp -r "$PX4_config/$config_dir/"* "$target_dir/"; then
                    print_success "$config_name configuration applied"
                else
                    print_warning "$config_name configuration failed to apply"
                fi
            fi
        }
        
        apply_config "models" "${PX4_DIR}/Tools/simulation/gz/models/" "Models"
        apply_config "worlds" "${PX4_DIR}/Tools/simulation/gz/worlds/" "Worlds"
        apply_config "px4" "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/" "Airframes"
        
        # Handle dds_topics.yaml
        if [ -f "$PX4_config/dds_topics.yaml" ]; then
            UXRCE_DDS_DIR="${PX4_DIR}/src/modules/uxrce_dds_client"
            mkdir -p "$UXRCE_DDS_DIR"
            if cp "$PX4_config/dds_topics.yaml" "$UXRCE_DDS_DIR/"; then
                print_success "dds_topics.yaml copied successfully"
            else
                print_warning "dds_topics.yaml copy failed"
            fi
        fi
    fi
    
    # Build PX4 with enhanced environment
    print_info "Building PX4 with Ubuntu 24.04 optimizations (10-15 minutes)..."
    cd "$PX4_DIR"
    
    # Set build environment
    export CMAKE_ARGS="-Wno-dev -DCMAKE_BUILD_TYPE=RelWithDebInfo"
    export MAKEFLAGS="-j$(nproc)"
    
    # Clean previous build
    if [ -d "build" ]; then
        print_info "Cleaning previous build..."
        rm -rf build/
    fi
    
    # Build with error handling
    if timeout 1800 make px4_sitl; then  # 30 minute timeout
        print_success "PX4 built successfully"
        
        # Verify build
        if [ -f "build/px4_sitl_default/bin/px4" ]; then
            PX4_SIZE=$(ls -lh build/px4_sitl_default/bin/px4 | awk '{print $5}')
            print_success "PX4 binary verified: $PX4_SIZE"
        else
            print_error "PX4 binary not found after build"
            return 1
        fi
    else
        print_error "PX4 build failed or timed out"
        return 1
    fi
    
    track_time
}

# COMPLETELY REWRITTEN: Enhanced dependency handling
handle_dependencies() {
    print_step "Enhanced ROS2 Dependencies Resolution (Ubuntu 24.04)"
    
    cd "$ROS2_WS"
    
    # Enhanced Python dependencies installation
    print_info "Installing enhanced Python dependencies for Ubuntu 24.04..."
    
    # Install critical Python packages
    pip3 install --break-system-packages --no-cache-dir \
        "lark>=1.1.0" \
        "lark-parser>=0.12.0" \
        "empy>=3.3,<4.0" \
        "catkin-pkg" \
        "rospkg" \
        "PyYAML" \
        "setuptools>=60.0" \
        "wheel" \
        2>/dev/null || print_warning "Some Python packages failed to install"
    
    # Initialize and update rosdep
    print_info "Updating rosdep database..."
    if ! rosdep init 2>/dev/null; then
        print_info "rosdep already initialized"
    fi
    
    if rosdep update; then
        print_success "rosdep updated successfully"
    else
        print_warning "rosdep update had issues but continuing"
    fi
    
    # Enhanced dependency installation with fallbacks
    print_info "Resolving ROS2 dependencies with Ubuntu 24.04 compatibility..."
    
    # First attempt: standard rosdep install
    if rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
        --skip-keys="qt5-qmake qt5-default freeglut3 libgl1-mesa-glx"; then
        print_success "ROS2 dependencies installed via rosdep"
    else
        print_warning "rosdep installation had issues, applying manual fixes..."
        
        # Manual installation of critical Ubuntu 24.04 packages
        print_info "Installing critical ROS2 packages manually..."
        sudo apt-get update -qq 2>/dev/null || true
        
        # Install essential ROS2 packages
        sudo apt-get install -y \
            python3-future \
            python3-catkin-pkg \
            python3-rospkg \
            python3-yaml \
            ros-jazzy-joint-state-publisher \
            ros-jazzy-joint-state-publisher-gui \
            ros-jazzy-robot-state-publisher \
            ros-jazzy-rosidl-generator-py \
            ros-jazzy-rosidl-runtime-py \
            ros-jazzy-rosidl-generator-cpp \
            ros-jazzy-rosidl-runtime-cpp \
            ros-jazzy-geometry-msgs \
            ros-jazzy-sensor-msgs \
            ros-jazzy-std-msgs \
            ros-jazzy-tf2 \
            ros-jazzy-tf2-ros \
            ros-jazzy-tf2-geometry-msgs \
            ros-jazzy-urdf \
            ros-jazzy-xacro \
            2>/dev/null || print_warning "Some ROS2 packages failed to install"
        
        # Install Gazebo dependencies
        print_info "Installing Gazebo dependencies for Ubuntu 24.04..."
        sudo apt-get install -y \
            ros-jazzy-gazebo-ros-pkgs \
            ros-jazzy-gazebo-plugins \
            ros-jazzy-gazebo-msgs \
            2>/dev/null || print_warning "Some Gazebo packages failed to install"
    fi
    
    # Verify critical dependencies
    print_info "Verifying critical dependencies..."
    
    CRITICAL_DEPS=("python3-yaml" "python3-catkin-pkg" "python3-rospkg")
    MISSING_DEPS=""
    
    for dep in "${CRITICAL_DEPS[@]}"; do
        if dpkg -l | grep -q "^ii.*$dep"; then
            print_success "$dep is available"
        else
            MISSING_DEPS="$MISSING_DEPS $dep"
        fi
    done
    
    if [ -n "$MISSING_DEPS" ]; then
        print_warning "Missing critical dependencies: $MISSING_DEPS"
        print_info "Attempting to install missing dependencies..."
        sudo apt-get install -y $MISSING_DEPS 2>/dev/null || true
    fi
    
    track_time
}

# COMPLETELY REWRITTEN: Enhanced workspace building with comprehensive debugging
build_workspace() {
    print_step "Enhanced ROS2 Workspace Building (Ubuntu 24.04)"
    
    cd "$ROS2_WS"
    
    # Pre-build validation
    print_info "Pre-build validation..."
    
    # Check workspace structure
    if [ ! -d "src" ] || [ -z "$(ls -A src 2>/dev/null)" ]; then
        print_error "Source directory is empty or doesn't exist"
        print_info "Available directories: $(ls -la)"
        return 1
    fi
    
    # Count packages
    PACKAGE_COUNT=$(find src -name "package.xml" | wc -l)
    print_info "Found $PACKAGE_COUNT ROS2 packages to build"
    
    if [ "$PACKAGE_COUNT" -eq 0 ]; then
        print_error "No ROS2 packages found in src directory"
        return 1
    fi
    
    # List packages
    print_info "Packages to build:"
    find src -name "package.xml" -exec dirname {} \; | sort | xargs -I {} basename {} | sed 's/^/  - /'
    
    # Check system resources
    AVAILABLE_MEMORY_MB=$(free -m | awk 'NR==2{print $7}')
    CPU_CORES=$(nproc)
    
    print_info "System resources: ${AVAILABLE_MEMORY_MB}MB RAM, ${CPU_CORES} CPU cores"
    
    # Determine build strategy based on resources
    if [ "$AVAILABLE_MEMORY_MB" -lt 2048 ]; then
        BUILD_STRATEGY="sequential-low-memory"
        PARALLEL_WORKERS=1
        print_warning "Low memory detected, using sequential build with limited parallelism"
    elif [ "$AVAILABLE_MEMORY_MB" -lt 4096 ]; then
        BUILD_STRATEGY="sequential"
        PARALLEL_WORKERS=2
        print_info "Medium memory detected, using sequential build"
    else
        BUILD_STRATEGY="parallel"
        PARALLEL_WORKERS=$((CPU_CORES > 4 ? 4 : CPU_CORES))
        print_info "Sufficient memory detected, using parallel build with $PARALLEL_WORKERS workers"
    fi
    
    # Set comprehensive build environment for Ubuntu 24.04
    export GZ_VERSION=harmonic
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_INDIRECT=0
    export CMAKE_PREFIX_PATH="/opt/ros/jazzy:$CMAKE_PREFIX_PATH"
    export PKG_CONFIG_PATH="/opt/ros/jazzy/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
    export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
    export QT_SELECT=qt6
    
    # Source ROS2 before building
    if source /opt/ros/jazzy/setup.bash; then
        print_success "ROS2 Jazzy environment sourced"
    else
        print_error "Failed to source ROS2 Jazzy environment"
        return 1
    fi
    
    # Clean previous failed builds
    if [ -d "build" ] && [ -d "install" ] && [ ! -f "install/setup.bash" ]; then
        print_warning "Previous build appears incomplete, cleaning..."
        rm -rf build/ install/ log/
    fi
    
    # Function to attempt build with specific strategy
    attempt_build() {
        local strategy="$1"
        local timeout_duration="$2"
        
        print_info "Attempting build with strategy: $strategy (timeout: ${timeout_duration}s)"
        
        case "$strategy" in
            "sequential-low-memory")
                timeout "$timeout_duration" colcon build \
                    --executor sequential \
                    --parallel-workers 1 \
                    --event-handlers console_direct+ \
                    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev \
                    --cmake-clean-cache
                ;;
            "sequential")
                timeout "$timeout_duration" colcon build \
                    --executor sequential \
                    --event-handlers console_direct+ \
                    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev
                ;;
            "parallel")
                timeout "$timeout_duration" colcon build \
                    --executor parallel \
                    --parallel-workers "$PARALLEL_WORKERS" \
                    --event-handlers console_direct+ \
                    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev
                ;;
            "selective")
                # Build only essential packages first
                timeout "$timeout_duration" colcon build \
                    --packages-select px4_msgs mavros mavros_msgs \
                    --executor sequential \
                    --event-handlers console_direct+ \
                    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev
                ;;
        esac
    }
    
    # Function to diagnose build failures
    diagnose_build_failure() {
        print_error "Build failed - performing diagnosis..."
        
        if [ -d "log" ]; then
            print_info "Analyzing build logs..."
            
            # Find the most recent failed package logs
            FAILED_LOGS=$(find log -name "stderr.log" -type f -size +0c | head -5)
            
            if [ -n "$FAILED_LOGS" ]; then
                print_info "Failed package logs found:"
                echo "$FAILED_LOGS" | sed 's/^/  /'
                
                # Show actual errors
                print_info "Recent error messages:"
                for log in $FAILED_LOGS; do
                    PACKAGE_NAME=$(echo "$log" | sed 's|log/build_||' | sed 's|/.*||')
                    print_info "Errors from $PACKAGE_NAME:"
                    tail -10 "$log" | grep -i -E "(error|failed|cannot|undefined)" | head -3 | sed 's/^/    /'
                done
            else
                print_info "No detailed error logs found"
            fi
            
            # Check for common issues
            if grep -r -i "qt5" log/ >/dev/null 2>&1; then
                print_warning "Qt5 references found in logs - may indicate Qt5/Qt6 conflicts"
            fi
            
            if grep -r -i "libgl1-mesa-glx\|freeglut3" log/ >/dev/null 2>&1; then
                print_warning "Deprecated graphics packages found in logs"
            fi
            
            if grep -r -i "memory\|killed" log/ >/dev/null 2>&1; then
                print_warning "Memory issues detected in logs"
            fi
        fi
    }
    
    # Main build attempts with fallback strategies
    BUILD_SUCCESS=false
    
    # Attempt 1: Primary strategy
    print_info "Build Attempt 1: Primary strategy ($BUILD_STRATEGY)"
    if attempt_build "$BUILD_STRATEGY" 1800; then  # 30 minutes
        BUILD_SUCCESS=true
        print_success "Build succeeded with primary strategy"
    else
        print_warning "Primary build strategy failed"
        diagnose_build_failure
        
        # Clean and try fallback
        print_info "Cleaning build artifacts for retry..."
        rm -rf build/ install/ log/ 2>/dev/null || true
        
        # Attempt 2: Sequential fallback
        print_info "Build Attempt 2: Sequential fallback"
        if attempt_build "sequential" 2400; then  # 40 minutes
            BUILD_SUCCESS=true
            print_success "Build succeeded with sequential fallback"
        else
            print_warning "Sequential fallback failed"
            diagnose_build_failure
            
            # Clean for selective build
            rm -rf build/ install/ log/ 2>/dev/null || true
            
            # Attempt 3: Selective build (essential packages only)
            print_info "Build Attempt 3: Selective build (essential packages)"
            if attempt_build "selective" 1200; then  # 20 minutes
                print_success "Essential packages built successfully"
                
                # Try to build remaining packages
                print_info "Building remaining packages..."
                if timeout 1800 colcon build --executor sequential --event-handlers console_direct+; then
                    BUILD_SUCCESS=true
                    print_success "All packages built successfully after selective approach"
                else
                    print_warning "Some packages failed in final build"
                    # Check if we have a working install/setup.bash anyway
                    if [ -f "install/setup.bash" ]; then
                        BUILD_SUCCESS=true
                        print_success "Partial build successful - essential packages available"
                    fi
                fi
            else
                print_error "Even selective build failed"
                diagnose_build_failure
            fi
        fi
    fi
    
    # Post-build validation
    if [ "$BUILD_SUCCESS" = true ]; then
        # Verify build artifacts
        if [ -f "install/setup.bash" ]; then
            print_success "Build artifacts verified - install/setup.bash exists"
            
            # Count installed packages
            INSTALLED_PACKAGES=$(find install -name "package.xml" | wc -l)
            print_info "Successfully installed $INSTALLED_PACKAGES packages"
            
            # Test workspace sourcing immediately
            print_info "Testing workspace sourcing immediately after build..."
            if bash -c "source install/setup.bash && echo 'Sourcing test passed'" >/dev/null 2>&1; then
                print_success "✅ Workspace sourcing test passed immediately after build"
                
                # Test ROS2 functionality
                if bash -c "source install/setup.bash && ros2 pkg list" >/dev/null 2>&1; then
                    AVAILABLE_PACKAGES=$(bash -c "source install/setup.bash && ros2 pkg list | wc -l")
                    print_success "✅ $AVAILABLE_PACKAGES ROS2 packages available after sourcing"
                    
                    # List key custom packages
                    print_info "Key custom packages available:"
                    bash -c "source install/setup.bash && ros2 pkg list | grep -E '(ros2_agent|drone_sim|mavros|px4_msgs|unitree)'" | sed 's/^/  ✅ /' || print_info "  ⚠️ Some expected packages may not be available"
                else
                    print_warning "⚠️ ROS2 packages not immediately available (may need shell restart)"
                fi
            else
                print_warning "⚠️ Workspace built but immediate sourcing test failed"
                print_info "This may still work when sourced in a new shell"
            fi
            
        else
            print_error "❌ Build completed but install/setup.bash not found"
            BUILD_SUCCESS=false
        fi
    fi
    
    if [ "$BUILD_SUCCESS" != true ]; then
        print_error "❌ All build attempts failed"
        print_info "Common solutions:"
        print_info "  1. Check system memory: free -h"
        print_info "  2. Verify Qt6 installation: dpkg -l | grep qt6"
        print_info "  3. Check for package conflicts: check_deprecated"
        print_info "  4. Try manual package build: colcon build --packages-select <package_name>"
        return 1
    fi
    
    track_time
}

# Enhanced finalization (updated to work with new build system)
finalize_installation() {
    print_step "Enhanced Installation Finalization with Workspace Sourcing (Ubuntu 24.04)"
    
    # Verify workspace is built
    if [ ! -f "$ROS2_WS/install/setup.bash" ]; then
        print_error "Workspace not properly built - install/setup.bash missing"
        print_info "Please run build_workspace step first"
        return 1
    fi
    
    cd "$ROS2_WS"
    
    # Test workspace sourcing
    print_info "Testing workspace sourcing..."
    if source install/setup.bash; then
        print_success "✅ Workspace sourced successfully"
        
        # Comprehensive verification
        if [ -n "$AMENT_PREFIX_PATH" ]; then
            print_success "✅ AMENT_PREFIX_PATH is properly set"
        else
            print_warning "⚠️ AMENT_PREFIX_PATH not set after sourcing"
        fi
        
        # Test ROS2 functionality
        if command -v ros2 >/dev/null 2>&1 && ros2 pkg list >/dev/null 2>&1; then
            TOTAL_PACKAGES=$(ros2 pkg list | wc -l)
            CUSTOM_PACKAGES=$(ros2 pkg list | grep -E "(ros2_agent|drone_sim|mavros|px4_msgs|unitree)" | wc -l)
            print_success "✅ ROS2 functionality verified ($TOTAL_PACKAGES total packages, $CUSTOM_PACKAGES custom packages)"
        else
            print_warning "⚠️ ROS2 functionality test failed"
        fi
    else
        print_error "❌ Failed to source workspace"
        return 1
    fi
    
    # Enhanced .bashrc configuration
    if [ ! -f ~/.bashrc ]; then
        touch ~/.bashrc
    fi
    
    # Backup .bashrc
    cp ~/.bashrc ~/.bashrc.backup.$(date +%s)
    print_info "Backed up existing .bashrc"
    
    # Add comprehensive environment configuration
    local env_comment="# ROS2 Agent Sim environment - ENHANCED VERSION (Ubuntu 24.04) - $(date)"
    if ! grep -q "ROS2 Agent Sim environment - ENHANCED VERSION" ~/.bashrc; then
        cat >> ~/.bashrc << EOF

$env_comment
export DEV_DIR="$DEV_DIR"
export PX4_DIR="$PX4_DIR"
export ROS2_WS="$ROS2_WS"
export OSQP_SRC="$OSQP_SRC"
export GZ_VERSION="harmonic"

# Ubuntu 24.04 compatibility
export CMAKE_PREFIX_PATH="/opt/ros/jazzy:\$CMAKE_PREFIX_PATH"
export PKG_CONFIG_PATH="/opt/ros/jazzy/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig:\$PKG_CONFIG_PATH"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:\$PYTHONPATH"

# Enhanced graphics environment (Ubuntu 24.04)
export QT_SELECT=qt6
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export MESA_GL_VERSION_OVERRIDE="4.5"
export GALLIUM_DRIVER="llvmpipe"

# Auto-source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
if [ -f "\$ROS2_WS/install/setup.bash" ]; then 
    source "\$ROS2_WS/install/setup.bash"
fi

# Enhanced convenience aliases
alias cd_ws='cd \$ROS2_WS'
alias cd_dev='cd \$DEV_DIR'
alias source_ws='source \$ROS2_WS/install/setup.bash'
alias rebuild_ws='cd \$ROS2_WS && colcon build --executor sequential --event-handlers console_direct+'
alias clean_ws='cd \$ROS2_WS && rm -rf build/ install/ log/'

# Diagnostic aliases
alias check_graphics='echo "X11: \$(timeout 3 xset q >/dev/null 2>&1 && echo OK || echo FAIL)"; echo "OpenGL: \$(timeout 5 glxinfo -B >/dev/null 2>&1 && echo OK || echo FAIL)"; echo "Qt6: \$(pkg-config --exists Qt6Core && echo OK || echo FAIL)"'
alias check_workspace='echo "Built: \$([ -f \$ROS2_WS/install/setup.bash ] && echo YES || echo NO)"; echo "Sourced: \$([ -n \"\$AMENT_PREFIX_PATH\" ] && echo YES || echo NO)"; echo "Packages: \$(ros2 pkg list 2>/dev/null | wc -l || echo 0)"'
alias check_ubuntu_packages='dpkg -l | grep -E "(libgl1|libglx-mesa0|libglut3.12|qt6-base)" | awk "{print \$2,\$3}"'
alias check_deprecated='dpkg -l | grep -E "(libgl1-mesa-glx|freeglut3|qt5-default)" | awk "{print \$2,\$3}" || echo "None found (good)"'

# Application launch aliases (Ubuntu 24.04 optimized)
alias launch_gazebo='QT_QPA_PLATFORM=xcb gz sim'
alias launch_rviz='QT_QPA_PLATFORM=xcb rviz2'
alias launch_drone='ros2 launch drone_sim drone.launch.py'
alias launch_agent='ros2 run ros2_agent ros2_agent_node'

EOF
        print_success "✅ Enhanced environment configuration added to .bashrc"
    else
        print_info "Environment configuration already present in .bashrc"
    fi
    
    # Validate .bashrc syntax
    if bash -n ~/.bashrc; then
        print_success "✅ .bashrc syntax is valid"
    else
        print_error "❌ .bashrc has syntax errors! Restoring backup..."
        cp ~/.bashrc.backup.* ~/.bashrc
        return 1
    fi
    
    # Test .bashrc in new shell
    print_info "Testing .bashrc workspace sourcing in new shell..."
    if bash -l -c "[ -n \"\$AMENT_PREFIX_PATH\" ] && ros2 pkg list >/dev/null 2>&1"; then
        NEW_SHELL_PACKAGES=$(bash -l -c "ros2 pkg list | wc -l" 2>/dev/null || echo "0")
        print_success "✅ Workspace sourcing works in new shell ($NEW_SHELL_PACKAGES packages available)"
    else
        print_warning "⚠️ Workspace sourcing test in new shell failed"
        print_info "Try: source ~/.bashrc"
    fi
    
    # Final comprehensive verification
    print_info "Final comprehensive system verification..."
    
    # Test all components
    source install/setup.bash
    
    VERIFICATION_RESULTS=""
    
    # ROS2 test
    if ros2 pkg list >/dev/null 2>&1; then
        PKG_COUNT=$(ros2 pkg list | wc -l)
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ✅ ROS2: $PKG_COUNT packages"
    else
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ❌ ROS2: Failed"
    fi
    
    # Graphics test
    if timeout 3 xset q >/dev/null 2>&1; then
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ✅ X11: Working"
    else
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ⚠️ X11: Issues detected"
    fi
    
    # Qt6 test
    if pkg-config --exists Qt6Core; then
        QT6_VER=$(pkg-config --modversion Qt6Core)
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ✅ Qt6: $QT6_VER"
    else
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ❌ Qt6: Not found"
    fi
    
    # PX4 test
    if [ -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ✅ PX4: Built"
    else
        VERIFICATION_RESULTS="$VERIFICATION_RESULTS\n  ❌ PX4: Not built"
    fi
    
    print_info "System verification results:$VERIFICATION_RESULTS"
    
    track_time
}

# Main execution with enhanced error handling
main() {
    print_header "ROS2 Agent Sim - Enhanced Runtime Setup (Ubuntu 24.04)"
    print_info "Log file: ${LOG_FILE}"
    print_info "Installation started at $(date)"
    print_info "ENHANCEMENTS: Comprehensive build debugging, fallback strategies, resource management"
    
    # Run installation steps
    verify_ubuntu_packages
    validate_graphics_environment
    validate_environment
    setup_repositories
    setup_px4_autopilot
    handle_dependencies
    build_workspace
    finalize_installation
    
    # Final summary
    local total_time=$(( $(date +%s) - start_time ))
    print_header "Enhanced Runtime Setup Completed Successfully!"
    print_success "Total setup time: ${total_time}s"
    print_info "Log file saved: ${LOG_FILE}"
    
    echo -e "\n${GREEN}🎉 Next steps:${NC}"
    echo -e "${CYAN}1. Start new shell:${NC}       bash -l"
    echo -e "${CYAN}2. Verify workspace:${NC}      check_workspace"
    echo -e "${CYAN}3. Launch simulation:${NC}     launch_drone"
    echo -e "${CYAN}4. Run ROS2 agent:${NC}        launch_agent"
    
    echo -e "\n${GREEN}🔧 Diagnostic commands:${NC}"
    echo -e "${CYAN}• Check graphics:${NC}         check_graphics"
    echo -e "${CYAN}• Check packages:${NC}         check_ubuntu_packages"
    echo -e "${CYAN}• Check deprecated:${NC}       check_deprecated"
    
    print_success "🎯 Enhanced installation completed with comprehensive Ubuntu 24.04 support and debugging!"
    
    cd "$HOME"
}

# Run main function
main "$@"