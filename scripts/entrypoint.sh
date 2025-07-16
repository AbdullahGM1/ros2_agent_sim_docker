#!/bin/bash
# FIXED: Enhanced entrypoint.sh with Ubuntu 24.04 compatibility and comprehensive Qt6/graphics support
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

print_info() { echo -e "${BLUE}[ENTRYPOINT]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_debug() { echo -e "${PURPLE}[DEBUG]${NC} $1"; }

print_info "Starting container initialization with Ubuntu 24.04 compatibility and comprehensive graphics support..."

# ========================================================================
# CRITICAL: Force UID/GID Mapping
# ========================================================================

print_info "Setting up UID/GID mapping..."

# Get host UID/GID from environment variables with fallbacks
HOST_UID=${LOCAL_USER_ID:-1000}
HOST_GID=${LOCAL_GROUP_ID:-1000}
HOST_USER=${HOST_USER:-"user"}

print_info "Host: $HOST_USER (UID: $HOST_UID, GID: $HOST_GID)"

# Check current container user UID/GID
CURRENT_USER_UID=$(id -u user)
CURRENT_USER_GID=$(id -g user)

if [ "$HOST_UID" != "$CURRENT_USER_UID" ] || [ "$HOST_GID" != "$CURRENT_USER_GID" ]; then
    print_info "Updating container user UID/GID to match host..."
    
    # If UID is taken by another user, remove that user first
    if id "$HOST_UID" >/dev/null 2>&1; then
        EXISTING_USER=$(id -nu "$HOST_UID" 2>/dev/null)
        if [ "$EXISTING_USER" != "user" ]; then
            print_warning "UID $HOST_UID is taken by '$EXISTING_USER' - removing conflicting user"
            userdel "$EXISTING_USER" 2>/dev/null || print_warning "Could not remove $EXISTING_USER"
        fi
    fi
    
    # Update user UID/GID to match host exactly
    usermod -u "$HOST_UID" user 2>/dev/null || {
        print_error "CRITICAL: Failed to change user UID to $HOST_UID"
        print_error "This will cause permission issues!"
    }
    
    groupmod -g "$HOST_GID" user 2>/dev/null || {
        print_error "CRITICAL: Failed to change user GID to $HOST_GID"
    }
    
    # Update home directory ownership
    chown -R "$HOST_UID:$HOST_GID" /home/user 2>/dev/null || print_warning "Could not update home ownership"
    
    print_success "User UID/GID updated to: $(id -u user):$(id -g user)"
else
    print_success "User UID/GID already matches host: $CURRENT_USER_UID:$CURRENT_USER_GID"
fi

# ========================================================================
# CRITICAL FIX: Comprehensive X11 Display Auto-Detection
# ========================================================================

print_info "Auto-detecting X11 display environment..."

# Enhanced display detection function
test_display() {
    local disp="$1"
    timeout 5 bash -c "DISPLAY='$disp' xset q" >/dev/null 2>&1
}

# List of displays to try in order of preference (Docker-optimized)
DISPLAY_CANDIDATES=":1 :0 :10 :2 :1003 :11 :12"
WORKING_DISPLAY=""
WORKING_DISPLAYS=""

# Find all working displays
print_debug "Testing available X11 displays..."
for disp in $DISPLAY_CANDIDATES; do
    if test_display "$disp"; then
        WORKING_DISPLAYS="$WORKING_DISPLAYS $disp"
        if [ -z "$WORKING_DISPLAY" ]; then
            WORKING_DISPLAY="$disp"
        fi
        print_debug "Working display found: $disp"
    fi
done

# Set the optimal display
if [ -n "$WORKING_DISPLAY" ]; then
    export DISPLAY="$WORKING_DISPLAY"
    print_success "X11 display auto-detected: $DISPLAY"
    if [ $(echo "$WORKING_DISPLAYS" | wc -w) -gt 1 ]; then
        print_info "Additional working displays: $WORKING_DISPLAYS"
    fi
else
    # Fallback display selection
    FALLBACK_DISPLAY="${DISPLAY:-:1}"
    export DISPLAY="$FALLBACK_DISPLAY"
    print_warning "No working X11 display detected, using fallback: $DISPLAY"
fi

print_info "Final DISPLAY setting: $DISPLAY"

# ========================================================================
# CRITICAL FIX: Qt6 and Graphics Environment Setup (Ubuntu 24.04)
# ========================================================================

print_info "Setting up comprehensive Qt6 and graphics environment for Ubuntu 24.04..."

# CRITICAL FIX: Qt6 Environment Variables (prevents Qt5/Qt6 conflicts in Ubuntu 24.04)
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

# Set Qt6 plugin paths explicitly for Ubuntu 24.04
QT6_PLUGIN_PATHS="/usr/lib/x86_64-linux-gnu/qt6/plugins:/usr/lib/qt6/plugins"
export QT_QPA_PLATFORM_PLUGIN_PATH="$QT6_PLUGIN_PATHS/platforms"
export QT_PLUGIN_PATH="$QT6_PLUGIN_PATHS"

# CRITICAL FIX: OpenGL and Mesa Environment (comprehensive rendering support for Ubuntu 24.04)
export LIBGL_ALWAYS_INDIRECT=0
export LIBGL_ALWAYS_SOFTWARE=0
export MESA_GL_VERSION_OVERRIDE="4.5"
export MESA_GLSL_VERSION_OVERRIDE="450" 
export GALLIUM_DRIVER="llvmpipe"

# Advanced graphics environment
export XDG_RUNTIME_DIR="/tmp/runtime-user"
export XDG_SESSION_TYPE="x11"
export WAYLAND_DISPLAY=""

# Additional Mesa and DRI configuration for Ubuntu 24.04
export MESA_LOADER_DRIVER_OVERRIDE="llvmpipe"
export EGL_PLATFORM="x11"

print_success "Qt6 and graphics environment configured for Ubuntu 24.04"

# ========================================================================
# CRITICAL FIX: X11 Authentication Setup  
# ========================================================================

print_info "Setting up X11 authentication..."

# Fix XAUTH if provided
if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
    print_debug "Using provided XAUTH file: $XAUTHORITY"
    
    # Check if XAUTH file has content
    if [ ! -s "$XAUTHORITY" ]; then
        print_warning "XAUTH file is empty, creating authentication entries"
        
        # Generate authentication entries
        COOKIE=$(openssl rand -hex 32 2>/dev/null || mcookie)
        
        # Add entries for current display and localhost variants
        xauth -f "$XAUTHORITY" add "$DISPLAY" . "$COOKIE" 2>/dev/null || true
        xauth -f "$XAUTHORITY" add "localhost$DISPLAY" . "$COOKIE" 2>/dev/null || true
        xauth -f "$XAUTHORITY" add "unix$DISPLAY" . "$COOKIE" 2>/dev/null || true
        
        # Fix ownership
        chown "$HOST_UID:$HOST_GID" "$XAUTHORITY" 2>/dev/null || true
        chmod 600 "$XAUTHORITY" 2>/dev/null || true
        
        print_success "Generated X11 authentication entries"
    else
        print_success "Using existing X11 authentication"
    fi
    
    # Verify XAUTH content
    AUTH_COUNT=$(xauth -f "$XAUTHORITY" list 2>/dev/null | wc -l || echo "0")
    print_debug "X11 auth entries: $AUTH_COUNT"
else
    print_warning "No XAUTHORITY provided, X11 apps may have authentication issues"
fi

# Test X11 connection with comprehensive fallback
print_info "Testing X11 connection..."
if test_display "$DISPLAY"; then
    print_success "✅ X11 connection verified for $DISPLAY"
    
    # Get X11 server information
    X11_INFO=$(timeout 3 xset q 2>/dev/null | grep -E "(X.Org|version)" | head -1 | xargs || echo "X11 Server")
    print_debug "X11 Server: $X11_INFO"
else
    print_warning "⚠️ X11 connection test failed for $DISPLAY"
    print_info "GUI applications will attempt software rendering fallbacks"
fi

# ========================================================================
# CRITICAL FIX: Graphics Runtime Directory Setup
# ========================================================================

print_info "Setting up graphics runtime environment..."

# Create and configure XDG runtime directory
if [ ! -d "/tmp/runtime-user" ]; then
    mkdir -p /tmp/runtime-user
    chmod 700 /tmp/runtime-user
    print_debug "Created XDG runtime directory"
fi
chown -R $HOST_UID:$HOST_GID /tmp/runtime-user 2>/dev/null || true

# Create Qt6 configuration directory for user
mkdir -p /home/user/.config/qt6
chown -R $HOST_UID:$HOST_GID /home/user/.config 2>/dev/null || true

# Set up additional graphics directories
mkdir -p /home/user/.cache/mesa_shader_cache
chown -R $HOST_UID:$HOST_GID /home/user/.cache 2>/dev/null || true

print_success "Graphics runtime environment setup completed"

# ========================================================================
# CRITICAL FIX: OpenGL Environment Testing and Configuration (Ubuntu 24.04)
# ========================================================================

print_info "Testing and configuring OpenGL environment for Ubuntu 24.04..."

# Test OpenGL availability
if command -v glxinfo >/dev/null 2>&1; then
    print_debug "Testing OpenGL capabilities..."
    
    # Test hardware OpenGL
    if timeout 10 glxinfo -B >/dev/null 2>&1; then
        RENDERER=$(glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Unknown")
        GL_VERSION=$(glxinfo -B 2>/dev/null | grep "OpenGL version" | cut -d: -f2 | xargs || echo "Unknown")
        print_success "✅ Hardware OpenGL available: $RENDERER"
        print_debug "OpenGL Version: $GL_VERSION"
        
        # Check if it's software rendering
        if echo "$RENDERER" | grep -qi "llvmpipe\|softpipe\|swrast"; then
            print_info "Software rendering detected, configuring optimizations..."
            export GALLIUM_DRIVER="llvmpipe"
            export MESA_GL_VERSION_OVERRIDE="4.5"
        fi
    else
        print_warning "Hardware OpenGL failed, testing software rendering..."
        
        # Test software OpenGL
        if LIBGL_ALWAYS_SOFTWARE=1 timeout 10 glxinfo -B >/dev/null 2>&1; then
            SOFT_RENDERER=$(LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Software")
            print_success "✅ Software OpenGL available: $SOFT_RENDERER"
            
            # Configure for software rendering
            export LIBGL_ALWAYS_SOFTWARE=0  # Don't force it globally, let apps choose
            export GALLIUM_DRIVER="llvmpipe"
            print_info "Software rendering configured as fallback"
        else
            print_error "❌ Both hardware and software OpenGL failed"
            print_warning "Graphics applications may not work properly"
        fi
    fi
else
    print_warning "⚠️ glxinfo not available, cannot test OpenGL"
fi

# ========================================================================
# CRITICAL FIX: Ubuntu 24.04 Package Verification
# ========================================================================

print_info "Verifying Ubuntu 24.04 package compatibility..."

# Check for critical packages that were changed in Ubuntu 24.04
PACKAGE_ISSUES=""

# Check libgl1 vs libgl1-mesa-glx
if ! dpkg -l | grep -q "^ii.*libgl1[^-]"; then
    PACKAGE_ISSUES="$PACKAGE_ISSUES libgl1(missing)"
fi

# Check libglx-mesa0
if ! dpkg -l | grep -q "^ii.*libglx-mesa0"; then
    PACKAGE_ISSUES="$PACKAGE_ISSUES libglx-mesa0(missing)"
fi

# Check libglut3.12 vs freeglut3
if ! dpkg -l | grep -q "^ii.*libglut3.12"; then
    PACKAGE_ISSUES="$PACKAGE_ISSUES libglut3.12(missing)"
fi

# Check Qt6 packages
if ! dpkg -l | grep -q "^ii.*qt6-base"; then
    PACKAGE_ISSUES="$PACKAGE_ISSUES qt6-base(missing)"
fi

if [ -n "$PACKAGE_ISSUES" ]; then
    print_warning "Ubuntu 24.04 package issues detected: $PACKAGE_ISSUES"
    print_info "These may cause graphics issues but fallbacks are configured"
else
    print_success "✅ Ubuntu 24.04 package compatibility verified"
fi

# ========================================================================
# CRITICAL FIX: Gazebo-Specific Environment Setup (Ubuntu 24.04)
# ========================================================================

print_info "Configuring Gazebo Harmonic environment for Ubuntu 24.04..."

# Gazebo-specific environment variables
export GAZEBO_MODEL_PATH="/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/models:$GAZEBO_MODEL_PATH"
export GAZEBO_RESOURCE_PATH="/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds:$GAZEBO_RESOURCE_PATH"
export IGN_GAZEBO_RESOURCE_PATH="/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/models:/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds:$IGN_GAZEBO_RESOURCE_PATH"

# Gazebo GUI optimizations for containers
export GZ_GUI_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins:$GZ_GUI_PLUGIN_PATH"

# CRITICAL FIX: Ogre2 rendering configuration for Gazebo (Ubuntu 24.04)
export OGRE_RTT_MODE="Copy"  # Helps with rendering in containers
export OGRE_CONFIG_PATH="/home/user/.ogre"

print_success "Gazebo environment configured for Ubuntu 24.04"

# ========================================================================
# CRITICAL FIX: Enhanced Bashrc Setup (Ubuntu 24.04)
# ========================================================================

print_info "Setting up enhanced user environment for Ubuntu 24.04..."

# Backup existing bashrc
if [ -f "/home/user/.bashrc" ]; then
    cp /home/user/.bashrc /home/user/.bashrc.backup.$(date +%s)
    print_debug "Backed up existing .bashrc"
fi

# Install enhanced bashrc template
if [ -f "/opt/bashrc_templates/bashrc_template.sh" ]; then
    print_info "Installing enhanced .bashrc template with Ubuntu 24.04 graphics support..."
    cp /opt/bashrc_templates/bashrc_template.sh /home/user/.bashrc
    
    # Add container-specific environment to bashrc
    cat >> /home/user/.bashrc << EOF

# =============================================================================
# Container-Specific Environment (Auto-added by entrypoint - Ubuntu 24.04)
# =============================================================================

# CRITICAL FIX: Container graphics environment
export DISPLAY="$DISPLAY"
export XAUTHORITY="$XAUTHORITY"
export QT_QPA_PLATFORM="$QT_QPA_PLATFORM"
export QT_X11_NO_MITSHM="$QT_X11_NO_MITSHM"
export LIBGL_ALWAYS_INDIRECT="$LIBGL_ALWAYS_INDIRECT"
export MESA_GL_VERSION_OVERRIDE="$MESA_GL_VERSION_OVERRIDE"
export GALLIUM_DRIVER="$GALLIUM_DRIVER"

# Container runtime paths
export XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR"
export QT_PLUGIN_PATH="$QT_PLUGIN_PATH"

# Gazebo optimizations
export GZ_GUI_PLUGIN_PATH="$GZ_GUI_PLUGIN_PATH"
export OGRE_RTT_MODE="$OGRE_RTT_MODE"

# Auto-enable graphics testing on container start
export AUTO_TEST_GRAPHICS=false  # Set to true for automatic testing

# Welcome message
echo ""
echo "🎉 ROS2 Agent Sim Container Ready (FIXED VERSION - Ubuntu 24.04)"
echo "   📊 Graphics: Qt6 + OpenGL configured for Ubuntu 24.04"
echo "   🖥️  Display: $DISPLAY"
echo "   🎮 Rendering: Hardware + Software fallback"
echo "   🚁 Gazebo: Harmonic with Ogre2 support"
echo "   📦 Packages: Ubuntu 24.04 compatible (libgl1 + libglx-mesa0 + libglut3.12)"
echo ""

EOF
    
    chown "$HOST_UID:$HOST_GID" /home/user/.bashrc
    chmod 644 /home/user/.bashrc
    print_success "Enhanced .bashrc installed with Ubuntu 24.04 graphics environment"
else
    print_warning "Template .bashrc not found, creating Ubuntu 24.04 compatible minimal version..."
    cat > /home/user/.bashrc << EOF
# Minimal .bashrc for ROS2 Agent Sim (Emergency Fallback - Ubuntu 24.04)

# Basic bash settings
export HISTCONTROL=ignoreboth
shopt -s histappend
export HISTSIZE=1000
export HISTFILESIZE=2000
PS1='\u@\h:\w\$ '

# Development environment
export DEV_DIR="/home/user/shared_volume"
export PX4_DIR="\$DEV_DIR/PX4-Autopilot"
export ROS2_WS="\$DEV_DIR/ros2_ws"
export OSQP_SRC="\$DEV_DIR"
export PATH="\$HOME/.local/bin:\$PATH"

# ROS2 Jazzy setup
export ROS_DISTRO="jazzy"
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
fi

# Source workspace setup if it exists
if [ -f "\$ROS2_WS/install/setup.bash" ]; then
    source "\$ROS2_WS/install/setup.bash"
fi

# Gazebo environment
export GZ_VERSION="harmonic"

# CRITICAL FIX: Graphics environment for Ubuntu 24.04
export DISPLAY="$DISPLAY"
export QT_QPA_PLATFORM="$QT_QPA_PLATFORM"
export QT_X11_NO_MITSHM="$QT_X11_NO_MITSHM"
export LIBGL_ALWAYS_INDIRECT="$LIBGL_ALWAYS_INDIRECT"
export MESA_GL_VERSION_OVERRIDE="$MESA_GL_VERSION_OVERRIDE"

echo "🚀 ROS2 Agent Sim Environment Ready (Minimal - Ubuntu 24.04)"
EOF
    chown "$HOST_UID:$HOST_GID" /home/user/.bashrc
    chmod 644 /home/user/.bashrc
    print_warning "Minimal .bashrc created for Ubuntu 24.04"
fi

# Set ROS2 environment
export ROS_DISTRO="jazzy"
source "/opt/ros/jazzy/setup.bash" || print_warning "Could not source ROS2 setup"

# ========================================================================
# CRITICAL FIX: Shared Volume Setup with Correct Ownership
# ========================================================================

print_info "Setting up shared volume with comprehensive ownership management..."

if [ -d "/home/user/shared_volume" ]; then
    # Fix shared volume directory ownership
    print_debug "Fixing shared volume directory ownership..."
    chown $HOST_UID:$HOST_GID /home/user/shared_volume/ 2>/dev/null || {
        print_warning "Cannot change shared volume directory ownership (may be expected for some mount types)"
    }
    chmod 755 /home/user/shared_volume/ 2>/dev/null || true
    
    # Create subdirectories with correct ownership
    mkdir -p /home/user/shared_volume/ros2_ws/src
    mkdir -p /home/user/shared_volume/.cache
    mkdir -p /home/user/shared_volume/.config
    chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/ros2_ws/ 2>/dev/null || true
    chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/.cache/ 2>/dev/null || true
    chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/.config/ 2>/dev/null || true
    
    # Copy installation files if they don't exist
    if [ ! -f "/home/user/shared_volume/install.sh" ] && [ -f "/home/user/backup/install.sh" ]; then
        print_debug "Copying install.sh with correct ownership..."
        cp /home/user/backup/install.sh /home/user/shared_volume/
        chmod +x /home/user/shared_volume/install.sh
        chown $HOST_UID:$HOST_GID /home/user/shared_volume/install.sh
        print_success "install.sh copied and configured"
    fi
    
    if [ ! -d "/home/user/shared_volume/PX4_config" ] && [ -d "/home/user/backup/PX4_config" ]; then
        print_debug "Copying PX4_config with correct ownership..."
        cp -r /home/user/backup/PX4_config /home/user/shared_volume/
        chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/PX4_config/
        print_success "PX4_config copied and configured"
    fi
    
    # Apply comprehensive ownership fix
    print_debug "Applying comprehensive ownership fix..."
    find /home/user/shared_volume -type d -exec chmod 755 {} \; 2>/dev/null || true
    find /home/user/shared_volume -type f -exec chmod 644 {} \; 2>/dev/null || true
    find /home/user/shared_volume -name "*.sh" -exec chmod +x {} \; 2>/dev/null || true
    
    # Try to fix ownership (may fail on some mount types, which is normal)
    chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/ 2>/dev/null || {
        print_warning "Some ownership changes failed (normal for certain mount types)"
    }
    
    print_success "Shared volume setup completed"
else
    print_warning "Shared volume directory not found!"
fi

# ========================================================================
# Service Setup
# ========================================================================

# Start Ollama if port is free
if ! netstat -tuln 2>/dev/null | grep -q ":11434 "; then
    print_info "Starting Ollama service..."
    sudo -u user ollama serve &
    sleep 3
    print_success "Ollama service started"
else
    print_info "Port 11434 in use, skipping Ollama startup"
fi

# ========================================================================
# CRITICAL FIX: Final Environment Verification (Ubuntu 24.04)
# ========================================================================

print_info "Performing final environment verification for Ubuntu 24.04..."

# Verify Qt6 environment
if [ -n "$QT_QPA_PLATFORM" ] && [ "$QT_QPA_PLATFORM" = "xcb" ]; then
    print_success "✅ Qt6 platform configured: $QT_QPA_PLATFORM"
else
    print_warning "⚠️ Qt6 platform configuration issue"
fi

# Verify OpenGL environment
if [ -n "$MESA_GL_VERSION_OVERRIDE" ]; then
    print_success "✅ OpenGL version override: $MESA_GL_VERSION_OVERRIDE"
else
    print_warning "⚠️ OpenGL configuration missing"
fi

# Verify X11 environment
if [ -n "$DISPLAY" ]; then
    print_success "✅ X11 display configured: $DISPLAY"
else
    print_error "❌ X11 display not configured"
fi

# Verify Ubuntu 24.04 specific packages
UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
print_info "Ubuntu version detected: $UBUNTU_VERSION"

if [ "$UBUNTU_VERSION" = "24.04" ]; then
    print_success "✅ Ubuntu 24.04 confirmed - using correct package configuration"
else
    print_warning "⚠️ Ubuntu version may not be 24.04 - package compatibility uncertain"
fi

# ========================================================================
# Switch to User Context and Execute Command
# ========================================================================

print_info "Switching to user context and executing command..."

# For persistent container mode
if [ "$1" = "tail" ] && [ "$2" = "-f" ] && [ "$3" = "/dev/null" ]; then
    print_success "Container initialized successfully in persistent mode"
    print_info "Container will keep running in background"
    print_info "Use 'docker exec -it ros2_agent_sim bash' to connect"
    
    # Final status report
    print_info "=== CONTAINER INITIALIZATION COMPLETE (Ubuntu 24.04) ==="
    print_info "User: user (UID: $(id -u user), GID: $(id -g user))"
    print_info "ROS_DISTRO: $ROS_DISTRO"
    print_info "DISPLAY: $DISPLAY"
    print_info "Qt6 Platform: $QT_QPA_PLATFORM"
    print_info "OpenGL: Mesa $MESA_GL_VERSION_OVERRIDE (Gallium: $GALLIUM_DRIVER)"
    print_info "X11 Status: $(timeout 3 xset q >/dev/null 2>&1 && echo '✅ Working' || echo '⚠️ Fallback Mode')"
    print_info "Shared Volume: $([ -d '/home/user/shared_volume' ] && echo '✅ Ready' || echo '❌ Not Found')"
    print_info "Graphics: Qt6 + OpenGL + Software Rendering Fallbacks (Ubuntu 24.04)"
    print_info "Packages: libgl1 + libglx-mesa0 + libglut3.12 (Ubuntu 24.04 compatible)"
    print_info "================================================="
    
    exec "$@"
fi

# For interactive mode
print_info "Switching to user context for interactive mode..."
exec sudo -u user -H bash -c "
    # Load enhanced environment
    if [ -f ~/.bashrc ]; then
        source ~/.bashrc
        echo '✅ Enhanced .bashrc loaded successfully (Ubuntu 24.04)'
        echo '🖥️  Graphics Environment:'
        echo '   Display: \$DISPLAY'
        echo '   Qt6 Platform: \$QT_QPA_PLATFORM'
        echo '   OpenGL: Mesa \$MESA_GL_VERSION_OVERRIDE'
        echo '   Renderer: \$GALLIUM_DRIVER'
        echo '   Ubuntu: 24.04 compatible packages'
    else
        echo '❌ .bashrc not found!'
        # Create emergency environment
        export ROS_DISTRO=jazzy
        export DEV_DIR=/home/user/shared_volume
        export ROS2_WS=\$DEV_DIR/ros2_ws
        export DISPLAY=$DISPLAY
        export QT_QPA_PLATFORM=$QT_QPA_PLATFORM
        export QT_X11_NO_MITSHM=$QT_X11_NO_MITSHM
        export LIBGL_ALWAYS_INDIRECT=$LIBGL_ALWAYS_INDIRECT
        export MESA_GL_VERSION_OVERRIDE=$MESA_GL_VERSION_OVERRIDE
        export GALLIUM_DRIVER=$GALLIUM_DRIVER
        source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    fi
    
    # Change to shared volume directory
    if [ -d '/home/user/shared_volume' ] && [ -w '/home/user/shared_volume' ]; then
        cd /home/user/shared_volume
        echo '✅ Successfully in shared volume with write access'
    else
        echo '⚠️  Shared volume not accessible, staying in home directory'
        cd /home/user
    fi
    
    # Final status check
    echo ''
    echo '=== INTERACTIVE CONTAINER READY (Ubuntu 24.04) ==='
    echo \"Current directory: \$(pwd)\"
    echo \"User: \$(whoami) (UID: \$(id -u), GID: \$(id -g))\"
    echo \"ROS_DISTRO: \$ROS_DISTRO\"
    echo \"DISPLAY: \$DISPLAY\"
    echo \"X11 Test: \$(timeout 5 xset q >/dev/null 2>&1 && echo '✅ Working' || echo '⚠️ Fallback Mode')\"
    echo \"OpenGL: \$(command -v glxinfo >/dev/null && echo '✅ Available' || echo '⚠️ Limited')\"
    echo \"Qt6: \$(echo \$QT_QPA_PLATFORM | grep -q xcb && echo '✅ Ready' || echo '⚠️ Basic')\"
    echo \"AMENT_PREFIX_PATH: \${AMENT_PREFIX_PATH:-'Not set'}\"
    echo \"Ubuntu 24.04 packages: \$(dpkg -l 2>/dev/null | grep -c 'libgl1\\|libglx-mesa0\\|libglut3.12' || echo '0') installed\"
    
    if [ -f 'install.sh' ]; then
        echo \"install.sh: \$(ls -l install.sh | awk '{print \$1, \$3, \$4}')\"
        echo \"Can edit install.sh: \$([ -w install.sh ] && echo 'YES ✅' || echo 'NO ❌')\"
    fi
    
    echo '=================================================='
    echo ''
    echo '🎯 Ready to use (Ubuntu 24.04 compatible):'
    echo '   • ros2 launch drone_sim drone.launch.py'
    echo '   • ros2 run ros2_agent ros2_agent_node'
    echo '   • start_gazebo (enhanced with Ubuntu 24.04 fallbacks)'
    echo '   • diagnose_graphics (if issues occur)'
    echo '   • check_packages (verify Ubuntu 24.04 packages)'
    echo ''
    
    # Execute the command
    exec \"\$@\"
" -- "$@"