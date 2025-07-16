#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() { echo -e "${BLUE}[ENTRYPOINT]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

print_info "Starting container initialization..."

# ========================================================================
# CRITICAL: Force UID/GID Mapping - No Alternatives
# ========================================================================

print_info "Setting up FORCED UID/GID mapping..."

# Get host UID/GID from environment variables with fallbacks
HOST_UID=${LOCAL_USER_ID:-1000}
HOST_GID=${LOCAL_GROUP_ID:-1000}
HOST_USER=${HOST_USER:-"user"}

print_info "Host: $HOST_USER (UID: $HOST_UID, GID: $HOST_GID)"

# Check current container user UID/GID
CURRENT_USER_UID=$(id -u user)
CURRENT_USER_GID=$(id -g user)

if [ "$HOST_UID" != "$CURRENT_USER_UID" ] || [ "$HOST_GID" != "$CURRENT_USER_GID" ]; then
    print_info "FORCING container user UID/GID to match host..."
    
    # If UID is taken by another user, REMOVE that user first
    if id "$HOST_UID" >/dev/null 2>&1; then
        EXISTING_USER=$(id -nu "$HOST_UID" 2>/dev/null)
        if [ "$EXISTING_USER" != "user" ]; then
            print_warning "UID $HOST_UID is taken by '$EXISTING_USER' - REMOVING conflicting user"
            userdel "$EXISTING_USER" 2>/dev/null || print_warning "Could not remove $EXISTING_USER"
        fi
    fi
    
    # FORCE update user UID/GID to match host exactly
    usermod -u "$HOST_UID" user 2>/dev/null || {
        print_error "CRITICAL: Failed to change user UID to $HOST_UID"
        print_error "This will cause permission issues!"
    }
    
    groupmod -g "$HOST_GID" user 2>/dev/null || {
        print_error "CRITICAL: Failed to change user GID to $HOST_GID"
    }
    
    # Update home directory ownership
    chown -R "$HOST_UID:$HOST_GID" /home/user 2>/dev/null || print_warning "Could not update home ownership"
    
    print_success "User UID/GID FORCED to: $(id -u user):$(id -g user)"
else
    print_success "User UID/GID already matches host: $CURRENT_USER_UID:$CURRENT_USER_GID"
fi

# ========================================================================
# AUTO-DETECT WORKING X11 DISPLAY
# ========================================================================

print_info "Auto-detecting working X11 display..."

# Function to test X11 display
test_display() {
    local disp="$1"
    timeout 5 sh -c "DISPLAY='$disp' xset q" >/dev/null 2>&1
}

# List of displays to try in order of preference
DISPLAY_CANDIDATES=":1 :0 :10 :2 :1003"
WORKING_DISPLAY=""

# Find working display
for disp in $DISPLAY_CANDIDATES; do
    if test_display "$disp"; then
        WORKING_DISPLAY="$disp"
        print_success "Found working X11 display: $disp"
        break
    fi
done

# Set the working display or fallback
if [ -n "$WORKING_DISPLAY" ]; then
    export DISPLAY="$WORKING_DISPLAY"
    print_success "Set DISPLAY=$WORKING_DISPLAY"
else
    # Fallback to environment or default
    DISPLAY=${DISPLAY:-:1}
    print_warning "No working X11 display found, using fallback: $DISPLAY"
    export DISPLAY
fi

# ========================================================================
# Setup X11 Environment Variables
# ========================================================================

print_info "Setting up X11 environment..."

# Set Qt environment for GUI applications
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export QT_XCB_GL_INTEGRATION=none
export QT_QPA_PLATFORM=xcb

# Fix XAUTH if needed
if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
    if [ ! -s "$XAUTHORITY" ]; then
        print_warning "XAUTH file is empty, creating authentication"
        COOKIE=$(mcookie 2>/dev/null || openssl rand -hex 16)
        xauth -f "$XAUTHORITY" add "$DISPLAY" . "$COOKIE" 2>/dev/null || true
        xauth -f "$XAUTHORITY" add "localhost$DISPLAY" . "$COOKIE" 2>/dev/null || true
        chown "$HOST_UID:$HOST_GID" "$XAUTHORITY" 2>/dev/null || true
    fi
    print_success "X11 authentication configured"
fi

# Test X11 connection
if test_display "$DISPLAY"; then
    print_success "✅ X11 connection verified for $DISPLAY"
else
    print_warning "⚠️ X11 connection not working for $DISPLAY"
fi

print_success "X11 environment setup completed"

# ========================================================================
# CRITICAL: Graphics and OpenGL Environment Setup
# ========================================================================

print_info "Setting up graphics and OpenGL environment..."

# Create XDG runtime directory if it doesn't exist
if [ ! -d "/tmp/runtime-user" ]; then
    mkdir -p /tmp/runtime-user
    chmod 700 /tmp/runtime-user
fi
chown -R $HOST_UID:$HOST_GID /tmp/runtime-user

# Set graphics environment variables
export XDG_RUNTIME_DIR="/tmp/runtime-user"
export XDG_SESSION_TYPE="x11"
export MESA_GL_VERSION_OVERRIDE="3.3"
export MESA_GLSL_VERSION_OVERRIDE="330"
export LIBGL_ALWAYS_INDIRECT=0
export LIBGL_ALWAYS_SOFTWARE=0

# Test OpenGL availability
if command -v glxinfo >/dev/null 2>&1; then
    if glxinfo >/dev/null 2>&1; then
        print_success "✅ OpenGL hardware acceleration available"
    else
        print_warning "⚠️ OpenGL hardware acceleration not available, using software rendering"
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=softpipe
    fi
else
    print_warning "⚠️ glxinfo not available, cannot test OpenGL"
fi

# Gazebo-specific environment variables
export GAZEBO_MODEL_PATH="/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/models:$GAZEBO_MODEL_PATH"
export GAZEBO_RESOURCE_PATH="/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds:$GAZEBO_RESOURCE_PATH"
export IGN_GAZEBO_RESOURCE_PATH="/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/models:/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds:$IGN_GAZEBO_RESOURCE_PATH"

print_success "Graphics environment setup completed"

# ========================================================================
# CRITICAL: Setup Proper .bashrc
# ========================================================================

print_info "Setting up proper .bashrc configuration..."

# Remove any existing broken bashrc entries
if [ -f "/home/user/.bashrc" ]; then
    print_info "Backing up existing .bashrc..."
    cp /home/user/.bashrc /home/user/.bashrc.backup.$(date +%s)
fi

# Use the template bashrc that doesn't reference non-existent paths
if [ -f "/opt/bashrc_templates/bashrc_template.sh" ]; then
    print_info "Installing proper .bashrc template..."
    cp /opt/bashrc_templates/bashrc_template.sh /home/user/.bashrc
    
    # Add X11 environment to bashrc
    cat >> /home/user/.bashrc << EOF

# X11 environment for GUI applications (auto-detected)
export DISPLAY="$DISPLAY"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export QT_XCB_GL_INTEGRATION=none
export QT_QPA_PLATFORM=xcb
EOF
    
    chown "$HOST_UID:$HOST_GID" /home/user/.bashrc
    chmod 644 /home/user/.bashrc
    print_success "New .bashrc installed with X11 environment"
else
    print_warning "Template .bashrc not found, creating minimal version..."
    cat > /home/user/.bashrc << 'EOF'
# Minimal .bashrc for ROS2 Agent Sim

# Basic bash settings
export HISTCONTROL=ignoreboth
shopt -s histappend
export HISTSIZE=1000
export HISTFILESIZE=2000

# Set a basic prompt
PS1='\u@\h:\w\$ '

# Development environment
export DEV_DIR="/home/user/shared_volume"
export PX4_DIR="$DEV_DIR/PX4-Autopilot"
export ROS2_WS="$DEV_DIR/ros2_ws"
export OSQP_SRC="$DEV_DIR"
export PATH="$HOME/.local/bin:$PATH"

# ROS2 Jazzy setup
export ROS_DISTRO="jazzy"
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
fi

# Source workspace setup if it exists
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
fi

# Gazebo environment
export GZ_VERSION="harmonic"

echo "🚀 ROS2 Agent Sim Environment Ready"
EOF
    chown "$HOST_UID:$HOST_GID" /home/user/.bashrc
    chmod 644 /home/user/.bashrc
fi

# Set ROS2 Jazzy environment
export ROS_DISTRO="jazzy"
source "/opt/ros/jazzy/setup.bash"

# ========================================================================
# CRITICAL: Shared Volume Setup with Correct Ownership
# ========================================================================

print_info "Setting up shared volume with correct ownership..."

if [ -d "/home/user/shared_volume" ]; then
    # FIRST: Fix the shared volume directory ownership (CRITICAL!)
    print_info "Fixing shared volume directory ownership..."
    chown $HOST_UID:$HOST_GID /home/user/shared_volume/ 2>/dev/null || {
        print_warning "Cannot change shared volume directory ownership"
    }
    chmod 755 /home/user/shared_volume/ 2>/dev/null || true
    
    # Create subdirectories with correct ownership
    mkdir -p /home/user/shared_volume/ros2_ws/src
    chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/ros2_ws/ 2>/dev/null || true
    
    # Copy files if they don't exist (as root, then fix ownership)
    if [ ! -f "/home/user/shared_volume/install.sh" ] && [ -f "/home/user/backup/install.sh" ]; then
        print_info "Copying install.sh with correct ownership..."
        cp /home/user/backup/install.sh /home/user/shared_volume/
        chmod +x /home/user/shared_volume/install.sh
        chown $HOST_UID:$HOST_GID /home/user/shared_volume/install.sh
        print_success "install.sh copied with correct ownership"
    fi
    
    if [ ! -d "/home/user/shared_volume/PX4_config" ] && [ -d "/home/user/backup/PX4_config" ]; then
        print_info "Copying PX4_config with correct ownership..."
        cp -r /home/user/backup/PX4_config /home/user/shared_volume/
        chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/PX4_config/
        print_success "PX4_config copied with correct ownership"
    fi
    
    # COMPREHENSIVE ownership fix for entire shared volume
    print_info "Applying comprehensive ownership fix..."
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
    ollama serve &
    sleep 3
else
    print_info "Port 11434 in use, skipping Ollama startup"
fi

# ========================================================================
# Switch to User Context WITHOUT password prompt (FIXED)
# ========================================================================

print_info "Switching to user context and executing command..."

# For persistent container mode, check if we should just keep running
if [ "$1" = "tail" ] && [ "$2" = "-f" ] && [ "$3" = "/dev/null" ]; then
    print_success "Container initialized successfully in persistent mode"
    print_info "Container will keep running in background"
    print_info "Use 'docker exec -it ros2_agent_sim bash' to connect"
    
    # Final status check
    print_info "=== CONTAINER READY ==="
    print_info "User: user (UID: $(id -u user), GID: $(id -g user))"
    print_info "ROS_DISTRO: $ROS_DISTRO"
    print_info "DISPLAY: $DISPLAY"
    print_info "X11 Test: $(test_display "$DISPLAY" && echo '✅ Working' || echo '❌ Not Working')"
    print_info "Shared Volume: $([ -d '/home/user/shared_volume' ] && echo '✅ Ready' || echo '❌ Not Found')"
    print_info "==================="
    
    exec "$@"
fi

# For interactive mode, switch to user and run command
print_info "Switching to user context for interactive mode..."
exec sudo -u user -H bash -c "
    # Test that .bashrc is working
    if [ -f ~/.bashrc ]; then
        source ~/.bashrc
        echo '✅ .bashrc sourced successfully'
        echo '🖥️  X11 Environment: DISPLAY=\$DISPLAY'
    else
        echo '❌ .bashrc not found!'
        # Create emergency bashrc
        echo 'export ROS_DISTRO=jazzy' > ~/.bashrc
        echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
        echo 'export DEV_DIR=/home/user/shared_volume' >> ~/.bashrc
        echo 'export ROS2_WS=\$DEV_DIR/ros2_ws' >> ~/.bashrc
        echo 'export DISPLAY=$DISPLAY' >> ~/.bashrc
        echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc
        source ~/.bashrc
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
    echo '=== CONTAINER READY ==='
    echo \"Current directory: \$(pwd)\"
    echo \"User: \$(whoami) (UID: \$(id -u), GID: \$(id -g))\"
    echo \"ROS_DISTRO: \$ROS_DISTRO\"
    echo \"DISPLAY: \$DISPLAY\"
    echo \"X11 Test: \$(timeout 5 xset q >/dev/null 2>&1 && echo '✅ Working' || echo '❌ Not Working')\"
    echo \"AMENT_PREFIX_PATH: \${AMENT_PREFIX_PATH:-'Not set'}\"
    if [ -f 'install.sh' ]; then
        echo \"install.sh: \$(ls -l install.sh | awk '{print \$1, \$3, \$4}')\"
        echo \"Can edit install.sh: \$([ -w install.sh ] && echo 'YES ✅' || echo 'NO ❌')\"
    fi
    if [ -f ~/.bashrc ]; then
        echo \"Bashrc exists: YES ✅ (Owner: \$(stat -c '%U:%G' ~/.bashrc))\"
    else
        echo \"Bashrc exists: NO ❌\"
    fi
    echo '==================='
    
    # Execute the command
    exec \"\$@\"
" -- "$@"