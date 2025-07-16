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
# Force UID/GID Mapping 
# ========================================================================

print_info "Setting up FORCED UID/GID mapping..."

# Get host UID/GID from environment variables with fallbacks
HOST_UID=${LOCAL_USER_ID:-1000}
HOST_GID=${LOCAL_GROUP_ID:-1000}
HOST_USER=${HOST_USER:-"user"}
UBUNTU_VERSION=${UBUNTU_VERSION:-"unknown"}

print_info "Host: $HOST_USER (UID: $HOST_UID, GID: $HOST_GID) on Ubuntu $UBUNTU_VERSION"

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
    DISPLAY="$disp" xset q >/dev/null 2>&1
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
    DISPLAY=${DISPLAY:-:0}
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
# Setup Proper .bashrc
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

# X11 aliases for convenience
alias test_x11='xset q && echo "✅ X11 working on \$DISPLAY"'
alias fix_x11='source /opt/scripts/detect_x11_display.sh'
alias start_rviz='test_x11 && rviz2'
EOF
    
    chown "$HOST_UID:$HOST_GID" /home/user/.bashrc
    chmod 644 /home/user/.bashrc
    print_success "New .bashrc installed with X11 environment"
else
    print_warning "Template .bashrc not found, creating minimal version..."
    cat > /home/user/.bashrc << EOF
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

# X11 environment (auto-detected)
export DISPLAY="$DISPLAY"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export QT_XCB_GL_INTEGRATION=none
export QT_QPA_PLATFORM=xcb

echo "🚀 ROS2 Agent Sim Environment Ready"
echo "   DISPLAY: \$DISPLAY"
EOF
    chown "$HOST_UID:$HOST_GID" /home/user/.bashrc
    chmod 644 /home/user/.bashrc
fi

# Set ROS2 Jazzy environment
export ROS_DISTRO="jazzy"
source "/opt/ros/jazzy/setup.bash"

# ========================================================================
# Create X11 Detection Script
# ========================================================================

print_info "Creating X11 detection script..."
mkdir -p /opt/scripts
cat > /opt/scripts/detect_x11_display.sh << 'EOF'
#!/bin/bash
# X11 Display Detection Script

echo "🔍 Detecting working X11 display..."

DISPLAY_CANDIDATES=":1 :0 :10 :2 :1003"
WORKING_DISPLAY=""

for disp in $DISPLAY_CANDIDATES; do
    if DISPLAY="$disp" xset q >/dev/null 2>&1; then
        WORKING_DISPLAY="$disp"
        echo "✅ Found working display: $disp"
        break
    fi
done

if [ -n "$WORKING_DISPLAY" ]; then
    export DISPLAY="$WORKING_DISPLAY"
    export QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_INDIRECT=0
    export QT_XCB_GL_INTEGRATION=none
    export QT_QPA_PLATFORM=xcb
    
    echo "✅ X11 environment updated:"
    echo "   DISPLAY=$DISPLAY"
    echo "   Test with: xset q"
    echo "   Start RViz2 with: rviz2"
else
    echo "❌ No working X11 display found"
    echo "Available X11 sockets:"
    ls -la /tmp/.X11-unix/ 2>/dev/null || echo "No X11 sockets found"
fi
EOF

chmod +x /opt/scripts/detect_x11_display.sh

# ========================================================================
# Shared Volume Setup with Correct Ownership
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

# Instead of using gosu or su with password, use sudo with NOPASSWD
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
    echo \"X11 Test: \$(xset q >/dev/null 2>&1 && echo '✅ Working' || echo '❌ Not Working')\"
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