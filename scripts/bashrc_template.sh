#!/bin/bash
# FIXED: ~/.bashrc for ROS2 Jazzy + Gazebo Harmonic with Qt6 and Graphics Support (Ubuntu 24.04)

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# Force color prompt - enhanced for Docker containers
force_color_prompt=yes
color_prompt=yes

# Ensure proper TERM setting for colors
if [ -z "$TERM" ] || [ "$TERM" = "dumb" ]; then
    export TERM=xterm-256color
fi

# Force color support - critical for Docker containers
export COLORTERM=truecolor
export FORCE_COLOR=1
export CLICOLOR_FORCE=1

# Enhanced color prompt configuration
if [ "$color_prompt" = yes ]; then
    # Colorful prompt with Docker-optimized colors
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    # Fallback prompt
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# =============================================================================
# ROS2 and Development Environment Setup
# =============================================================================

# Set development directory
export DEV_DIR="/home/user/shared_volume"
export PX4_DIR="$DEV_DIR/PX4-Autopilot"
export ROS2_WS="$DEV_DIR/ros2_ws"
export OSQP_SRC="$DEV_DIR"

# Add Python local bin to PATH
export PATH="$HOME/.local/bin:$PATH"

# ROS2 Jazzy setup
export ROS_DISTRO="jazzy"
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
fi

# Source workspace setup if it exists (runtime workspace)
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
fi

# Source built-in workspace setup if it exists (container workspace)
if [ -f "/home/user/ros2_ws/install/setup.bash" ]; then
    source "/home/user/ros2_ws/install/setup.bash"
fi

# Gazebo environment
export GZ_VERSION="harmonic"

# Gazebo simulation paths for PX4
export GZ_SIM_RESOURCE_PATH=/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/models:/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# =============================================================================
# CRITICAL FIX: Qt6 and Graphics Environment Setup (Ubuntu 24.04)
# =============================================================================

# CRITICAL FIX: Qt6 Environment Variables (resolves Qt5/Qt6 conflicts)
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1
export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt6/plugins/platforms:/usr/lib/x86_64-linux-gnu/qt6/plugins"

# CRITICAL FIX: OpenGL and Mesa Environment (enables software rendering for Ubuntu 24.04)
export LIBGL_ALWAYS_INDIRECT=0
export LIBGL_ALWAYS_SOFTWARE=0
export MESA_GL_VERSION_OVERRIDE="4.5"
export MESA_GLSL_VERSION_OVERRIDE="450"
export GALLIUM_DRIVER="llvmpipe"

# Graphics system environment
export XDG_RUNTIME_DIR="/tmp/runtime-user"
export XDG_SESSION_TYPE="x11"
export WAYLAND_DISPLAY=""

# =============================================================================
# CRITICAL FIX: X11 GUI Environment Setup with Auto-Detection
# =============================================================================

# Function to auto-detect working X11 display
detect_x11_display() {
    local candidates=":1 :0 :10 :2 :1003"
    local working_display=""
    
    for disp in $candidates; do
        if timeout 3 bash -c "DISPLAY='$disp' xset q" >/dev/null 2>&1; then
            working_display="$disp"
            break
        fi
    done
    
    if [ -n "$working_display" ]; then
        export DISPLAY="$working_display"
        return 0
    else
        return 1
    fi
}

# Auto-detect and set X11 display if not already set correctly
if ! timeout 3 xset q >/dev/null 2>&1; then
    if detect_x11_display; then
        echo "✅ Auto-detected X11 display: $DISPLAY"
    else
        echo "⚠️ No working X11 display found"
        # Fallback to common displays
        export DISPLAY="${DISPLAY:-:1}"
        echo "🔄 Using fallback display: $DISPLAY"
    fi
fi

# =============================================================================
# CRITICAL FIX: Enhanced Graphics Functions with Fallbacks (Ubuntu 24.04)
# =============================================================================

# Function to start applications with graphics fallbacks
start_with_graphics_fallback() {
    local app_name="$1"
    local app_command="$2"
    shift 2
    
    echo "🚀 Starting $app_name with graphics support..."
    
    # Test X11 first
    if ! timeout 3 xset q >/dev/null 2>&1; then
        echo "⚠️ X11 not working, trying to auto-detect..."
        if detect_x11_display; then
            echo "✅ X11 fixed: $DISPLAY"
        else
            echo "❌ Cannot fix X11, trying software rendering..."
            LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=offscreen "$app_command" "$@"
            return
        fi
    fi
    
    # Try normal execution
    echo "🎯 Launching $app_name on $DISPLAY..."
    if "$app_command" "$@"; then
        echo "✅ $app_name started successfully"
    else
        echo "❌ $app_name failed, trying software rendering..."
        LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=softpipe "$app_command" "$@"
    fi
}

# Enhanced function to start RViz2 with fallbacks
start_rviz2() {
    start_with_graphics_fallback "RViz2" "rviz2" "$@"
}

# Enhanced function to start Gazebo with fallbacks (Ubuntu 24.04 compatible)
start_gazebo_sim() {
    echo "🚀 Starting Gazebo Harmonic with Qt6 support..."
    
    # Test X11 first
    if ! timeout 3 xset q >/dev/null 2>&1; then
        echo "⚠️ X11 not working, trying to fix..."
        if detect_x11_display; then
            echo "✅ X11 fixed: $DISPLAY"
        else
            echo "❌ Cannot fix X11, starting headless..."
            HEADLESS=1 gz sim "$@"
            return
        fi
    fi
    
    # Set Qt6 environment for Gazebo
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    
    # Try normal Gazebo with Qt6 support
    echo "🎯 Launching Gazebo Harmonic with Qt6 on $DISPLAY..."
    if gz sim "$@"; then
        echo "✅ Gazebo started successfully"
    else
        echo "❌ Gazebo failed, trying with software rendering..."
        LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe gz sim "$@"
    fi
}

# Function to diagnose graphics issues (Ubuntu 24.04 aware)
diagnose_graphics() {
    echo "🔍 Comprehensive Graphics Diagnostics (Ubuntu 24.04)"
    echo "================================================="
    echo "Environment Variables:"
    echo "  DISPLAY: $DISPLAY"
    echo "  XAUTHORITY: ${XAUTHORITY:-'Not set'}"
    echo "  QT_QPA_PLATFORM: $QT_QPA_PLATFORM"
    echo "  LIBGL_ALWAYS_SOFTWARE: ${LIBGL_ALWAYS_SOFTWARE:-'0'}"
    echo "  MESA_GL_VERSION_OVERRIDE: $MESA_GL_VERSION_OVERRIDE"
    echo "  GALLIUM_DRIVER: $GALLIUM_DRIVER"
    echo ""
    
    echo "X11 Test:"
    if timeout 5 xset q >/dev/null 2>&1; then
        echo "  ✅ X11 working on $DISPLAY"
        xset q | grep -E "(X.Org|version)" | head -2 | sed 's/^/  /'
    else
        echo "  ❌ X11 not working on $DISPLAY"
    fi
    echo ""
    
    echo "Available X11 sockets:"
    if [ -d "/tmp/.X11-unix" ]; then
        ls -la /tmp/.X11-unix/ 2>/dev/null | sed 's/^/  /' || echo "  No X11 sockets found"
    else
        echo "  /tmp/.X11-unix directory not found"
    fi
    echo ""
    
    echo "OpenGL Information (Ubuntu 24.04):"
    if command -v glxinfo >/dev/null 2>&1; then
        if timeout 10 glxinfo -B >/dev/null 2>&1; then
            echo "  ✅ OpenGL working"
            glxinfo -B | grep -E "(OpenGL vendor|OpenGL renderer|OpenGL version)" | sed 's/^/  /'
        else
            echo "  ⚠️ OpenGL test failed, trying software rendering..."
            LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B | grep -E "(OpenGL vendor|OpenGL renderer|OpenGL version)" | sed 's/^/  /' 2>/dev/null || echo "  ❌ Software rendering also failed"
        fi
    else
        echo "  ❌ glxinfo not available"
    fi
    echo ""
    
    echo "Qt6 Platform Plugins (Ubuntu 24.04):"
    QT6_PLATFORMS=$(find /usr -name "*qt6*" -name "*platforms*" -type d 2>/dev/null | head -3)
    if [ -n "$QT6_PLATFORMS" ]; then
        echo "  ✅ Qt6 platforms found:"
        echo "$QT6_PLATFORMS" | sed 's/^/    /'
        if [ -d "/usr/lib/x86_64-linux-gnu/qt6/plugins/platforms" ]; then
            echo "  Available platforms:"
            ls /usr/lib/x86_64-linux-gnu/qt6/plugins/platforms/ 2>/dev/null | sed 's/^/    /' || echo "    None found"
        fi
    else
        echo "  ❌ Qt6 platform plugins not found"
    fi
    echo ""
    
    echo "Mesa/OpenGL Libraries (Ubuntu 24.04):"
    echo "  libgl1: $(dpkg -l | grep libgl1 | awk '{print $2,$3}' || echo 'Not found')"
    echo "  libglx-mesa0: $(dpkg -l | grep libglx-mesa0 | awk '{print $2,$3}' || echo 'Not found')"
    echo "  libglut3.12: $(dpkg -l | grep libglut3.12 | awk '{print $2,$3}' || echo 'Not found')"
    echo ""
    
    echo "Gazebo Harmonic Test:"
    if command -v gz >/dev/null 2>&1; then
        echo "  ✅ Gazebo command available"
        gz --version 2>/dev/null | head -1 | sed 's/^/  /' || echo "  Version check failed"
    else
        echo "  ❌ Gazebo command not found"
    fi
}

# Function to test all graphics components (Ubuntu 24.04)
test_graphics_stack() {
    echo "🧪 Testing Complete Graphics Stack (Ubuntu 24.04)"
    echo "=============================================="
    
    echo "1. Testing X11..."
    if timeout 3 xset q >/dev/null 2>&1; then
        echo "   ✅ X11 working"
    else
        echo "   ❌ X11 failed"
        return 1
    fi
    
    echo "2. Testing OpenGL..."
    if timeout 10 glxinfo -B >/dev/null 2>&1; then
        echo "   ✅ OpenGL working"
    else
        echo "   ⚠️ Hardware OpenGL failed, testing software..."
        if LIBGL_ALWAYS_SOFTWARE=1 timeout 10 glxinfo -B >/dev/null 2>&1; then
            echo "   ✅ Software OpenGL working"
        else
            echo "   ❌ Both hardware and software OpenGL failed"
            return 1
        fi
    fi
    
    echo "3. Testing Qt6..."
    if find /usr -name "*qt6*" -name "*platforms*" -type d 2>/dev/null | grep -q platforms; then
        echo "   ✅ Qt6 platform plugins found"
    else
        echo "   ❌ Qt6 platform plugins missing"
        return 1
    fi
    
    echo "4. Testing Gazebo..."
    if command -v gz >/dev/null 2>&1; then
        echo "   ✅ Gazebo command available"
    else
        echo "   ❌ Gazebo not found"
        return 1
    fi
    
    echo ""
    echo "🎉 Graphics stack test completed for Ubuntu 24.04!"
    return 0
}

# =============================================================================
# Enhanced Aliases and Functions (Ubuntu 24.04)
# =============================================================================

# Add useful aliases for ROS2 development
alias rosdep_install='rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy'
alias colcon_build='colcon build --executor sequential --event-handlers console_direct+'
alias source_ros='source /opt/ros/jazzy/setup.bash'
alias source_ws='source $ROS2_WS/install/setup.bash'
alias cd_ws='cd $ROS2_WS'
alias cd_dev='cd $DEV_DIR'

# PX4 aliases
alias px4_sitl='cd $PX4_DIR && make px4_sitl'
alias px4_gazebo='cd $PX4_DIR && make px4_sitl gz_x500'
alias px4_gazebo_headless='cd $PX4_DIR && HEADLESS=1 make px4_sitl gz_x500'
alias px4_clean='cd $PX4_DIR && make clean && make distclean'

# CRITICAL FIX: Enhanced Gazebo aliases with Qt6 support (Ubuntu 24.04)
alias gz_list_models='gz model --list'
alias gz_list_worlds='gz world --list'
alias gz_start='start_gazebo_sim'
alias gz_debug='QT_DEBUG_PLUGINS=1 start_gazebo_sim'
alias gz_software='LIBGL_ALWAYS_SOFTWARE=1 start_gazebo_sim'

# X11 and GUI aliases with enhanced functionality
alias test_x11='timeout 3 xset q && echo "✅ X11 working on $DISPLAY" || echo "❌ X11 not working"'
alias fix_x11='detect_x11_display && echo "✅ X11 fixed: $DISPLAY" || echo "❌ No working X11 display found"'
alias start_rviz='start_rviz2'
alias start_gazebo='start_gazebo_sim'
alias graphics_debug='diagnose_graphics'
alias test_graphics='test_graphics_stack'

# Software rendering aliases
alias rviz_software='LIBGL_ALWAYS_SOFTWARE=1 rviz2'
alias gazebo_software='LIBGL_ALWAYS_SOFTWARE=1 gz sim'
alias glxinfo_software='LIBGL_ALWAYS_SOFTWARE=1 glxinfo'

# Qt6 specific aliases
alias qt6_debug='QT_DEBUG_PLUGINS=1 QT_QPA_PLATFORM=xcb'
alias qt6_software='QT_QPA_PLATFORM=offscreen'

# Ubuntu 24.04 specific package check aliases
alias check_packages='echo "Checking Ubuntu 24.04 packages:"; dpkg -l | grep -E "(libgl1|libglx-mesa0|libglut3.12|qt6-base)" | awk "{print \$2,\$3}"'
alias check_mesa='echo "Mesa packages:"; dpkg -l | grep mesa | awk "{print \$2,\$3}"'

# =============================================================================
# Startup Diagnostics and Status (Ubuntu 24.04)
# =============================================================================

# Show environment status
echo "🚀 ROS2 Agent Sim Environment Ready (FIXED VERSION - Ubuntu 24.04)"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   DEV_DIR: $DEV_DIR"
echo "   ROS2_WS: $ROS2_WS"
echo "   PX4_DIR: $PX4_DIR"
echo "   GZ_VERSION: $GZ_VERSION"
echo "   DISPLAY: $DISPLAY"

# Check if workspaces are properly sourced
if [ -n "$AMENT_PREFIX_PATH" ]; then
    echo "   ✅ ROS2 workspace sourced"
else
    echo "   ⚠️  ROS2 workspace not sourced"
fi

# Check if PX4 Gazebo paths are set
if [ -n "$GZ_SIM_RESOURCE_PATH" ]; then
    echo "   ✅ Gazebo resource paths configured"
else
    echo "   ⚠️  Gazebo resource paths not configured"
fi

# CRITICAL FIX: Check X11 and Qt6 status (Ubuntu 24.04)
if timeout 3 xset q >/dev/null 2>&1; then
    echo "   ✅ X11 GUI ready ($DISPLAY)"
else
    echo "   ⚠️  X11 GUI not working (run 'fix_x11' to auto-detect)"
fi

# Check Qt6 environment
if [ -n "$QT_QPA_PLATFORM" ] && [ "$QT_QPA_PLATFORM" = "xcb" ]; then
    echo "   ✅ Qt6 environment configured"
else
    echo "   ⚠️  Qt6 environment not properly configured"
fi

# Check OpenGL status (Ubuntu 24.04 aware)
if command -v glxinfo >/dev/null 2>&1; then
    if timeout 5 glxinfo -B >/dev/null 2>&1; then
        RENDERER=$(glxinfo -B 2>/dev/null | grep "OpenGL renderer" | cut -d: -f2 | xargs || echo "Unknown")
        echo "   ✅ OpenGL ready: $RENDERER"
    else
        echo "   ⚠️  OpenGL hardware failed, software rendering available"
    fi
else
    echo "   ⚠️  OpenGL testing tools not available"
fi

# Check Ubuntu 24.04 specific packages
if dpkg -l | grep -q "libgl1.*24\|libglx-mesa0.*24"; then
    echo "   ✅ Ubuntu 24.04 Mesa packages detected"
else
    echo "   ⚠️  Ubuntu 24.04 Mesa packages may not be properly installed"
fi

# Show helpful commands
echo ""
echo "🎯 Quick Commands (FIXED VERSION - Ubuntu 24.04):"
echo "   test_x11         - Test X11 connection"
echo "   fix_x11          - Auto-detect working X11 display"
echo "   start_gazebo     - Start Gazebo with auto-fallback"
echo "   start_rviz2      - Start RViz2 with auto-fallback"
echo "   diagnose_graphics - Complete graphics diagnostics"
echo "   test_graphics    - Test entire graphics stack"
echo "   gazebo_software  - Force software rendering for Gazebo"
echo "   qt6_debug        - Enable Qt6 debug output"
echo "   check_packages   - Check Ubuntu 24.04 packages"

# CRITICAL FIX: Auto-run graphics test on startup (optional)
if [ "${AUTO_TEST_GRAPHICS:-false}" = "true" ]; then
    echo ""
    echo "🔍 Auto-testing graphics environment..."
    test_graphics_stack >/dev/null 2>&1 && echo "✅ Graphics stack verified" || echo "⚠️ Graphics issues detected - run 'diagnose_graphics'"
fi