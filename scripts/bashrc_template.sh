#!/bin/bash
# ~/.bashrc: executed by bash(1) for non-login shells.

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
# X11 GUI Environment Setup (AUTO-DETECTION)
# =============================================================================

# Function to auto-detect working X11 display
detect_x11_display() {
    local candidates=":1 :0 :10 :2 :1003"
    local working_display=""
    
    for disp in $candidates; do
        if DISPLAY="$disp" xset q >/dev/null 2>&1; then
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
if ! xset q >/dev/null 2>&1; then
    if detect_x11_display; then
        echo "✅ Auto-detected X11 display: $DISPLAY"
    else
        echo "⚠️ No working X11 display found"
        # Fallback to common displays
        export DISPLAY="${DISPLAY:-:1}"
    fi
fi

# Set X11 environment variables for GUI applications
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export QT_XCB_GL_INTEGRATION=none
export QT_QPA_PLATFORM=xcb

# =============================================================================
# Enhanced Aliases and Functions
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

# Gazebo aliases
alias gz_list_models='gz model --list'
alias gz_list_worlds='gz world --list'

# X11 and GUI aliases
alias test_x11='xset q && echo "✅ X11 working on $DISPLAY" || echo "❌ X11 not working"'
alias fix_x11='detect_x11_display && echo "✅ X11 fixed: $DISPLAY" || echo "❌ No working X11 display found"'
alias start_rviz='test_x11 && rviz2 || echo "❌ Fix X11 first with: fix_x11"'
alias start_gazebo='test_x11 && gz sim || echo "❌ Fix X11 first with: fix_x11"'
alias gui_debug='echo "DISPLAY: $DISPLAY" && echo "X11 Test:" && test_x11 && echo "Qt Platform: $QT_QPA_PLATFORM"'

# Enhanced function to start RViz2 with fallbacks
start_rviz2() {
    echo "🚀 Starting RViz2..."
    
    # Test X11 first
    if ! xset q >/dev/null 2>&1; then
        echo "⚠️ X11 not working, trying to fix..."
        if detect_x11_display; then
            echo "✅ X11 fixed: $DISPLAY"
        else
            echo "❌ Cannot fix X11, trying software rendering..."
            LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=offscreen rviz2 "$@"
            return
        fi
    fi
    
    # Try normal RViz2
    echo "🎯 Launching RViz2 on $DISPLAY..."
    rviz2 "$@" || {
        echo "❌ RViz2 failed, trying software rendering..."
        LIBGL_ALWAYS_SOFTWARE=1 rviz2 "$@"
    }
}

# Enhanced function to start Gazebo with fallbacks
start_gazebo_sim() {
    echo "🚀 Starting Gazebo Simulation..."
    
    # Test X11 first
    if ! xset q >/dev/null 2>&1; then
        echo "⚠️ X11 not working, trying to fix..."
        if detect_x11_display; then
            echo "✅ X11 fixed: $DISPLAY"
        else
            echo "❌ Cannot fix X11, starting headless..."
            HEADLESS=1 gz sim "$@"
            return
        fi
    fi
    
    # Try normal Gazebo
    echo "🎯 Launching Gazebo on $DISPLAY..."
    gz sim "$@"
}

# Function to diagnose GUI issues
diagnose_gui() {
    echo "🔍 GUI Diagnostics"
    echo "=================="
    echo "DISPLAY: $DISPLAY"
    echo "XAUTHORITY: ${XAUTHORITY:-'Not set'}"
    echo "QT_QPA_PLATFORM: $QT_QPA_PLATFORM"
    echo ""
    echo "X11 Test:"
    test_x11
    echo ""
    echo "Available X11 sockets:"
    ls -la /tmp/.X11-unix/ 2>/dev/null || echo "No X11 sockets found"
    echo ""
    echo "Qt platform plugins:"
    find /opt/python-env /usr -name "*platforms*" -type d 2>/dev/null | head -3
}

# Show environment status
echo "🚀 ROS2 Agent Sim Environment Ready"
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

# Check X11 status
if xset q >/dev/null 2>&1; then
    echo "   ✅ X11 GUI ready ($DISPLAY)"
else
    echo "   ⚠️  X11 GUI not working (run 'fix_x11' to fix)"
fi

# Show helpful commands
echo ""
echo "🎯 Quick Commands:"
echo "   test_x11      - Test X11 connection"
echo "   start_rviz2   - Start RViz2 with auto-fallback"
echo "   diagnose_gui  - Diagnose GUI issues"
echo "   fix_x11       - Auto-fix X11 display"