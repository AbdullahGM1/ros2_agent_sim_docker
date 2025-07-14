#!/bin/bash
# Enhanced Docker script for ROS2 Agent Sim environment
# Automatically builds image if missing and runs container
# Enhanced with robust GPU support and cross-platform UID mapping
#
# Usage: ./docker_run.sh
#
# Authors: AbdullahGM1 <agm.musalami@gmail.com>

# Docker configuration
DOCKER_REPO="ros2-agent-sim:latest"
DOCKERFILE="docker/Dockerfile.ros2-agent-sim"
CONTAINER_NAME="ros2_agent_sim"
WORKSPACE_DIR=~/${CONTAINER_NAME}_shared_volume
DOCKER_OPTS=""
SUDO_PASSWORD="user"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Function to check if Docker is installed and running
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        print_error "Docker is not running. Please start Docker daemon."
        exit 1
    fi
}

# Function to check if image exists
check_image_exists() {
    if docker images --format "table {{.Repository}}:{{.Tag}}" | grep -q "^${DOCKER_REPO}$"; then
        return 0  # Image exists
    else
        return 1  # Image doesn't exist
    fi
}

# Function to build Docker image with enhanced feedback
build_image() {
    print_info "Building Docker image: ${DOCKER_REPO}"
    print_info "Using Dockerfile: ${DOCKERFILE}"
    
    if [ ! -f "${DOCKERFILE}" ]; then
        print_error "Dockerfile not found: ${DOCKERFILE}"
        print_error "Please ensure you're running this script from the correct directory."
        exit 1
    fi
    
    echo
    print_info "Starting Docker build process..."
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Start timer
    start_time=$(date +%s)
    
    # Build with progress
    if docker build -f "${DOCKERFILE}" -t "${DOCKER_REPO}" . 2>&1; then
        end_time=$(date +%s)
        duration=$((end_time - start_time))
        
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        print_success "Docker image built successfully!"
        print_success "Build completed in ${duration} seconds"
        print_success "Image: ${DOCKER_REPO}"
        echo
        return 0
    else
        end_time=$(date +%s)
        duration=$((end_time - start_time))
        
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        print_error "Docker image built failed after ${duration} seconds"
        print_error "Build process encountered errors"
        echo
        print_error "Full build log is shown above. Please check for errors and try again."
        exit 1
    fi
}

# ✅ ENHANCED: Better GPU detection and support setup
setup_gpu_support() {
    print_info "🎮 Setting up GPU support..."
    
    # Detect GPU type
    GPU_TYPE="none"
    if command -v nvidia-smi &> /dev/null; then
        GPU_TYPE="nvidia"
        print_info "NVIDIA GPU detected: $(nvidia-smi --query-gpu=name --format=csv,noheader,nounits | head -1)"
    elif lspci | grep -i amd | grep -i vga &> /dev/null; then
        GPU_TYPE="amd"
        print_info "AMD GPU detected"
    elif lspci | grep -i intel | grep -i vga &> /dev/null; then
        GPU_TYPE="intel"
        print_info "Intel GPU detected"
    else
        print_warning "No dedicated GPU detected, using software rendering"
    fi
    
    # Setup NVIDIA GPU support with enhanced compatibility
    if [ "$GPU_TYPE" = "nvidia" ]; then
        # Check Docker version for GPU support method
        DOCKER_VERSION=$(docker version --format '{{.Server.Version}}' 2>/dev/null || echo "20.10.0")
        DOCKER_MAJOR=$(echo $DOCKER_VERSION | cut -d. -f1)
        DOCKER_MINOR=$(echo $DOCKER_VERSION | cut -d. -f2)
        
        print_info "Docker version: $DOCKER_VERSION"
        
        # Enhanced Docker version checking
        if [ "$DOCKER_MAJOR" -gt 19 ] || ([ "$DOCKER_MAJOR" -eq 19 ] && [ "$DOCKER_MINOR" -ge 3 ]); then
            # Modern Docker with native GPU support
            DOCKER_OPTS="$DOCKER_OPTS --gpus all"
            print_success "Using native Docker GPU support (--gpus all)"
            
            # Additional NVIDIA environment variables
            DOCKER_OPTS="$DOCKER_OPTS -e NVIDIA_VISIBLE_DEVICES=all"
            DOCKER_OPTS="$DOCKER_OPTS -e NVIDIA_DRIVER_CAPABILITIES=all"
            
        else
            # Legacy Docker - check for nvidia-docker2
            print_warning "Docker version < 19.03 detected, checking for nvidia-docker2..."
            if command -v nvidia-docker &> /dev/null || dpkg --list 2>/dev/null | grep -q nvidia-docker2; then
                DOCKER_OPTS="$DOCKER_OPTS --runtime=nvidia"
                print_success "Using nvidia-docker2 runtime"
            else
                print_error "Please either:"
                print_error "1. Update Docker to version 19.03+ for native GPU support"
                print_error "2. Install nvidia-docker2 for legacy support"
                print_warning "Continuing without GPU support..."
                return 1
            fi
        fi
        
        # Check NVIDIA driver compatibility
        if command -v nvidia-smi &> /dev/null; then
            DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader,nounits | head -1)
            print_info "NVIDIA Driver version: $DRIVER_VERSION"
            
            # Check if driver is recent enough for CUDA 12.9
            DRIVER_MAJOR=$(echo $DRIVER_VERSION | cut -d. -f1)
            if [ "$DRIVER_MAJOR" -lt 525 ]; then
                print_warning "NVIDIA driver version $DRIVER_VERSION may not support CUDA 12.9"
                print_warning "Consider updating to driver version 525+ for best compatibility"
            fi
        fi
        
    elif [ "$GPU_TYPE" = "amd" ]; then
        # AMD GPU support
        DOCKER_OPTS="$DOCKER_OPTS --device=/dev/dri"
        print_info "Added AMD GPU device support"
        
    elif [ "$GPU_TYPE" = "intel" ]; then
        # Intel integrated graphics support
        DOCKER_OPTS="$DOCKER_OPTS --device=/dev/dri"
        print_info "Added Intel GPU device support"
    fi
    
    print_info "GPU arguments: $DOCKER_OPTS"
}

# ✅ ENHANCED: Better X11 authentication and graphics setup
setup_x11_auth() {
    print_info "🖥️  Setting up X11 authentication and graphics..."
    
    # Check if X11 is available
    if [ -z "$DISPLAY" ]; then
        print_warning "DISPLAY variable not set. GUI applications may not work."
        print_info "Consider running: export DISPLAY=:0"
    else
        print_info "DISPLAY: $DISPLAY"
    fi
    
    # Allow X server connections from docker (with better error handling)
    if command -v xhost &> /dev/null; then
        xhost +local:root 2>/dev/null
        print_info "Added X11 permissions for Docker"
    else
        print_warning "xhost command not found. GUI applications may have permission issues."
    fi
    
    # Enhanced X authentication setup
    XAUTH=/tmp/.docker.xauth
    if [ -n "$DISPLAY" ]; then
        # Extract display number for xauth
        DISPLAY_NUM=$(echo $DISPLAY | sed 's/.*://' | sed 's/\..*//')
        xauth_list=$(xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/')
        
        if [ ! -f $XAUTH ]; then
            print_info "Creating XAUTH file: $XAUTH"
            touch $XAUTH
            chmod a+r $XAUTH
            if [ ! -z "$xauth_list" ]; then
                echo $xauth_list | xauth -f $XAUTH nmerge -
                print_success "X11 authentication configured"
            else
                print_warning "No X11 authentication entries found"
            fi
        else
            print_info "Using existing XAUTH file: $XAUTH"
        fi
        
        # Verify XAUTH file
        if [ ! -f $XAUTH ]; then
            print_error "Failed to create XAUTH file. GUI applications may not work."
            return 1
        fi
    else
        print_warning "No DISPLAY set, skipping X11 authentication"
    fi
    
    return 0
}

# Function to setup workspace
setup_workspace() {
    print_info "📁 Setting up workspace directory: $WORKSPACE_DIR"
    
    # Create workspace directory if it doesn't exist
    if [ ! -d $WORKSPACE_DIR ]; then
        mkdir -p $WORKSPACE_DIR
        print_success "Created workspace directory: $WORKSPACE_DIR"
    else
        print_info "Workspace directory already exists: $WORKSPACE_DIR"
    fi
    
    # CRITICAL: Fix workspace permissions immediately
    print_info "🔧 Ensuring correct workspace permissions..."
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    
    # Fix ownership if needed
    current_owner=$(stat -c "%U" "$WORKSPACE_DIR" 2>/dev/null || echo "unknown")
    if [ "$current_owner" != "$(whoami)" ]; then
        print_warning "Workspace owned by '$current_owner', fixing to '$(whoami)'..."
        chown -R $(whoami):$(whoami) "$WORKSPACE_DIR" 2>/dev/null || {
            print_warning "Could not fix workspace ownership, trying with sudo..."
            sudo chown -R $(whoami):$(whoami) "$WORKSPACE_DIR" || {
                print_error "Failed to fix workspace ownership"
            }
        }
    fi
}

# Function to setup host user information for UID mapping
setup_host_user_info() {
    print_info "👤 Setting up cross-platform UID mapping..."
    
    # Get host user information
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    HOST_USER=$(whoami)
    
    # Detect Ubuntu version for better compatibility
    UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
    
    print_info "Host user: $HOST_USER (UID: $HOST_UID, GID: $HOST_GID)"
    print_info "Ubuntu version: $UBUNTU_VERSION"
    
    # Validate UID range (should be >= 1000 for regular users)
    if [ "$HOST_UID" -lt 1000 ]; then
        print_warning "Host UID ($HOST_UID) is less than 1000. This might be a system user."
        print_warning "Proceeding anyway, but you may need to run as sudo."
    fi
    
    # Export for use in run_container function
    export HOST_UID HOST_GID HOST_USER UBUNTU_VERSION
}

# ✅ ENHANCED: Better container execution with improved volume mounts
run_container() {
    print_info "🚀 Preparing to run container: $CONTAINER_NAME"
    
    # Setup environment variables
    BASE_CMD="export DEV_DIR=/home/user/shared_volume && \
        export PX4_DIR=\$DEV_DIR/PX4-Autopilot &&\
        export ROS2_WS=\$DEV_DIR/ros2_ws &&\
        export OSQP_SRC=\$DEV_DIR &&\
        cd /home/user/shared_volume &&\
        source /home/user/.bashrc &&\
        if [ -f \"/home/user/shared_volume/ros2_ws/install/setup.bash\" ]; then
            source /home/user/shared_volume/ros2_ws/install/setup.bash
        fi"
    
    # Default command
    CMD="$BASE_CMD && /bin/bash"
    
    # Add additional environment variables
    if [[ -n "$GIT_TOKEN" ]] && [[ -n "$GIT_USER" ]]; then
        CMD="export GIT_USER=$GIT_USER && export GIT_TOKEN=$GIT_TOKEN && $CMD"
    fi
    
    if [[ -n "$SUDO_PASSWORD" ]]; then
        CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
    fi
    
    # Check if container already exists
    if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
        # Test if container is healthy and accessible
        if docker exec ${CONTAINER_NAME} pwd >/dev/null 2>&1; then
            # Container is healthy, check if it's running
            if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
                print_info "♻️  Restarting existing container..."
                docker start ${CONTAINER_NAME}
            fi
            
            print_info "🔗 Connecting to existing container: ${CONTAINER_NAME}"
            # Enhanced exec command with better environment
            docker exec --user user --workdir /home/user/shared_volume -it ${CONTAINER_NAME} env \
                TERM=xterm-256color \
                COLORTERM=truecolor \
                FORCE_COLOR=1 \
                CLICOLOR_FORCE=1 \
                DISPLAY=${DISPLAY:-:0} \
                bash -l -c "${CMD}"
        else
            # Container has issues, remove and recreate
            print_warning "🗑️  Existing container has issues, removing and recreating..."
            docker stop ${CONTAINER_NAME} 2>/dev/null || true
            docker rm ${CONTAINER_NAME} 2>/dev/null || true
            
            print_info "🆕 Running new container: ${CONTAINER_NAME}"
            print_info "🔗 UID mapping: Host $HOST_UID → Container user"
            
            # ✅ ENHANCED: Create new container with better volume mounts and graphics support
            docker run -it \
                --network host \
                --env="DISPLAY=${DISPLAY:-:0}" \
                --env="TERM=xterm-256color" \
                --env="COLORTERM=truecolor" \
                --env="FORCE_COLOR=1" \
                --env="CLICOLOR_FORCE=1" \
                -e LOCAL_USER_ID="$HOST_UID" \
                -e LOCAL_GROUP_ID="$HOST_GID" \
                -e HOST_USER="$HOST_USER" \
                -e UBUNTU_VERSION="$UBUNTU_VERSION" \
                -e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
                --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                --volume="/etc/localtime:/etc/localtime:ro" \
                --volume="$WORKSPACE_DIR:/home/user/shared_volume:rw" \
                --volume="/dev:/dev:rw" \
                --name=${CONTAINER_NAME} \
                --privileged \
                --workdir /home/user/shared_volume \
                --security-opt seccomp=unconfined \
                --security-opt apparmor=unconfined \
                $DOCKER_OPTS \
                ${DOCKER_REPO} \
                bash -l -c "${CMD}"
        fi
    else
        print_info "🆕 Running new container: ${CONTAINER_NAME}"
        print_info "🔗 UID mapping: Host $HOST_UID → Container user"
        
        # ✅ ENHANCED: Docker run command with comprehensive graphics and GPU support
        docker run -it \
            --network host \
            --env="DISPLAY=${DISPLAY:-:0}" \
            --env="TERM=xterm-256color" \
            --env="COLORTERM=truecolor" \
            --env="FORCE_COLOR=1" \
            --env="CLICOLOR_FORCE=1" \
            -e LOCAL_USER_ID="$HOST_UID" \
            -e LOCAL_GROUP_ID="$HOST_GID" \
            -e HOST_USER="$HOST_USER" \
            -e UBUNTU_VERSION="$UBUNTU_VERSION" \
            -e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="/etc/localtime:/etc/localtime:ro" \
            --volume="$WORKSPACE_DIR:/home/user/shared_volume:rw" \
            --volume="/dev:/dev:rw" \
            --name=${CONTAINER_NAME} \
            --privileged \
            --workdir /home/user/shared_volume \
            --security-opt seccomp=unconfined \
            --security-opt apparmor=unconfined \
            $DOCKER_OPTS \
            ${DOCKER_REPO} \
            bash -l -c "${CMD}"
    fi
}

# Function to cleanup
cleanup() {
    print_info "🧹 Cleaning up X server permissions..."
    if command -v xhost &> /dev/null; then
        xhost -local:root 2>/dev/null
    fi
}

# ✅ ENHANCED: Better system information display
show_system_info() {
    print_info "💻 System Information:"
    echo "  - Docker version: $(docker --version 2>/dev/null | cut -d' ' -f3 | cut -d',' -f1 || echo 'Unknown')"
    echo "  - Host OS: $(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown Linux')"
    echo "  - Host User: $(whoami) (UID: $(id -u), GID: $(id -g))"
    
    # Enhanced GPU information
    if command -v nvidia-smi &> /dev/null; then
        GPU_INFO=$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader,nounits | head -1)
        echo "  - NVIDIA GPU: $GPU_INFO"
    elif lspci | grep -i vga | grep -i amd &> /dev/null; then
        echo "  - GPU: AMD GPU detected"
    elif lspci | grep -i vga | grep -i intel &> /dev/null; then
        echo "  - GPU: Intel integrated graphics"
    else
        echo "  - GPU: No dedicated GPU detected"
    fi
    
    # Display information
    echo "  - Display: ${DISPLAY:-'Not set'}"
    echo "  - X11 Auth: $([ -f /tmp/.docker.xauth ] && echo 'Configured' || echo 'Not configured')"
    echo
}

# Main execution
main() {
    echo
    print_info "🚀 ROS2 Agent Sim Docker Environment (Enhanced GPU Support)"
    print_info "Container: $CONTAINER_NAME"
    print_info "Workspace: $WORKSPACE_DIR"
    echo
    
    # Show enhanced system information
    show_system_info
    
    # Check Docker installation
    check_docker
    
    # Setup host user information for UID mapping
    setup_host_user_info
    
    # Check if image exists, build if missing
    if check_image_exists; then
        print_success "✅ Docker image already exists: ${DOCKER_REPO}"
    else
        print_warning "⚠️  Docker image not found: ${DOCKER_REPO}"
        build_image
    fi
    
    # Setup enhanced GPU support
    setup_gpu_support
    
    # Setup enhanced X11 authentication
    setup_x11_auth
    
    # Setup workspace with proper permissions
    setup_workspace
    
    # Run container
    echo
    print_info "🐳 Starting container with enhanced GPU support..."
    run_container
    
    # Cleanup on exit
    trap cleanup EXIT
}

# Run main function
main "$@"