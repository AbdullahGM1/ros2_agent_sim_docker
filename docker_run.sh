#!/bin/bash
# Modern Docker script for ROS2 Jazzy + Gazebo Harmonic + Ollama + PX4 + MAVROS + ROSA
# Enhanced to keep container running in background

set -e

# Configuration
CONTAINER_NAME="ros2_agent_sim"
IMAGE_NAME="ros2-agent-sim:latest"
DOCKERFILE_PATH="docker/Dockerfile.ros2-agent-sim"
WORKSPACE_DIR="$HOME/${CONTAINER_NAME}_shared_volume"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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
    if docker images --format "table {{.Repository}}:{{.Tag}}" | grep -q "^${IMAGE_NAME}$"; then
        return 0
    else
        return 1
    fi
}

# Function to build Docker image
build_image() {
    print_info "Building Docker image: ${IMAGE_NAME}"
    print_info "Using Dockerfile: ${DOCKERFILE_PATH}"
    
    if [ ! -f "${DOCKERFILE_PATH}" ]; then
        print_error "Dockerfile not found: ${DOCKERFILE_PATH}"
        print_error "Please ensure you're running this script from the correct directory."
        exit 1
    fi

    echo
    print_info "Starting Docker build process..."
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    start_time=$(date +%s)
    
    if docker build -f "${DOCKERFILE_PATH}" -t "${IMAGE_NAME}" . 2>&1; then
        end_time=$(date +%s)
        duration=$((end_time - start_time))
        
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        print_success "Docker image built successfully!"
        print_success "Build completed in ${duration} seconds"
        print_success "Image: ${IMAGE_NAME}"
        echo
        return 0
    else
        end_time=$(date +%s)
        duration=$((end_time - start_time))
        
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        print_error "Docker build failed after ${duration} seconds"
        exit 1
    fi
}

# Function to create missing supporting files
create_missing_file() {
    local file="$1"
    case "$file" in
        "scripts/px4_dev.sh")
            cat > "$file" << 'EOF'
#!/bin/bash
# PX4 development environment setup
set -e

echo "Setting up PX4 development environment..."

# Install basic dependencies
apt-get update && apt-get install -y \
    cmake \
    build-essential \
    git \
    ninja-build \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements if file exists
if [ -f /tmp/requirements.txt ]; then
    pip install -r /tmp/requirements.txt
fi

echo "PX4 development environment setup complete"
EOF
            chmod +x "$file"
            ;;
            
        "scripts/requirements.txt")
            cat > "$file" << 'EOF'
# PX4 Python requirements
pyserial
pyulog
numpy
jinja2
pyyaml
cerberus
packaging
toml
EOF
            ;;
            
        "scripts/bashrc_template.sh")
            cat > "$file" << 'EOF'
# ROS2 Jazzy + Gazebo Harmonic + Python Virtual Environment setup

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source ros_gz bridge
if [ -f /opt/ros2_ws/install/setup.bash ]; then
    source /opt/ros2_ws/install/setup.bash
fi

# Activate Python virtual environment
export PATH="/opt/python-env/bin:$PATH"
export VIRTUAL_ENV="/opt/python-env"

# Gazebo Harmonic
export GZ_VERSION=harmonic

# Environment variables
export DEV_DIR=/home/user/shared_volume
export PX4_DIR=$DEV_DIR/PX4-Autopilot
export ROS2_WS=$DEV_DIR/ros2_ws
export OSQP_SRC=$DEV_DIR
export ROS_DOMAIN_ID=0

# Aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

echo "🚀 ROS2 Jazzy + Gazebo Harmonic + Python AI/ML Environment Ready!"
echo "📁 Workspace: $DEV_DIR"
echo "🐍 Python Virtual Environment: Active"
echo "🤖 ROS2 Jazzy: Sourced"
echo "🏗️  Gazebo Harmonic: Ready"
EOF
            ;;
            
        "scripts/entrypoint.sh")
            # Create the fixed entrypoint that keeps container running
            cat > "$file" << 'EOF'
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

# Get host UID/GID from environment variables with fallbacks
HOST_UID=${LOCAL_USER_ID:-1000}
HOST_GID=${LOCAL_GROUP_ID:-1000}
HOST_USER=${HOST_USER:-"user"}

print_info "Host: $HOST_USER (UID: $HOST_UID, GID: $HOST_GID)"

# Setup user mapping
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
    
    # Update user UID/GID
    usermod -u "$HOST_UID" user 2>/dev/null || print_error "Failed to change user UID"
    groupmod -g "$HOST_GID" user 2>/dev/null || print_error "Failed to change user GID"
    
    # Update home directory ownership
    chown -R "$HOST_UID:$HOST_GID" /home/user 2>/dev/null || print_warning "Could not update home ownership"
    
    print_success "User UID/GID updated to: $(id -u user):$(id -g user)"
fi

# Setup bashrc
if [ -f "/opt/bashrc_templates/bashrc_template.sh" ]; then
    print_info "Installing proper .bashrc template..."
    cp /opt/bashrc_templates/bashrc_template.sh /home/user/.bashrc
    chown "$HOST_UID:$HOST_GID" /home/user/.bashrc
    chmod 644 /home/user/.bashrc
    print_success "New .bashrc installed"
fi

# Setup shared volume
if [ -d "/home/user/shared_volume" ]; then
    print_info "Setting up shared volume..."
    chown $HOST_UID:$HOST_GID /home/user/shared_volume/ 2>/dev/null || true
    chmod 755 /home/user/shared_volume/ 2>/dev/null || true
    
    # Create subdirectories
    mkdir -p /home/user/shared_volume/ros2_ws/src
    chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/ros2_ws/ 2>/dev/null || true
    
    # Copy files if needed
    if [ ! -f "/home/user/shared_volume/install.sh" ] && [ -f "/home/user/backup/install.sh" ]; then
        cp /home/user/backup/install.sh /home/user/shared_volume/
        chmod +x /home/user/shared_volume/install.sh
        chown $HOST_UID:$HOST_GID /home/user/shared_volume/install.sh
    fi
    
    if [ ! -d "/home/user/shared_volume/PX4_config" ] && [ -d "/home/user/backup/PX4_config" ]; then
        cp -r /home/user/backup/PX4_config /home/user/shared_volume/
        chown -R $HOST_UID:$HOST_GID /home/user/shared_volume/PX4_config/
    fi
fi

# Start Ollama service
if ! netstat -tuln 2>/dev/null | grep -q ":11434 "; then
    print_info "Starting Ollama service..."
    ollama serve &
    sleep 3
fi

# For persistent container mode, check if we should just keep running
if [ "$1" = "tail" ] && [ "$2" = "-f" ] && [ "$3" = "/dev/null" ]; then
    print_success "Container initialized successfully in persistent mode"
    print_info "Container will keep running in background"
    print_info "Use 'docker exec -it ${CONTAINER_NAME} bash' to connect"
    exec "$@"
fi

# For interactive mode, switch to user and run command
print_info "Switching to user context..."
exec sudo -u user -H bash -c "
    source ~/.bashrc
    cd /home/user/shared_volume 2>/dev/null || cd /home/user
    exec \"\$@\"
" -- "$@"
EOF
            chmod +x "$file"
            ;;
            
        "scripts/install.sh")
            cat > "$file" << 'EOF'
#!/bin/bash
# Installation script placeholder
echo "Running installation setup..."
echo "This is a placeholder. Replace with your actual installation script."
EOF
            chmod +x "$file"
            ;;
    esac
}

# Function to create middleware profiles
create_middleware_profiles() {
    mkdir -p middleware_profiles
    
    cat > middleware_profiles/rtps_udp_profile.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    
    <participant profile_name="udp_participant_profile">
        <rtps>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
}

# Enhanced GPU detection and support setup
setup_gpu_support() {
    print_info "🎮 Setting up GPU support..."
    
    DOCKER_OPTS=""
    
    # Detect GPU type
    if command -v nvidia-smi &> /dev/null; then
        print_info "NVIDIA GPU detected: $(nvidia-smi --query-gpu=name --format=csv,noheader,nounits | head -1)"
        
        # Check Docker version for GPU support
        DOCKER_VERSION=$(docker version --format '{{.Server.Version}}' 2>/dev/null || echo "20.10.0")
        DOCKER_MAJOR=$(echo $DOCKER_VERSION | cut -d. -f1)
        DOCKER_MINOR=$(echo $DOCKER_VERSION | cut -d. -f2)
        
        if [ "$DOCKER_MAJOR" -gt 19 ] || ([ "$DOCKER_MAJOR" -eq 19 ] && [ "$DOCKER_MINOR" -ge 3 ]); then
            DOCKER_OPTS="$DOCKER_OPTS --gpus all"
            print_success "Using native Docker GPU support (--gpus all)"
        else
            print_warning "Docker version < 19.03 detected"
            if command -v nvidia-docker &> /dev/null; then
                DOCKER_OPTS="$DOCKER_OPTS --runtime=nvidia"
                print_success "Using nvidia-docker runtime"
            else
                print_error "Please update Docker to version 19.03+ or install nvidia-docker2"
            fi
        fi
        
        # Additional NVIDIA environment variables
        DOCKER_OPTS="$DOCKER_OPTS -e NVIDIA_VISIBLE_DEVICES=all"
        DOCKER_OPTS="$DOCKER_OPTS -e NVIDIA_DRIVER_CAPABILITIES=all"
        
    elif lspci | grep -i vga | grep -i amd &> /dev/null; then
        print_info "AMD GPU detected"
        DOCKER_OPTS="$DOCKER_OPTS --device=/dev/dri"
        
    elif lspci | grep -i vga | grep -i intel &> /dev/null; then
        print_info "Intel GPU detected"
        DOCKER_OPTS="$DOCKER_OPTS --device=/dev/dri"
    else
        print_warning "No dedicated GPU detected, using software rendering"
    fi
    
    export DOCKER_OPTS
}

# X11 authentication and graphics setup
setup_x11_auth() {
    print_info "🖥️  Setting up X11 authentication..."
    
    # Ensure DISPLAY is set
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
    fi
    
    print_info "Display set to: $DISPLAY"
    
    # Allow X server connections (the key fix)
    if command -v xhost &> /dev/null; then
        xhost +local:root 2>/dev/null || print_warning "Could not set X11 permissions"
        xhost +local:docker 2>/dev/null || true
        print_success "X11 permissions configured"
    else
        print_warning "xhost command not found. Installing..."
        sudo apt-get update && sudo apt-get install -y x11-xserver-utils 2>/dev/null || true
    fi
    
    # Setup XAUTH file (simpler version)
    XAUTH=/tmp/.docker.xauth
    if [ -n "$DISPLAY" ]; then
        # Create auth file if it doesn't exist
        if [ ! -f $XAUTH ]; then
            touch $XAUTH
            chmod 666 $XAUTH
        fi
        
        # Add current display to auth file
        xauth_list=$(xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' || true)
        if [ -n "$xauth_list" ]; then
            echo $xauth_list | xauth -f $XAUTH nmerge - 2>/dev/null || true
            print_success "X11 authentication configured"
        fi
    fi
    
    export XAUTH
}

# Function to setup workspace
setup_workspace() {
    print_info "📁 Setting up workspace directory: $WORKSPACE_DIR"
    
    if [ ! -d $WORKSPACE_DIR ]; then
        mkdir -p $WORKSPACE_DIR
        print_success "Created workspace directory: $WORKSPACE_DIR"
    fi
    
    # Fix permissions
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    if [ -d "$WORKSPACE_DIR" ]; then
        chown -R $(whoami):$(whoami) "$WORKSPACE_DIR" 2>/dev/null || true
    fi
}

# Function to start container in persistent mode
start_persistent_container() {
    print_info "🚀 Starting container in persistent mode: $CONTAINER_NAME"
    
    # Get host user information
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    HOST_USER=$(whoami)
    
    # Start container in detached mode with proper X11 support
    docker run -d \
        --name=${CONTAINER_NAME} \
        --hostname=ros2-dev \
        --network host \
        --env="DISPLAY=${DISPLAY:-:0}" \
        --env="XAUTHORITY=${XAUTH:-/tmp/.docker.xauth}" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="LIBGL_ALWAYS_INDIRECT=0" \
        --env="TERM=xterm-256color" \
        --env="COLORTERM=truecolor" \
        --env="FORCE_COLOR=1" \
        --env="CLICOLOR_FORCE=1" \
        --env="CONTAINER_NAME=${CONTAINER_NAME}" \
        -e LOCAL_USER_ID="$HOST_UID" \
        -e LOCAL_GROUP_ID="$HOST_GID" \
        -e HOST_USER="$HOST_USER" \
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${XAUTH:-/tmp/.docker.xauth}:${XAUTH:-/tmp/.docker.xauth}:rw" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="$WORKSPACE_DIR:/home/user/shared_volume:rw" \
        --volume="/dev:/dev:rw" \
        --workdir /home/user/shared_volume \
        --security-opt seccomp=unconfined \
        --security-opt apparmor=unconfined \
        $DOCKER_OPTS \
        ${IMAGE_NAME} \
        tail -f /dev/null
    
    # Wait for container to be ready
    sleep 2
    
    # Check if container is running
    if [ "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
        print_success "Container started successfully in persistent mode"
        return 0
    else
        print_error "Failed to start container"
        docker logs ${CONTAINER_NAME}
        return 1
    fi
}

# Function to run or connect to container
run_container() {
    print_info "🐳 Managing container: $CONTAINER_NAME"
    
    # Check if container exists
    if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
        # Container exists, check if it's running
        if [ "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
            # Container is running, connect to it
            print_info "Container is already running"
            connect_to_container
        else
            # Container exists but stopped, start it
            print_info "Starting stopped container..."
            docker start ${CONTAINER_NAME}
            sleep 2
            connect_to_container
        fi
    else
        # Container doesn't exist, create and start it
        print_info "Creating new persistent container..."
        if start_persistent_container; then
            connect_to_container
        else
            print_error "Failed to create container"
            exit 1
        fi
    fi
}

# Cleanup function
cleanup() {
    print_info "🧹 Cleaning up X server permissions..."
    if command -v xhost &> /dev/null; then
        xhost -local:root 2>/dev/null || true
    fi
}

# Show system information
show_system_info() {
    print_info "💻 System Information:"
    echo "  - Docker version: $(docker --version 2>/dev/null | cut -d' ' -f3 | cut -d',' -f1 || echo 'Unknown')"
    echo "  - Host OS: $(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown Linux')"
    echo "  - Host User: $(whoami) (UID: $(id -u), GID: $(id -g))"
    
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
    
    echo "  - Display: ${DISPLAY:-'Not set'}"
    echo
}

# Show container status
show_container_status() {
    print_info "📊 Container Status:"
    
    if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
        if [ "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
            print_success "Container '${CONTAINER_NAME}' is RUNNING"
            echo "  - To connect: docker exec -it ${CONTAINER_NAME} bash"
            echo "  - To see logs: docker logs ${CONTAINER_NAME}"
        else
            print_warning "Container '${CONTAINER_NAME}' is STOPPED"
            echo "  - To start: docker start ${CONTAINER_NAME}"
        fi
    else
        print_info "Container '${CONTAINER_NAME}' does not exist"
    fi
    echo
}

# Show help
show_help() {
    echo "Usage: $0 [COMMAND]"
    echo
    echo "🚀 ROS2 Jazzy + Gazebo Harmonic + Ollama + PX4 + MAVROS + ROSA Development Environment"
    echo
    echo "Commands:"
    echo "  run       Start/connect to persistent container (default)"
    echo "  build     Build the Docker image"
    echo "  rebuild   Force rebuild the Docker image"
    echo "  clean     Remove container and image"
    echo "  shell     Open additional shell in running container"
    echo "  logs      Show container logs"
    echo "  stop      Stop the container (keeps it for later)"
    echo "  restart   Restart the container"
    echo "  status    Show container status"
    echo "  help      Show this help"
    echo
    echo "The container runs persistently in the background."
    echo "You can exit and reconnect without losing your work."
    echo
    echo "Examples:"
    echo "  $0                # Start/connect to container"
    echo "  $0 build          # Build image"
    echo "  $0 status         # Check container status"
    echo "  $0 shell          # Open another terminal in container"
}

# Main execution
main() {
    echo
    print_info "🚀 ROS2 Agent Sim Docker Environment"
    print_info "🎯 ROS2 Jazzy + Gazebo Harmonic + Ollama + PX4 + MAVROS + ROSA"
    print_info "Container: $CONTAINER_NAME"
    print_info "Workspace: $WORKSPACE_DIR"
    echo
    
    show_system_info
    check_docker
    
    if check_image_exists; then
        print_success "✅ Docker image exists: ${IMAGE_NAME}"
    else
        print_warning "⚠️  Docker image not found: ${IMAGE_NAME}"
        build_image
    fi
    
    setup_gpu_support
    setup_x11_auth
    setup_workspace
    
    echo
    print_info "🐳 Starting persistent container environment..."
    run_container
    
    trap cleanup EXIT
}

# Handle commands
case "${1:-run}" in
    "run")
        main
        ;;
    "build")
        check_docker
        build_image
        ;;
    "rebuild")
        check_docker
        print_info "Force rebuilding image..."
        docker rmi "${IMAGE_NAME}" 2>/dev/null || true
        build_image
        ;;
    "clean")
        print_info "Cleaning up container and image..."
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
        docker rmi "$IMAGE_NAME" 2>/dev/null || true
        rm -rf "$WORKSPACE_DIR" 2>/dev/null || true
        print_success "Cleanup complete"
        ;;
    "shell")
        if [ "$(docker ps -q -f name="$CONTAINER_NAME")" ]; then
            connect_to_container
        else
            print_error "Container $CONTAINER_NAME is not running"
            print_info "Use '$0 run' to start it first"
            exit 1
        fi
        ;;
    "logs")
        docker logs "$CONTAINER_NAME"
        ;;
    "stop")
        docker stop "$CONTAINER_NAME"
        print_success "Container stopped (but not removed)"
        print_info "Use '$0 run' to restart it"
        ;;
    "restart")
        docker restart "$CONTAINER_NAME"
        print_success "Container restarted"
        ;;
    "status")
        show_container_status
        ;;
    "help")
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac