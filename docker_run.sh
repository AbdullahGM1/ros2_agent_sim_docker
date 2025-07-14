#!/bin/bash
# Modern Docker script for ROS2 Jazzy + Gazebo Harmonic + Ollama + PX4 + MAVROS + ROSA
# Maintains all original functionality while fixing PEP 668 and dependency issues

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
    
    # Check for required supporting files
    local required_files=(
        "scripts/px4_dev.sh"
        "scripts/requirements.txt" 
        "scripts/bashrc_template.sh"
        "scripts/entrypoint.sh"
        "scripts/install.sh"
    )
    
    for file in "${required_files[@]}"; do
        if [ ! -f "$file" ]; then
            print_warning "Creating missing file: $file"
            mkdir -p "$(dirname "$file")"
            create_missing_file "$file"
        fi
    done
    
    # Check for middleware profiles
    if [ ! -d "middleware_profiles" ]; then
        print_warning "Creating missing middleware_profiles directory"
        create_middleware_profiles
    fi
    
    # Check for PX4_config
    if [ ! -d "PX4_config" ]; then
        print_warning "Creating missing PX4_config directory"
        mkdir -p PX4_config
        echo "# PX4 configuration files go here" > PX4_config/README.md
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
            cat > "$file" << 'EOF'
#!/bin/bash
set -e

# Handle UID/GID mapping for cross-platform compatibility
LOCAL_USER_ID=${LOCAL_USER_ID:-1000}
LOCAL_GROUP_ID=${LOCAL_GROUP_ID:-1000}

# Update user UID/GID if different
if [ "$LOCAL_USER_ID" != "1000" ] || [ "$LOCAL_GROUP_ID" != "1000" ]; then
    echo "Updating user UID/GID to $LOCAL_USER_ID:$LOCAL_GROUP_ID"
    groupmod -g $LOCAL_GROUP_ID user
    usermod -u $LOCAL_USER_ID -g $LOCAL_GROUP_ID user
    chown -R user:user /home/user
fi

# Setup bashrc from template
if [ -f /opt/bashrc_templates/bashrc_template.sh ] && [ ! -f /home/user/.bashrc_setup ]; then
    cat /opt/bashrc_templates/bashrc_template.sh >> /home/user/.bashrc
    touch /home/user/.bashrc_setup
    chown user:user /home/user/.bashrc /home/user/.bashrc_setup
fi

# Execute command as user
exec gosu user "$@"
EOF
            chmod +x "$file"
            ;;
            
        "scripts/install.sh")
            cat > "$file" << 'EOF'
#!/bin/bash
# Installation script for additional components

echo "Running additional installation setup..."

# Create workspace directories
mkdir -p /home/user/shared_volume/ros2_ws/src
mkdir -p /home/user/shared_volume/PX4-Autopilot

# Set permissions
chown -R user:user /home/user/shared_volume

echo "Installation setup complete"
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

# Enhanced X11 authentication and graphics setup
setup_x11_auth() {
    print_info "🖥️  Setting up X11 authentication..."
    
    if [ -z "$DISPLAY" ]; then
        print_warning "DISPLAY variable not set. GUI applications may not work."
        export DISPLAY=:0
    fi
    
    # Allow X server connections
    if command -v xhost &> /dev/null; then
        xhost +local:root 2>/dev/null || print_warning "Could not set X11 permissions"
        print_success "X11 permissions configured"
    else
        print_warning "xhost command not found. GUI applications may have permission issues."
    fi
    
    # Setup XAUTH file
    XAUTH=/tmp/.docker.xauth
    if [ -n "$DISPLAY" ]; then
        xauth_list=$(xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' || true)
        if [ ! -f $XAUTH ]; then
            touch $XAUTH
            chmod a+r $XAUTH
            if [ -n "$xauth_list" ]; then
                echo $xauth_list | xauth -f $XAUTH nmerge -
                print_success "X11 authentication configured"
            fi
        fi
    fi
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

# Function to run or connect to container
run_container() {
    print_info "🚀 Preparing to run container: $CONTAINER_NAME"
    
    # Get host user information
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    HOST_USER=$(whoami)
    
    # Base environment setup
    BASE_CMD="export DEV_DIR=/home/user/shared_volume && \
        export PX4_DIR=\$DEV_DIR/PX4-Autopilot && \
        export ROS2_WS=\$DEV_DIR/ros2_ws && \
        export OSQP_SRC=\$DEV_DIR && \
        cd /home/user/shared_volume && \
        source /home/user/.bashrc"
    
    CMD="$BASE_CMD && /bin/bash"
    
    # Check if container already exists
    if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
        # Test if container is healthy
        if docker exec ${CONTAINER_NAME} pwd >/dev/null 2>&1; then
            if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
                print_info "♻️  Restarting existing container..."
                docker start ${CONTAINER_NAME}
            fi
            
            print_info "🔗 Connecting to existing container: ${CONTAINER_NAME}"
            docker exec --user user --workdir /home/user/shared_volume -it ${CONTAINER_NAME} \
                env TERM=xterm-256color COLORTERM=truecolor FORCE_COLOR=1 \
                bash -l -c "${CMD}"
        else
            # Container has issues, recreate
            print_warning "🗑️  Removing problematic container and recreating..."
            docker stop ${CONTAINER_NAME} 2>/dev/null || true
            docker rm ${CONTAINER_NAME} 2>/dev/null || true
            run_new_container
        fi
    else
        run_new_container
    fi
}

# Function to run new container
run_new_container() {
    print_info "🆕 Running new container: ${CONTAINER_NAME}"
    print_info "🔗 UID mapping: Host $HOST_UID → Container user"
    
    docker run -it \
        --name=${CONTAINER_NAME} \
        --hostname=ros2-dev \
        --network host \
        --env="DISPLAY=${DISPLAY:-:0}" \
        --env="TERM=xterm-256color" \
        --env="COLORTERM=truecolor" \
        --env="FORCE_COLOR=1" \
        --env="CLICOLOR_FORCE=1" \
        -e LOCAL_USER_ID="$HOST_UID" \
        -e LOCAL_GROUP_ID="$HOST_GID" \
        -e HOST_USER="$HOST_USER" \
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="$WORKSPACE_DIR:/home/user/shared_volume:rw" \
        --volume="/dev:/dev:rw" \
        --workdir /home/user/shared_volume \
        --security-opt seccomp=unconfined \
        --security-opt apparmor=unconfined \
        $DOCKER_OPTS \
        ${IMAGE_NAME} \
        bash -l -c "${CMD}"
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

# Show help
show_help() {
    echo "Usage: $0 [COMMAND]"
    echo
    echo "🚀 ROS2 Jazzy + Gazebo Harmonic + Ollama + PX4 + MAVROS + ROSA Development Environment"
    echo
    echo "Commands:"
    echo "  run       Start or connect to container (default)"
    echo "  build     Build the Docker image"
    echo "  rebuild   Force rebuild the Docker image"
    echo "  clean     Remove container and image"
    echo "  shell     Open additional shell in running container"
    echo "  logs      Show container logs"
    echo "  stop      Stop the container"
    echo "  restart   Restart the container"
    echo "  help      Show this help"
    echo
    echo "Examples:"
    echo "  $0                # Start/connect to container"
    echo "  $0 build          # Build image"
    echo "  $0 clean          # Clean up everything"
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
    print_info "🐳 Starting container with all components..."
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
            docker exec -it --user user "$CONTAINER_NAME" /bin/bash
        else
            print_error "Container $CONTAINER_NAME is not running"
            exit 1
        fi
        ;;
    "logs")
        docker logs "$CONTAINER_NAME"
        ;;
    "stop")
        docker stop "$CONTAINER_NAME"
        print_success "Container stopped"
        ;;
    "restart")
        docker restart "$CONTAINER_NAME"
        print_success "Container restarted"
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