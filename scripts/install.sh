#!/bin/bash

echo "Installing Python dependencies and Ollama for Go2 ROS Agent..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Password for sudo commands
pass="user"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to run commands with sudo and password
run_sudo() {
    echo "$pass" | sudo -S "$@"
}

echo -e "${GREEN}Installing Python dependencies...${NC}"
run_sudo pip3 install \
    rich \
    langchain \
    langchain-ollama \
    langchain-community \
    opencv-python \
    numpy

# Check if ollama is installed
if ! command_exists ollama; then
    echo -e "${YELLOW}Installing Ollama...${NC}"
    echo "$pass" | sudo -S bash -c "curl -fsSL https://ollama.ai/install.sh | sh"
    
    # Start ollama service
    run_sudo systemctl start ollama
    run_sudo systemctl enable ollama
else
    echo -e "${GREEN}Ollama is already installed${NC}"
fi

# Pull the default model
echo -e "${GREEN}Pulling Qwen3:8b model for LLM...${NC}"
ollama pull qwen3:8b

echo -e "${GREEN}Installation complete!${NC}"
echo
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Build your package:"
echo "   colcon build --packages-select go2_ros_agent"
echo
echo "2. Run the agent:"
echo "   ros2 run go2_ros_agent go2_agent"
echo
echo -e "${GREEN}Installation successful!${NC}"