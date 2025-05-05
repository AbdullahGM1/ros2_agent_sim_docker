#!/bin/bash
# Build script for custom Docker image

IMAGE_NAME="ros2-agent-sim"
TAG="latest"

echo "Building Docker image: ${IMAGE_NAME}:${TAG}"

# Build the Docker image using the specific Dockerfile
docker build -f Dockerfile.ros2-agent-sim -t ${IMAGE_NAME}:${TAG} .

if [ $? -eq 0 ]; then
    echo "Docker image built successfully!"
    echo "To run the container, use: ./docker_run.sh"
else
    echo "Failed to build Docker image"
    exit 1
fi