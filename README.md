# ROS2 Humble + Gazebo Garden + PX4 + ROSA + ChatOllama Docker Environment

A complete Docker-based development environment for autonomous robotics featuring ROS2 Humble, Gazebo Garden simulation, PX4 autopilot integration, NASA's ROSA task planning framework, and AI capabilities with Ollama/LangChain.

![Docker Build](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ros2-humble-blue.svg?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/ubuntu-22.04-orange.svg?style=for-the-badge&logo=ubuntu&logoColor=white)
![Gazebo](https://img.shields.io/badge/gazebo-garden-green.svg?style=for-the-badge&logo=gazebo&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-autopilot-blue.svg?style=for-the-badge&logo=ardupilot&logoColor=white)
![ROS Agent](https://img.shields.io/badge/ROS-Agent-red.svg?style=for-the-badge&logo=ros&logoColor=white)
![Ollama](https://img.shields.io/badge/Ollama-LLM-purple.svg?style=for-the-badge&logo=ollama&logoColor=white)


## 📋 Table of Contents
- [Features](#-features)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Setting Up the ROS2 Agent and Simulation](#-setting-up-the-ros2-agent-and-simulation)
- [Usage](#-usage)
- [Directory Structure](#-directory-structure)
- [Container Management](#-container-management)
- [Acknowledgments](#-acknowledgments)
- [License](#-license)
- [Contact](#-contact)
- [Additional Resources](#-additional-resources)


## 🚀 Features

### Core Robotics Stack
- **ROS2 Humble Desktop** - Latest Robot Operating System 2 with complete desktop features
- **Gazebo Garden** - Modern 3D robot simulation with physics engine
- **XRCE-DDS Agent** - Lightweight DDS middleware for embedded systems
- **MAVROS** - MAVLink communication bridge for PX4/ArduPilot
- **rqt_tf** - ROS Transform visualization and debugging

### Flight Systems
- **PX4 Development Tools** - Complete toolchain for PX4 autopilot development
- **JSBSim** - Flight dynamics and control simulator  
- **ros_gz_bridge** - Custom-built bridge between ROS2 and Gazebo
- **SITL (Software-in-the-loop)** - PX4 simulation environment

### AI & Task Planning
- **ROSA (NASA JPL)** - ROS Agent task planning framework
- **Ollama + ChatOllama** - Local LLM integration with Qwen3:8b model
- **LangChain** - AI application development framework

### Development Tools
- **VS Code** - Complete IDE integrated in container
- **RQt tools** - Robotics visualization and debugging
- **Python Development Environment** - Complete toolchain with pip packages
- **gedit** - Text editor for quick edits

## 📋 Prerequisites

- Ubuntu 22.04 (host system)
- Docker installed (version 19.03+)
- NVIDIA GPU support 
- X11 server for GUI applications
- 20GB+ free disk space

## 🔧 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/AbdullahGM1/ros2_agent_sim_docker.git
cd ros2-agent-sim-docker
```

### 2. Build the Docker Image
```bash
chmod +x build.sh docker_run.sh
./build.sh
```

> **Note:** The building process may take 30-60 minutes depending on your system specifications and internet connection.

The build process includes:
- ROS2 Humble installation
- Gazebo Garden setup
- PX4 development environment
- Ollama and Qwen3:8b model download
- All necessary dependencies

### 3. Run the Container
```bash
./docker_run.sh
```

### 4. Install Dependencies Inside the Container
Once inside the container, run the installation script to set up all dependencies:
```bash
cd /shared_volume
./install.sh
```

## 🔄 Setting Up the ROS2 Agent and Simulation

After completing the installation steps above, follow these steps to set up the ROS2 Agent and simulation environment:

### 1. Clone the ROS2 Agent Simulation Package
```bash
cd ~/ros2_ws/src/
git clone https://github.com/AbdullahGM1/ros2_agent_sim.git
```

This package ([ros2_agent_sim](https://github.com/AbdullahGM1/ros2_agent_sim)) contains:
- ROS2 Agent Package - For LLM-based robot interaction
- Simulation environment - Integrated with PX4 for drone simulation

### 2. Build the Package
```bash
cd ~/ros2_ws
colcon build 
```

### 3. Source the Setup Files
```bash
source install/setup.bash
```

### 4. Launch the Drone Simulation
```bash
ros2 launch drone_sim drone.launch.py
```
This will launch a drone simulation that is connected to PX4 autopilot.

### 5. Run the ROS2 Agent
In a new terminal (inside the container), run:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_agent ros2_agent_node
```
This launches the interactive CLI interface to communicate with and control the robots.

## 🔨 Usage

### Interacting with the Drone
The ROS2 Agent provides a natural language interface to command the drone. Example commands:

```
> Take off to 2 meters
> Fly to position x=10, y=5, z=3
> Land
> What is your position
> Show me the camera feed
```

## 📁 Directory Structure

```
ros2-agent-sim-docker/
├── build.sh                 # Build script for Docker image
├── Dockerfile.ros2-agent-sim # Main Dockerfile
├── docker_run.sh            # Script to run the container
├── middleware_profiles      # DDS configuration profiles
│   └── rtps_udp_profile.xml
├── PX4_config               # PX4 configuration files
│   ├── 4022_gz_x500_lidar_camera
│   ├── 4023_gz_x3_uav
│   ├── CMakeLists.txt
│   ├── models              # Drone and sensor models
│   │   ├── gimbal_small_3d
│   │   ├── lidar
│   │   ├── x500
│   │   └── x500_lidar_camera
│   └── worlds              # Simulation worlds
│       └── default.sdf
├── README.md
└── scripts                  # Container initialization scripts
    ├── entrypoint.sh
    ├── install.sh
    ├── px4_dev.sh
    └── requirements.txt
```

## 🐳 Container Management

### Container Access and Credentials
- Default password for the user in the container: **user**

### Starting the Container
```bash
# Default startup
./docker_run.sh

# Custom container name
./docker_run.sh custom_container_name

# With custom command
./docker_run.sh container_name "command"
```

### Accessing Running Container
```bash
docker exec -it ros2_agent_sim bash
```

### Container Management Commands
```bash
# Stop container
docker stop ros2_agent_sim

# Remove container
docker rm ros2_agent_sim

# Remove image (full cleanup)
docker rmi ros2-agent-sim:latest

# Check logs
docker logs ros2_agent_sim

# Rebuild from scratch
docker build --no-cache -f Dockerfile.ros2-agent-sim -t ros2-agent-sim:latest .
```

## 🎉 Acknowledgments

This project builds upon the excellent work of:

- [smart_track Docker Environment](https://github.com/mzahana/smart_track/tree/main/docker) by [Mohammed Abdelkader](https://github.com/mzahana)
- [PX4 ROS2 Humble Integration](https://github.com/mzahana/px4_ros2_humble) by [Mohammed Abdelkader](https://github.com/mzahana)
- [ROSA (NASA JPL)](https://github.com/nasa-jpl/rosa) - ROS Agent task planning framework

Special thanks to [Mohammed Abdelkader](https://github.com/mzahana) for providing the foundational Docker configurations and ROS2-PX4 integration scripts that made this project possible.

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Contact

Abdullah GM - [@AbdullahGM1](https://github.com/AbdullahGM1) - agm.musalami@gmail.com

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Garden Documentation](https://gazebosim.org/docs/garden/)
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [NASA ROSA Repository](https://github.com/nasa-jpl/rosa)
- [Ollama Documentation](https://github.com/ollama/ollama)
- [LangChain Documentation](https://python.langchain.com/)
- [ROS2 Agent Simulation](https://github.com/AbdullahGM1/ros2_agent_sim) - The simulation and agent package used in this project

---

<div align="center">
    Made with ❤️ by <a href="https://github.com/AbdullahGM1">Abdullah GM</a>
</div>