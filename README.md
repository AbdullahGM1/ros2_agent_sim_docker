# ROS2 Jazzy + Gazebo Harmonic + PX4 + ROSA + ChatOllama Docker Environment 

A complete Docker-based development environment for autonomous robotics featuring ROS2 Jazzy, Gazebo Harmonic simulation, PX4 autopilot integration, NASA's ROSA task planning framework, and AI capabilities with Ollama/LangChain. 

![Docker Build](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ros2-jazzy-blue.svg?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/ubuntu-24.04-orange.svg?style=for-the-badge&logo=ubuntu&logoColor=white)
![Gazebo](https://img.shields.io/badge/gazebo-harmonic-green.svg?style=for-the-badge&logo=gazebo&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-autopilot-blue.svg?style=for-the-badge&logo=ardupilot&logoColor=white)
![ROS Agent](https://img.shields.io/badge/ROS-Agent-red.svg?style=for-the-badge&logo=ros&logoColor=white)
![Ollama](https://img.shields.io/badge/Ollama-LLM-purple.svg?style=for-the-badge&logo=ollama&logoColor=white)
![Qt6](https://img.shields.io/badge/Qt6-GUI-green.svg?style=for-the-badge&logo=qt&logoColor=white)


## 📋 Table of Contents
- [Demo](#️-demo)
- [Future Vision](#-future-vision)
- [Features](#-features)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Setting Up the ROS2 Agent and Simulation](#-setting-up-the-ros2-agent-and-simulation)
- [Usage](#-usage)
- [Graphics Diagnostics](#-graphics-diagnostics)
- [Directory Structure](#-directory-structure)
- [Container Management](#-container-management)
- [Acknowledgments](#-acknowledgments)
- [License](#-license)
- [Contact](#-contact)
- [Additional Resources](#-additional-resources)

## 🖼️ Demo
<div align="center">
  <img src="/images/drone_rosa.gif" alt="ROS2 Agent Simulation Demo" width="800"/>
  <p><i>Demo of drone simulation with ROS2 Agent and Ollama integration</i></p>
</div>

## Sim TF-Tree
<div align="center">
  <img src="/images/tf tree.png" alt="ROS2 Agent Simulation Demo" width="800"/>
  <p><i>The TF Tree for the simulation for the Go2 and UAV robots</i></p>
</div>

## Rviz2
<div align="center">
  <img src="/images/rviz.png" alt="ROS2 Agent Simulation Demo" width="800"/>
  <p><i>Rivz2 for the two robots</i></p>
</div>

## 🔮 Future Vision
<div align="center">
  <img src="/images/multi_robot_flowchart.png" alt="Multi-Robot ROS2 Agent System Architecture" width="900"/>
  <p><i>Multi-Robot ROS2 Agent System Architecture - End Goal</i></p>
</div>

This represents our ultimate vision for a comprehensive multi-robot coordination system. The current implementation focuses on drone control, while this architecture showcases the planned expansion to include wheeled robots, legged robots (Unitree Go2), and advanced mission coordination capabilities.

### **System Flow Architecture:**

**🔄 Information Flow:**
1. **User** → **Multi-Robot CLI** → **ROSA Agent** (Natural Language Commands)
2. **ROSA Agent** → **Robot-Specific Tools** (Decision Making & Tool Selection)
3. **Robot Tools** → **ROS2 Topics & Clients** (Hardware Communication)
4. **Topics & Clients** → **CLI** → **User** (Feedback & Status Loop)

**📡 Communication Layers:**
- **MAVROS Topics & Clients**: Drone communication (state, pose, gimbal, setpoints)
- **Nav Topics & Clients**: Wheeled robot communication (cmd_vel, odometry, mapping, scanning)
- **Go2 Topics & Clients**: Legged robot communication (joint_states, walking commands, IMU, footstep planning)

**🧠 Central Intelligence:**
The ROSA Agent serves as the central decision-making brain, utilizing:
- **System Prompts**: Robot coordination, safety guidelines, mission context
- **Configuration System**: robots.yaml defining robot capabilities and topic mappings
- **Multi-Agent LLM**: Enhanced AI coordination between multiple robots

**📈 Key Features:**
- **Bidirectional Communication**: Real-time feedback from robots through CLI interface
- **Multi-Robot Coordination**: Simultaneous control of different robot types
- **Natural Language Interface**: Intuitive command structure for complex missions
- **Scalable Architecture**: Easy addition of new robot types and capabilities

### **Planned System Components:**

- **Multi-Robot CLI**: Unified command interface for all robot types with real-time status display
- **Robot Manager**: Lifecycle management and resource allocation across robot fleet
- **Mission Coordinator**: Task allocation and multi-robot synchronization for complex operations
- **Robot Factory**: Dynamic robot instance creation and management
- **Multi-Agent LLM**: Enhanced AI coordination between multiple robots with shared situational awareness
- **Configuration Management**: Dynamic robot discovery and capability mapping through robots.yaml

## 🚀 Features

### Core Robotics Stack (FIXED VERSION)
- **ROS2 Jazzy Desktop** - Latest Robot Operating System 2 with complete desktop features
- **Gazebo Harmonic** - Modern 3D robot simulation with Qt6 and enhanced graphics support
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
- **RQt tools** - Robotics visualization and debugging with Qt6 support
- **Python Development Environment** - Complete toolchain with pip packages
- **gedit** - Text editor for quick edits

### FIXED VERSION Graphics Stack
- **Qt6 Support** - Full Qt6 platform with XCB backend
- **Mesa Graphics** - Ubuntu 24.04 compatible OpenGL libraries
- **Graphics Diagnostics** - Built-in tools for troubleshooting GUI issues
- **Fallback Rendering** - Automatic hardware → software rendering cascade
- **X11 Auto-Detection** - Smart display detection with multiple fallbacks

## 📋 Prerequisites

- Ubuntu 22.04 or 24.04 (host system) - **Optimized for Ubuntu 24.04**
- Docker installed (version 19.03+)
- NVIDIA GPU support (optional, software fallbacks included)
- X11 server for GUI applications
- 20GB+ free disk space

## 🔧 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/AbdullahGM1/ros2_agent_sim_docker.git
cd ros2-agent-sim-docker
```

### 2. Make the Script Executable
```bash
chmod +x docker_run.sh
```

### 3. Run the Environment (FIXED VERSION)
```bash
./docker_run.sh
```

> **Note:** The script will automatically check if the Docker image exists and build it if necessary. The building process may take 30-60 minutes depending on your system specifications and internet connection. This FIXED VERSION includes comprehensive Ubuntu 24.04 compatibility and graphics environment setup.

The automated process includes:
- Docker image building with Ubuntu 24.04 fixes (if not exists)
- ROS2 Jazzy installation
- Gazebo Harmonic setup with Qt6 support
- PX4 development environment
- Ollama and Qwen3:8b model download
- All necessary dependencies with graphics fixes

### 4. Install Dependencies Inside the Container
Once inside the container, run the installation script to set up all dependencies:
```bash
cd /home/user/shared_volume
./install.sh
```

The install script includes:
- **Ubuntu 24.04 package verification**
- **Graphics environment validation**
- **Enhanced error handling and diagnostics**
- **Comprehensive workspace setup**

<div align="center">
  <h3>⚠️ ATTENTION ⚠️</h3>
  <p><strong>If the ros2_agent_sim package was not automatically cloned to ros2_ws/src/ during installation, you must manually clone it using the commands below.</strong></p>
</div>

## 🔄 Setting Up the ROS2 Agent and Simulation

After completing the installation steps above, follow these steps to set up the ROS2 Agent and simulation environment:

### 1. Clone the ROS2 Agent Simulation Package
```bash
cd ~/ros2_ws/src/
git clone --recursive https://github.com/AbdullahGM1/ros2_agent_sim.git
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

### 4. Launch the Drone Simulation (Enhanced Graphics Support)
```bash
ros2 launch drone_sim drone.launch.py
```
This will launch a drone simulation that is connected to PX4 autopilot with Qt6 and enhanced graphics support.

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
├── docker_run.sh              # Enhanced script with graphics support (build + run)
├── docker/
│   └── Dockerfile.ros2-agent-sim  # FIXED Dockerfile with Ubuntu 24.04 support
├── middleware_profiles        # DDS configuration profiles
│   └── rtps_udp_profile.xml
├── PX4_config                 # PX4 configuration files
│   ├── px4/
│   │   ├── 4020_gz_x500_d435
│   │   ├── 4021_gz_x500_lidar_camera
│   │   ├── 4022_gz_x3_uav
│   │   └── CMakeLists.txt
│   └── worlds/                # Simulation worlds
│       └── default.sdf
├── README.md
└── scripts/                   # Enhanced container scripts with graphics support
    ├── entrypoint.sh          # FIXED entrypoint with Ubuntu 24.04 compatibility
    ├── install.sh             # Enhanced installation with graphics validation
    ├── bashrc_template.sh     # Enhanced bashrc with graphics environment
    └── requirements.txt
```

## 🐳 Container Management

### Container Access and Credentials
- Default password for the user in the container: **user**

### Starting the Container (Enhanced)
```bash
# Simple startup with graphics support (automatic build if needed)
./docker_run.sh

# Check container and graphics status
./docker_run.sh status

# Force rebuild with Ubuntu 24.04 fixes
./docker_run.sh rebuild
```

### Enhanced Container Management Commands
```bash
# Stop container
docker stop ros2_agent_sim

# Remove container
docker rm ros2_agent_sim

# Remove image (full cleanup)
docker rmi ros2-agent-sim:latest

# Check logs
docker logs ros2_agent_sim

# Force rebuild from scratch with fixes
./docker_run.sh rebuild

# Open additional shell
./docker_run.sh shell

# Show comprehensive status
./docker_run.sh status
```

### Container Graphics Environment
The FIXED VERSION automatically configures:
- **X11 Authentication**: Multi-display detection and fallbacks
- **Qt6 Environment**: Platform plugins and conflict resolution
- **OpenGL Support**: Hardware acceleration with software fallbacks
- **GPU Passthrough**: NVIDIA/AMD/Intel GPU support
- **Graphics Debugging**: Built-in diagnostic tools

## 🎉 Acknowledgments

This project builds upon the excellent work of:

- [ROSA (NASA JPL)](https://github.com/nasa-jpl/rosa) - ROS Agent task planning framework
- [smart_track Docker Environment](https://github.com/mzahana/smart_track/tree/main/docker) by [Mohammed Abdelkader](https://github.com/mzahana)

Special thanks to [Mohammed Abdelkader](https://github.com/mzahana) for providing the foundational Docker configurations and ROS2-PX4 integration scripts that made this project possible.


## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Contact

Abdullah GM - [@AbdullahGM1](https://github.com/AbdullahGM1) - agm.musalami@gmail.com

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [NASA ROSA Repository](https://github.com/nasa-jpl/rosa)
- [Ollama Documentation](https://github.com/ollama/ollama)
- [LangChain Documentation](https://python.langchain.com/)
- [ROS2 Agent Simulation](https://github.com/AbdullahGM1/ros2_agent_sim) - The simulation and agent package used in this project
- [Qt6 Documentation](https://doc.qt.io/qt-6/) - For Qt6 platform and graphics information
- [Ubuntu 24.04 Release Notes](https://wiki.ubuntu.com/NobleNumbat/ReleaseNotes) - Ubuntu 24.04 compatibility information

---

<div align="center">
    <strong>FIXED VERSION with Ubuntu 24.04 Support & Enhanced Graphics</strong><br>
    Made with ❤️ by <a href="https://github.com/AbdullahGM1">Abdullah GM</a>
</div>
