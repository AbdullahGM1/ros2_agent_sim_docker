# ROS2 Humble + Gazebo Garden + PX4 + ROSA + ChatOllama Docker Environment

A complete Docker-based development environment for autonomous robotics featuring ROS2 Humble, Gazebo Garden simulation, PX4 autopilot integration, NASA's ROSA task planning framework, and AI capabilities with Ollama/LangChain.

![Docker Build](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ros2-humble-blue.svg?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/ubuntu-22.04-orange.svg?style=for-the-badge&logo=ubuntu&logoColor=white)
![Gazebo](https://img.shields.io/badge/gazebo-garden-green.svg?style=for-the-badge&logo=gazebo&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-autopilot-blue.svg?style=for-the-badge&logo=ardupilot&logoColor=white)
![Ollama](https://img.shields.io/badge/Ollama-LLM-purple.svg?style=for-the-badge&logo=ollama&logoColor=white)

## 🚀 Features

### Core Robotics Stack
- **ROS2 Humble Desktop Full** - Latest Robot Operating System 2 with complete desktop features
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
- NVIDIA GPU support (optional, for hardware acceleration)
- X11 server for GUI applications
- 20GB+ free disk space
- 8GB+ RAM recommended
- Internet connection for building (downloads ~2GB of packages)

## 🔧 Quick Start

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

The container will start with all services running, including:
- ROS2 Humble environment
- Gazebo Garden simulator
- Ollama with Qwen3:8b model
- VS Code for development
- XRCE-DDS Agent

## 📁 Directory Structure

```
ros2-agent-sim-docker/
├── Dockerfile.ros2-agent-sim  # Main Dockerfile
├── docker_run.sh              # Container runner script
├── build.sh                   # Image builder script
├── scripts/
│   ├── entrypoint.sh         # Container entry point
│   ├── px4_dev.sh            # PX4 development setup
│   └── requirements.txt      # Python dependencies
├── middleware_profiles/
│   └── rtps_udp_profile.xml  # Middleware configuration
└── README.md
```

<!-- ## 🔨 Usage

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
``` -->


### Testing Qwen3 Model
```bash

ollama run qwen3:8b

#It will run the model 
```

<!-- ### Running Gazebo Simulation
```bash
# Inside the container
gz sim

# With specific world
gz sim worlds/empty.sdf
```

### Starting PX4 SITL
```bash
# Clone and build PX4 if not already done
cd shared_volume
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Run SITL
make px4_sitl gazebo
```

### Using ROSA (NASA JPL Task Planning)
```bash
# Source ROSA workspace
source ~/rosa_ws/install/setup.bash

# Run ROSA examples
cd ~/rosa_ws/src/rosa
# Follow ROSA documentation for specific usage
```

### Starting XRCE-DDS Agent
```bash
# Start agent for UDP transport
MicroXRCEAgent udp4 -p 8888

# Start agent for serial transport
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
``` -->

<!-- ## 🛠️ Customization

### Adding New Dependencies

1. **Python Packages:**
   ```bash
   # Edit scripts/requirements.txt
   nano scripts/requirements.txt
   # Add new package
   new-package==1.0.0
   ```

2. **System Packages:**
   ```bash
   # Edit scripts/px4_dev.sh
   nano scripts/px4_dev.sh
   # Add apt-get install commands
   ```

3. **ROS2 Packages:**
   ```bash
   # Add to Dockerfile
   RUN apt install -y ros-humble-your-package
   ```

### Middleware Configuration
```bash
# Edit middleware_profiles/rtps_udp_profile.xml
nano middleware_profiles/rtps_udp_profile.xml
``` -->

### Container Commands
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


### 🎉 Acknowledgments

This project builds upon the excellent work of:

- [smart_track Docker Environment](https://github.com/mzahana/smart_track/tree/main/docker) by [Mohammed Abdelkader](https://github.com/mzahana)
- [PX4 ROS2 Humble Integration](https://github.com/mzahana/px4_ros2_humble) by [Mohammed Abdelkader](https://github.com/mzahana)
- [ROSA (NASA JPL)](https://github.com/nasa-jpl/rosa) - ROS Agent task planning framework

Special thanks to [Mohammed Abdelkader](https://github.com/mzahana) for providing the foundational Docker configurations and ROS2-PX4 integration scripts that made this project possible.

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Contact

Abdullah GM - [@AbdullahGM1](https://github.com/AbdullahGM1) - agm.musalami@gmail.com


<!-- ## 🚦 Roadmap

- [ ] Add support for additional LLM models (GPT-4, Claude)
- [ ] Integrate QGroundControl for flight planning
- [ ] Add ROS2 navigation stack integration
- [ ] Support for real hardware testing with USB passthrough
- [ ] Add CI/CD pipeline for automated testing
- [ ] Jupyter notebook integration for interactive development
- [ ] Multi-robot simulation support
- [ ] Add camera/sensor simulation packages -->


## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Garden Documentation](https://gazebosim.org/docs/garden/)
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [NASA ROSA Repository](https://github.com/nasa-jpl/rosa)
- [Ollama Documentation](https://github.com/ollama/ollama)
- [LangChain Documentation](https://python.langchain.com/)


---

<div align="center">
    Made with ❤️ by <a href="https://github.com/AbdullahGM1">Abdullah GM</a>
</div>