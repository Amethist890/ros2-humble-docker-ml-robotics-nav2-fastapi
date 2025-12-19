# ROS 2 Humble GPU Full Stack

Production-ready ROS 2 Humble development environment with GPU support, optimized for robotics R&D on Windows with Docker.

## Features

- **GPU-Accelerated Docker Container** with CUDA 12.1 and ROS 2 Humble
- **VS Code Dev Container** integration for seamless development
- **Sample ROS 2 Packages** (C++ and Python templates)
- **Sensor Fusion Pipeline** with multi-sensor data fusion
- **ROS 2 + FastAPI Bridge** for HTTP REST API access to topics
- **Multi-Node Docker Compose** for robotics simulation
- **Nav2 Navigation Stack** containerized deployment
- **Turtlesim** with GUI support over X11

## Quick Start

### Prerequisites

- Docker Desktop with WSL2 backend
- NVIDIA GPU with drivers (for GPU features)
- VcXsrv or similar X server (for GUI apps)
- VS Code with Dev Containers extension

### 1. Start X Server (Windows)

Install and run VcXsrv with:
- Multiple windows
- Display number: 0
- Start no client
- **Disable access control** (important!)

### 2. Build and Run Container

```bash
# Build the GPU image
cd docker
docker build -t ros_humble_gpu -f Dockerfile.gpu .

# Run interactively
docker run -it --name ros_humble_dev --gpus all \
  --network host \
  -v ${PWD}/src:/home/dev/ros_ws/src \
  -e DISPLAY=host.docker.internal:0.0 \
  -e QT_X11_NO_MITSHM=1 \
  ros_humble_gpu
```

### 3. Build Workspace (inside container)

```bash
cd /home/dev/ros_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Run Sample Nodes

```bash
# Terminal 1: C++ Talker
ros2 run sample_cpp_pkg talker

# Terminal 2: C++ Listener
ros2 run sample_cpp_pkg listener

# Terminal 3: Sensor Fusion Pipeline
ros2 launch sample_py_pkg sensor_fusion.launch.py

# Terminal 4: FastAPI Bridge
ros2 run ros_fastapi_bridge bridge
# Access API docs: http://localhost:8000/docs
```

### 5. Run Turtlesim (GUI)

```bash
# Terminal 1: Turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Teleop (use arrow keys)
ros2 run turtlesim turtle_teleop_key
```

## Docker Compose

### Start Multi-Node Simulation

```bash
# Start core services
docker-compose up -d ros_daemon turtlesim sensor_fusion sensor_publisher fastapi_bridge

# Start with GPU inference
docker-compose --profile gpu up -d

# Start interactive teleop
docker-compose --profile interactive run teleop

# Start demo (talker/listener)
docker-compose --profile demo up -d
```

### Start Nav2 Stack

```bash
cd src/nav2_bringup

# Ensure you have map files in ./maps
docker-compose up -d

# Start with RViz (GUI)
docker-compose --profile gui up -d
```

## Project Structure

```
ros2-humble/
├── docker/
│   ├── Dockerfile.gpu          # GPU-enabled ROS 2 image
│   └── entrypoint.sh           # Auto-source ROS environment
├── .devcontainer/
│   └── devcontainer.json       # VS Code Dev Container config
├── src/
│   ├── sample_cpp_pkg/         # C++ package template
│   │   ├── src/
│   │   │   ├── talker_node.cpp
│   │   │   └── listener_node.cpp
│   │   └── launch/
│   ├── sample_py_pkg/          # Python package with sensor fusion
│   │   ├── sample_py_pkg/
│   │   │   ├── sensor_fusion_node.py
│   │   │   ├── sensor_publisher.py
│   │   │   └── gpu_inference_stub.py
│   │   └── launch/
│   ├── ros_fastapi_bridge/     # HTTP REST API bridge
│   │   └── ros_fastapi_bridge/
│   │       └── bridge_node.py
│   └── nav2_bringup/           # Nav2 containerized stack
│       ├── docker-compose.yml
│       ├── params/
│       └── maps/
├── models/                      # ML models directory
├── docker-compose.yml           # Multi-node compose
└── README.md
```

## VS Code Development

1. Open this folder in VS Code
2. Install "Dev Containers" extension
3. Press `F1` → "Dev Containers: Reopen in Container"
4. Wait for container build
5. Start developing!

## API Endpoints (FastAPI Bridge)

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/topics` | List topics |
| GET | `/last/{topic}` | Get last message |
| POST | `/publish/string` | Publish string |
| POST | `/publish/cmd_vel` | Publish Twist |
| GET | `/docs` | Swagger UI |

## Troubleshooting

### GUI not showing

1. Ensure VcXsrv is running with "Disable access control"
2. Check DISPLAY variable: `echo $DISPLAY`
3. Test X11: `xeyes`

### ros2 command not found

```bash
source /opt/ros/humble/setup.bash
```

### Arrow keys not working in teleop

Use Git Bash or WSL terminal instead of PowerShell.

### GPU not detected

```bash
# Check NVIDIA runtime
nvidia-smi

# Run with GPU
docker run --gpus all ...
```

---

## 📸 Terminal Output Examples

### Docker Build Success

```
PS E:\codebase\ros2-humble> docker build -t ros_humble_gpu -f docker/Dockerfile.gpu .
[+] Building 285.3s (17/17) FINISHED
 => [internal] load build definition from Dockerfile.gpu
 => FROM docker.io/nvidia/cuda:12.1.1-devel-ubuntu22.04
 => RUN apt-get update && apt-get install -y ros-humble-desktop
 => Successfully built ros_humble_gpu
 
IMAGE                   DISK USAGE   CONTENT SIZE
ros_humble_gpu:latest   22.2GB       7.18GB
```

### Workspace Build

```
root@docker-desktop:/home/dev/ros_ws# colcon build --symlink-install
Starting >>> ros_fastapi_bridge
Starting >>> sample_cpp_pkg
Starting >>> sample_py_pkg
Finished <<< ros_fastapi_bridge [2.41s]
Finished <<< sample_py_pkg [2.44s]
Finished <<< sample_cpp_pkg [20.3s]

Summary: 3 packages finished [20.8s]
```

### C++ Talker/Listener Communication

```
[INFO] [talker]: Talker node started
[INFO] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 0'
[INFO] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 1'
[INFO] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 2'
[INFO] [listener]: Listener node started
[INFO] [listener]: Received: 'Hello from C++ ROS 2! Count: 3'
[INFO] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 4'
[INFO] [listener]: Received: 'Hello from C++ ROS 2! Count: 4'
[INFO] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 5'
[INFO] [listener]: Received: 'Hello from C++ ROS 2! Count: 5'
```

### Sensor Fusion Pipeline

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [sensor_publisher-1]: process started with pid [337]
[INFO] [sensor_fusion-2]: process started with pid [339]
[sensor_publisher-1] [INFO] [sensor_publisher]: Sensor publisher started (rate: 10.0 Hz)
[sensor_fusion-2] [INFO] [sensor_fusion]: Sensor fusion node started (method: mean)

root@docker-desktop:/home/dev/ros_ws# ros2 topic echo /fused --once
data: 25.656349182128906
---
```

### FastAPI Bridge

```
INFO:     Started server process [387]
INFO:     Waiting for application startup.
[INFO] [ros_fastapi_bridge]: ROS FastAPI Bridge node started
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)

# Health Check
root@docker-desktop# curl -s http://localhost:8000/health
{"status":"healthy","ros_node":true}

# Topics List
root@docker-desktop# curl -s http://localhost:8000/topics
{"subscribed":["bridge/string_in","bridge/float_in"],"publishers":["bridge/string_out","cmd_vel"],"cached":[]}
```

### Active ROS Topics

```
root@docker-desktop:/home/dev/ros_ws# ros2 topic list
/bridge/float_in
/bridge/string_in
/bridge/string_out
/chatter
/cmd_vel
/fused
/parameter_events
/rosout
/sensor_a
/sensor_b
```

---

## 🧪 Test Results Summary

| Component | Status | Description |
|-----------|--------|-------------|
| ✅ Docker Build | Pass | GPU image built (22.2GB) |
| ✅ Workspace Build | Pass | 3 packages compiled |
| ✅ C++ Talker/Listener | Pass | Pub/sub working at 2Hz |
| ✅ Sensor Fusion | Pass | Mean fusion outputting ~25.6 |
| ✅ FastAPI Bridge | Pass | REST API on port 8000 |
| ✅ ROS 2 Topics | Pass | 10 active topics |

---

## License

Apache-2.0

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Docker + ROS Best Practices](https://www.ros.org/blog/getting-started/)