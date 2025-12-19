# ROS 2 Humble GPU Stack - Testing Results

**Date:** December 5, 2025  
**Platform:** Windows 10/11 + Docker Desktop + WSL2  
**Image:** ros_humble_gpu (22.2GB)

---

## 1. Docker Image Build

### Command

```powershell
docker build -t ros_humble_gpu -f docker/Dockerfile.gpu .
```

### Output

```
[+] Building 285.3s (17/17) FINISHED                    docker:desktop-linux
 => [internal] load build definition from Dockerfile.gpu                  0.1s
 => [internal] load metadata for docker.io/nvidia/cuda:12.1.1-devel      2.5s
 => [ 1/12] FROM docker.io/nvidia/cuda:12.1.1-devel-ubuntu22.04          0.0s
 => CACHED [ 2/12] RUN apt-get update && apt-get install -y curl gnupg   0.0s
 => CACHED [ 3/12] RUN curl -sSL ros.key | gpg --dearmor                  0.0s
 => CACHED [ 4/12] RUN apt-get update && apt-get install ros-humble      0.0s
 => [ 5/12] RUN apt-get update && apt-get install nvidia-cuda-toolkit   40.3s
 => [ 6/12] RUN apt-get update && apt-get install dev tools             15.2s
 => [ 7/12] RUN pip3 install fastapi uvicorn                             3.1s
 => [ 8/12] RUN groupadd && useradd dev                                  0.5s
 => [ 9/12] RUN rosdep init && rosdep update                            12.4s
 => [10/12] WORKDIR /home/dev/ros_ws                                     0.0s
 => [11/12] COPY entrypoint.sh /home/dev/entrypoint.sh                   0.1s
 => [12/12] RUN chmod +x /home/dev/entrypoint.sh                         0.3s
 => exporting to image                                                  45.2s

Image Size: 22.2GB (7.18GB compressed)
Build Time: ~5 minutes
```

### Result: ✅ PASS

---

## 2. Container Launch

### Command

```powershell
docker run -it --name ros_humble_dev `
  --network host `
  -v E:\codebase\ros2-humble\src:/home/dev/ros_ws/src `
  -e DISPLAY=host.docker.internal:0.0 `
  -e QT_X11_NO_MITSHM=1 `
  ros_humble_gpu
```

### Output

```
root@docker-desktop:/home/dev/ros_ws#
```

### Result: ✅ PASS

---

## 3. Workspace Build

### Command

```bash
colcon build --symlink-install
```

### Output

```
Starting >>> ros_fastapi_bridge
Starting >>> sample_cpp_pkg
Starting >>> sample_py_pkg
Finished <<< ros_fastapi_bridge [2.41s]
Finished <<< sample_py_pkg [2.44s]
Finished <<< sample_cpp_pkg [20.3s]

Summary: 3 packages finished [20.8s]
```

### Packages Built

| Package | Language | Build Time |
|---------|----------|------------|
| ros_fastapi_bridge | Python | 2.41s |
| sample_py_pkg | Python | 2.44s |
| sample_cpp_pkg | C++ | 20.3s |

### Result: ✅ PASS

---

## 4. C++ Talker/Listener Test

### Commands

```bash
# Terminal 1
ros2 run sample_cpp_pkg talker &

# Terminal 2
ros2 run sample_cpp_pkg listener
```

### Output

```
[INFO] [1764939663.762815309] [talker]: Talker node started
[INFO] [1764939664.262926199] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 0'
[INFO] [1764939664.726254723] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 1'
[INFO] [1764939665.192234489] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 2'
[INFO] [1764939665.398293664] [listener]: Listener node started
[INFO] [1764939665.642953749] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 3'
[INFO] [1764939665.643461513] [listener]: Received: 'Hello from C++ ROS 2! Count: 3'
[INFO] [1764939666.124928710] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 4'
[INFO] [1764939666.125376989] [listener]: Received: 'Hello from C++ ROS 2! Count: 4'
[INFO] [1764939666.587476314] [talker]: Publishing: 'Hello from C++ ROS 2! Count: 5'
[INFO] [1764939666.587928136] [listener]: Received: 'Hello from C++ ROS 2! Count: 5'
```

### Metrics

- **Publish Rate:** 2 Hz (500ms interval)
- **Latency:** < 1ms (same machine)
- **Messages Delivered:** 100%

### Result: ✅ PASS

---

## 5. Sensor Fusion Pipeline Test

### Command

```bash
ros2 launch sample_py_pkg sensor_fusion.launch.py
```

### Output

```
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-12-05
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [sensor_publisher-1]: process started with pid [337]
[INFO] [sensor_fusion-2]: process started with pid [339]
[sensor_publisher-1] [INFO] [sensor_publisher]: Sensor publisher started (rate: 10.0 Hz)
[sensor_fusion-2] [INFO] [sensor_fusion]: Sensor fusion node started (method: mean)
```

### Fused Data Output

```bash
ros2 topic echo /fused --once
data: 25.656349182128906
---
```

### Configuration

- **Base Value:** 25.0
- **Sensor A Noise:** σ = 0.5
- **Sensor B Noise:** σ = 1.0
- **Fusion Method:** Mean
- **Publish Rate:** 10 Hz

### Result: ✅ PASS

---

## 6. FastAPI Bridge Test

### Command

```bash
ros2 run ros_fastapi_bridge bridge
```

### Output

```
INFO:     Started server process [387]
INFO:     Waiting for application startup.
[INFO] [1764939825.633622597] [ros_fastapi_bridge]: ROS FastAPI Bridge node started
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

### API Tests

```bash
# Health Check
curl -s http://localhost:8000/health
{"status":"healthy","ros_node":true}

# Topics List
curl -s http://localhost:8000/topics
{"subscribed":["bridge/string_in","bridge/float_in"],"publishers":["bridge/string_out","cmd_vel"],"cached":[]}
```

### Endpoints Verified

| Endpoint | Method | Status |
|----------|--------|--------|
| `/health` | GET | ✅ 200 OK |
| `/topics` | GET | ✅ 200 OK |
| `/publish/string` | POST | ✅ 200 OK |
| `/publish/cmd_vel` | POST | ✅ 200 OK |
| `/docs` | GET | ✅ Swagger UI |

### Result: ✅ PASS

---

## 7. ROS Topics Inventory

### Command

```bash
ros2 topic list
```

### Output

```
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

### Topic Analysis

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| /chatter | String | talker | listener |
| /sensor_a | Float32 | sensor_publisher | sensor_fusion |
| /sensor_b | Float32 | sensor_publisher | sensor_fusion |
| /fused | Float32 | sensor_fusion | - |
| /cmd_vel | Twist | fastapi_bridge | - |
| /bridge/* | Various | fastapi_bridge | fastapi_bridge |

### Result: ✅ PASS

---

## Summary

| Test | Status | Notes |
|------|--------|-------|
| Docker Build | ✅ PASS | 22.2GB image |
| Container Launch | ✅ PASS | Immediate startup |
| Workspace Build | ✅ PASS | 3 packages in 20.8s |
| C++ Talker/Listener | ✅ PASS | 2Hz, <1ms latency |
| Sensor Fusion | ✅ PASS | Mean fusion working |
| FastAPI Bridge | ✅ PASS | REST API operational |
| ROS Topics | ✅ PASS | 10 active topics |

**Overall: ALL TESTS PASSED ✅**

---

## Environment Details

```
Host OS: Windows 10/11
Docker: Docker Desktop with WSL2
Base Image: nvidia/cuda:12.1.1-devel-ubuntu22.04
ROS Distro: Humble Hawksbill
Python: 3.10
GCC: 11.4.0
```
