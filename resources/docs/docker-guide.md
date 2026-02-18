# Docker Troubleshooting and Usage Guide

This guide covers how to use Docker to run PhysicalMCP's simulation stack, troubleshoot common issues, and customize the container setup for your needs.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Prerequisites](#prerequisites)
- [Starting the Stack](#starting-the-stack)
- [X11 Forwarding for Gazebo GUI](#x11-forwarding-for-gazebo-gui)
- [Common Issues and Solutions](#common-issues-and-solutions)
- [Building Custom Images](#building-custom-images)
- [Environment Variables](#environment-variables)
- [Advanced Usage](#advanced-usage)

---

## Architecture Overview

The Docker setup consists of three files in the `docker/` directory:

```
docker/
  ├── Dockerfile.bridge       # ROS2 Humble + Python bridge
  ├── Dockerfile.sim          # Gazebo + TurtleBot3 simulation
  └── docker-compose.yml      # Orchestrates both containers
```

### Dockerfile.bridge

Builds the Python ROS2 bridge that translates WebSocket commands from the MCP server into native ROS2 operations.

- **Base image:** `ros:humble` (ROS2 Humble on Ubuntu 22.04)
- **Installs:** Python 3, pip, the `physical-mcp-bridge` package and its dependencies (`websockets>=12.0`)
- **Exposes:** Port 9090 (WebSocket server)
- **Entry command:** Sources the ROS2 setup and runs `physical_mcp_bridge.bridge_node`

### Dockerfile.sim

Builds the Gazebo simulation environment with a TurtleBot3 robot.

- **Base image:** `ros:humble`
- **Installs:** `ros-humble-gazebo-ros-pkgs`, `ros-humble-turtlebot3-gazebo`, `ros-humble-turtlebot3-description`, `ros-humble-turtlebot3-navigation2`, `ros-humble-nav2-bringup`
- **Default model:** TurtleBot3 Burger (`TURTLEBOT3_MODEL=burger`)
- **Entry command:** Launches the `turtlebot3_world.launch.py` Gazebo world

### docker-compose.yml

Orchestrates both containers with the following configuration:

- **`sim` service:** Runs the Gazebo simulation with X11 display forwarding
- **`bridge` service:** Runs the ROS2 bridge, depends on `sim`
- **Network mode:** Both services use `network_mode: host`, meaning they share the host's network stack directly. This is required for ROS2 DDS discovery to work between the containers and allows the MCP server on the host to reach the bridge on port 9090 without explicit port mapping.
- **Restart policy:** `unless-stopped` for both services

---

## Prerequisites

Before using the Docker setup, ensure you have:

1. **Docker Engine** (version 20.10 or later)
   ```bash
   docker --version
   ```

2. **Docker Compose** (v2, included with modern Docker Desktop)
   ```bash
   docker compose version
   ```

3. **Disk space:** Approximately 5 GB free for the ROS2 Humble base images, Gazebo packages, and TurtleBot3 simulation files. The first build pulls these layers and caches them locally.

4. **X11 server** (optional, only needed if you want to see the Gazebo GUI window):
   - **macOS:** Install [XQuartz](https://www.xquartz.org/)
   - **Linux:** X11 is typically available by default

---

## Starting the Stack

### Full stack (simulation + bridge)

This is the most common way to get started. It launches both the Gazebo simulation and the ROS2 bridge:

```bash
cd docker
docker compose up
```

On the first run, Docker builds both images from scratch. This can take 10-15 minutes depending on your internet connection and hardware. Subsequent runs use cached layers and start in seconds.

Once running, you will see log output from both containers. The bridge listens on `ws://localhost:9090` and the MCP server can connect to it.

To run in detached mode (background):

```bash
docker compose up -d
```

To view logs when running in detached mode:

```bash
docker compose logs -f          # all services
docker compose logs -f bridge   # bridge only
docker compose logs -f sim      # simulation only
```

### Bridge only (no simulation)

If you already have a ROS2 environment running (real hardware, a separately launched simulation, or another source of ROS2 topics), you can start just the bridge:

```bash
docker compose up bridge
```

The bridge still uses `network_mode: host`, so it will discover ROS2 nodes on the host's DDS network automatically. Note that `docker compose up bridge` will also start the `sim` service because `bridge` has a `depends_on: sim` declaration. To truly run only the bridge without the simulation, you can run it directly:

```bash
docker compose run --rm --service-ports bridge
```

Or temporarily modify the `docker-compose.yml` to remove the `depends_on` entry.

### Stopping the stack

```bash
docker compose down
```

To also remove the built images:

```bash
docker compose down --rmi local
```

### Rebuilding after code changes

If you modify the Python bridge code in `packages/ros2-bridge/`, rebuild the bridge image:

```bash
docker compose up --build bridge
```

To rebuild everything:

```bash
docker compose up --build
```

---

## X11 Forwarding for Gazebo GUI

The Gazebo simulation renders a 3D window. To see it, you need X11 forwarding configured on your host.

### Linux

Grant Docker access to your X11 display:

```bash
xhost +local:root
```

Then start the stack normally:

```bash
docker compose up
```

The `docker-compose.yml` already mounts `/tmp/.X11-unix` and passes the `DISPLAY` environment variable. Gazebo should open a window on your desktop.

To revoke access after you are done:

```bash
xhost -local:root
```

### macOS

macOS does not have a native X11 server. You need XQuartz:

1. **Install XQuartz:**
   ```bash
   brew install --cask xquartz
   ```
   Or download from [xquartz.org](https://www.xquartz.org/).

2. **Log out and back in** (required after first XQuartz install for the `DISPLAY` variable to be set).

3. **Start XQuartz** and enable network connections:
   - Open XQuartz
   - Go to **Preferences > Security**
   - Check **"Allow connections from network clients"**

4. **Grant access and set DISPLAY:**
   ```bash
   xhost +local:docker
   export DISPLAY=host.docker.internal:0
   ```

5. **Start the stack:**
   ```bash
   docker compose up
   ```

**Note:** X11 forwarding over the network on macOS introduces latency. The Gazebo GUI may feel sluggish. For development, this is acceptable. If you do not need the visual display, the simulation still runs headlessly and publishes all ROS2 topics normally without X11.

### Running headless (no GUI)

If you do not need the Gazebo visual window, you can run the simulation headless by setting the `DISPLAY` variable to empty:

```bash
DISPLAY= docker compose up
```

Gazebo will run in server-only mode. All ROS2 topics (`/cmd_vel`, `/odom`, `/scan`, etc.) still function. The physics simulation still runs. You simply do not see the 3D rendering.

---

## Common Issues and Solutions

### Port 9090 already in use

**Symptom:** The bridge container fails to start with an error like `Address already in use` or `OSError: [Errno 98] Address already in use`.

**Cause:** Another process on your host is already listening on port 9090. This is common if you have a previous bridge instance running, another WebSocket server, or `rosbridge_suite` installed.

**Solution:**

1. Find what is using port 9090:
   ```bash
   # Linux
   sudo lsof -i :9090

   # macOS
   lsof -i :9090
   ```

2. Stop the conflicting process, or change the bridge port by editing the bridge startup command in `Dockerfile.bridge` or passing a port argument.

3. If a previous Docker stack is still running:
   ```bash
   docker compose down
   docker compose up
   ```

### Gazebo display issues

**Symptom:** Gazebo crashes on startup, shows a black window, or produces OpenGL errors.

**Cause:** X11 forwarding is not configured, or the GPU drivers inside the container do not match the host.

**Solutions:**

- Verify X11 access is granted (see [X11 Forwarding](#x11-forwarding-for-gazebo-gui) section above).
- If you see OpenGL errors, try running with software rendering:
  ```bash
  # Add to docker-compose.yml under the sim service environment:
  - LIBGL_ALWAYS_SOFTWARE=1
  ```
- If you do not need the GUI, run headless:
  ```bash
  DISPLAY= docker compose up
  ```

### Bridge starts before simulation is ready

**Symptom:** The bridge starts and connects, but `ros2_topic_list` returns empty or very few topics. The TurtleBot3 topics (`/cmd_vel`, `/odom`, `/scan`) are missing.

**Cause:** The bridge container starts as soon as the `sim` container's process has launched, but the Gazebo simulation takes additional time to fully load the world, spawn the robot model, and advertise all ROS2 topics. During this window, the bridge sees an incomplete ROS2 graph.

**Solution:** Restart the bridge after the simulation is fully loaded. You will know the simulation is ready when you see Gazebo log messages indicating the world has loaded (or the Gazebo GUI window appears with the robot visible).

```bash
docker compose restart bridge
```

This is a known timing issue with `depends_on` in Docker Compose, which only waits for the container to start, not for the application inside to be fully ready. A health-check-based solution is possible but not yet implemented.

### Disk space issues

**Symptom:** Docker build fails with "no space left on device" or images fail to pull.

**Cause:** The ROS2 Humble base images and Gazebo packages are large (several GB combined).

**Solution:**

1. Check available Docker disk space:
   ```bash
   docker system df
   ```

2. Clean up unused images and build cache:
   ```bash
   docker system prune -a
   ```
   **Warning:** This removes all unused images, not just PhysicalMCP ones. Use with caution if you have other Docker projects.

3. Ensure you have at least 5 GB free before building.

### Container exits immediately

**Symptom:** `docker compose up` starts the containers but one or both exit right away.

**Cause:** Usually a missing dependency, a build error, or a ROS2 sourcing issue.

**Solution:**

1. Check the container logs:
   ```bash
   docker compose logs sim
   docker compose logs bridge
   ```

2. If you see Python import errors in the bridge logs, rebuild:
   ```bash
   docker compose up --build
   ```

3. If the sim container crashes, it may be a GPU/display issue. Try running headless.

### ROS2 DDS communication issues between containers

**Symptom:** The bridge is running and Gazebo is running, but the bridge cannot see any ROS2 topics from the simulation.

**Cause:** DDS discovery requires the containers to be on the same network and share the same `ROS_DOMAIN_ID`.

**Solution:** Both services use `network_mode: host`, which means they share the host's network namespace and DDS discovery should work automatically. If you have changed the network configuration:

1. Verify both containers use `network_mode: host` in `docker-compose.yml`.
2. Ensure `ROS_DOMAIN_ID` is the same in both containers (default is 0 if unset).
3. Check that no DDS-related environment variables (`ROS_LOCALHOST_ONLY`, `FASTRTPS_DEFAULT_PROFILES_FILE`, etc.) are interfering.

---

## Building Custom Images

### Modifying the bridge image

To add Python packages to the bridge container, edit `Dockerfile.bridge`:

```dockerfile
FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY packages/ros2-bridge/ /app/
RUN pip3 install --no-cache-dir -r requirements.txt
RUN pip3 install -e .

# Add your custom packages here:
RUN pip3 install numpy scipy

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

EXPOSE 9090

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python3 -m physical_mcp_bridge.bridge_node"]
```

### Modifying the simulation image

To use a different TurtleBot3 model (Waffle, Waffle Pi) or a different Gazebo world, edit `Dockerfile.sim`:

```dockerfile
# Change the model
ENV TURTLEBOT3_MODEL=waffle

# Or change the launch file for a different world
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && export TURTLEBOT3_MODEL=waffle && ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py"]
```

Available TurtleBot3 worlds:
- `turtlebot3_world.launch.py` (default, small room with obstacles)
- `turtlebot3_house.launch.py` (house layout)
- `turtlebot3_empty_world.launch.py` (empty plane)

### Adding additional ROS2 packages to the simulation

To install more ROS2 packages (for example, SLAM or additional sensor plugins):

```dockerfile
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    && rm -rf /var/lib/apt/lists/*
```

After modifying any Dockerfile, rebuild:

```bash
docker compose up --build
```

---

## Environment Variables

The `docker-compose.yml` file passes the following environment variables to the containers. You can override them when starting the stack.

### Simulation container (`sim`)

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | `${DISPLAY}` (from host) | X11 display for Gazebo GUI. Set to empty for headless mode. |
| `TURTLEBOT3_MODEL` | `burger` | TurtleBot3 model variant: `burger`, `waffle`, or `waffle_pi`. |

### Bridge container (`bridge`)

The bridge container does not have explicit environment variables in the default `docker-compose.yml`, but you can add:

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID. Must match the simulation container if set. |
| `ROS_LOCALHOST_ONLY` | unset | Set to `1` to restrict DDS discovery to localhost. |

### Overriding at runtime

```bash
# Use Waffle model instead of Burger
TURTLEBOT3_MODEL=waffle docker compose up

# Run headless
DISPLAY= docker compose up

# Set a custom ROS domain
ROS_DOMAIN_ID=42 docker compose up
```

### Adding variables to docker-compose.yml

To persist variable overrides, add them to the `environment` section of the relevant service:

```yaml
services:
  bridge:
    build:
      context: ..
      dockerfile: docker/Dockerfile.bridge
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
    depends_on:
      - sim
    restart: unless-stopped
```

---

## Advanced Usage

### Running the MCP server on the host while Docker provides the simulation

This is the standard development workflow:

1. Start the Docker simulation and bridge:
   ```bash
   cd docker
   docker compose up -d
   ```

2. On your host machine, start the MCP server pointing at the bridge:
   ```bash
   PHYSICAL_MCP_BRIDGE_URL=ws://localhost:9090 npx @ricardothe3rd/physical-mcp
   ```

   Since the bridge uses `network_mode: host`, port 9090 is directly accessible on localhost.

### Connecting to a remote Docker stack

If the Docker stack is running on another machine (for example, a development server):

```bash
PHYSICAL_MCP_BRIDGE_URL=ws://REMOTE_IP:9090 npx @ricardothe3rd/physical-mcp
```

Ensure port 9090 is accessible through any firewalls between the machines.

### Viewing ROS2 topics from the host

If you have ROS2 installed on your host, you can directly interact with the simulation's ROS2 graph because the containers use `network_mode: host`:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /odom
```

### Multiple simultaneous simulations

To run multiple isolated simulations, use different `ROS_DOMAIN_ID` values and bridge ports. This requires modifying the compose file or creating separate compose files for each simulation instance.
