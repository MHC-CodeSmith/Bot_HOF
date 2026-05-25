# TurtleBot4 Simulator (ROS 2 Jazzy + Gazebo Harmonic) — Docker

A **reproducible** environment for simulating TurtleBot4 using **Gazebo (Harmonic)**. This setup supports **SLAM (slam_toolbox)** for map generation, saving maps, and running **Localization + Navigation2 (Nav2)**.

> **Note:** Tested on Ubuntu 24.04 with NVIDIA GPU (OpenGL 4.6) using `--gpus all`.

## Project Structure

```text
turtlebot4_jazzy/
├── sim/                    # Dockerfile, run scripts and Gazebo world
├── maps/                   # Saved maps (yaml/pgm)
├── params/                 # Nav2 and waypoint parameters
├── scripts/                # Mission, diagnostics and benchmark helpers
├── factory_integration/    # OPC-UA/factory UI experiments
├── vision/                 # Vision datasets/assets
└── README.md               # Documentation
```

## Requirements

- **Docker** and **NVIDIA Container Toolkit** (for GPU acceleration)
- **X11 Server** active (required for RViz2 and Gazebo GUI)
- **Internet connection** (to download worlds and ROS/Gazebo packages)

## Build

Build the Docker image:

```bash
cd sim
docker build --no-cache -t turtlebot4:jazzy .
```

## Run Container

Start the container with GPU and X11 forwarding enabled:

```bash
./sim/run_docker.sh
```

Inside the container, source the ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
```

## Usage

### 1. SLAM + Nav2 + RViz (Mapping)

To start the simulation with SLAM and mapping enabled:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  slam:=true nav2:=true rviz:=true
```

**In RViz:**
- Set `Fixed Frame = map`.
- Add **LaserScan** (`/scan`) and **Map** (`/map`) displays if they are not already present.

**Verification:**
```bash
ros2 topic hz /scan
ros2 topic echo /map --once --qos-durability transient_local
```

You can now drive the robot using the Teleop panel in the Gazebo GUI or via `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

### 2. Saving the Map

If the container was started with the volume mount `-v $PWD/maps:/root/maps` (default in `run_docker.sh`), you can save the generate map to the host machine:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/warehouse
```
This will generate `maps/warehouse.yaml` and `maps/warehouse.pgm`.

### 3. Localization + Nav2 (Navigation)

To run navigation with a pre-existing map (disable SLAM):

```bash
# Optional: Launch simulation separately if needed
# ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

# Launch Localization (AMCL) providing the map path
ros2 launch turtlebot4_navigation localization.launch.py map:=/root/maps/warehouse.yaml

# Launch Nav2
ros2 launch turtlebot4_navigation nav2.launch.py
```

**In RViz:**
1. Use **2D Pose Estimate** to set the initial robot position.
2. Set a destination using **2D Goal Pose** or the Navigation2 panel.

## Troubleshooting

- **Broken TF (`map -> odom -> base_link`)**:
  Run `view_frames` to verify the transform tree is connected.
  ```bash
  ros2 run tf2_tools view_frames
  ```

- **Map not appearing**:
  Ensure the QoS profile is set to `TRANSIENT_LOCAL`.
  ```bash
  ros2 topic echo /map --once --qos-durability transient_local
  ```

- **GUI Freezing / Crashing**:
  Verify GPU acceleration inside the container:
  ```bash
  apt-get update && apt-get install -y mesa-utils
  glxinfo | grep -E "OpenGL renderer|OpenGL version"
  ```
