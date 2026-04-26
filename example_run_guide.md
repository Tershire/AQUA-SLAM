# AQUA-SLAM Example Run Guide (ROS2 Jazzy)

## Prerequisites

- Docker + Docker Compose
- NVIDIA GPU (for RViz2 / OpenGL)
- X server running on the host (`DISPLAY` set)
- Dataset placed under `dataset/` inside the AQUA-SLAM source directory

Tested dataset: `dataset/short_test_ros2/` (15-second stereo + IMU + DVL bag)

---

## 1. Build Docker Image

```bash
cd docker/ros2_jazzy
docker compose build
```

This installs all dependencies and pre-builds `waterlinked_a50_ros_driver` inside the image.

---

## 2. Start Container

```bash
docker compose up -d
```

The AQUA-SLAM source is mounted at `/root/ros2_ws/src/AQUA-SLAM` inside the container.

---

## 3. Build AQUA-SLAM

Enter the container and build:

```bash
docker exec -it aqua_slam_ros2_dev bash
```

Inside the container:

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aqua_slam
source install/setup.bash
```

---

## 4. Launch SLAM + RViz2

In one terminal (inside the container):

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch aqua_slam blue_gx5_StructureEasy.launch.py
```

This starts:
- `aqua_slam_node` — main SLAM node (stereo + IMU + DVL)
- `dvl_converter` — converts `/dvl/data` (WaterLinked A50) → `/bluerov2/DVL` (nav_msgs/Odometry)
- `robot_state_publisher` — publishes robot URDF
- `static_transform_publisher` — `odom` → `orb_slam` static TF
- `rviz2` — visualizer with pre-configured layout

To disable RViz2:

```bash
ros2 launch aqua_slam blue_gx5_StructureEasy.launch.py use_rviz:=false
```

---

## 5. Play Dataset

In a second terminal (inside the container):

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 bag play /root/ros2_ws/src/AQUA-SLAM/dataset/short_test_ros2/
```

> **Note:** The workspace must be sourced before `ros2 bag play` so that the bag player
> can deserialize `waterlinked_a50_ros_driver/msg/DVL` messages from the bag.

---

## 6. What to Expect in RViz2

| Display | Topic | Notes |
|---|---|---|
| PointCloud2 (sparse map) | `/AQUA_SLAM/sparse_map` | Appears after SLAM initialization (~3 s into bag) |
| Path (camera trajectory) | `/AQUA_SLAM/orb_path` | Drawn as SLAM tracks keyframes |
| Pose (DVL integration) | `/AQUA_SLAM/integration_cur` | Current DVL-integrated pose |
| Path (DVL integration) | `/AQUA_SLAM/integration_path` | Full DVL-integrated trajectory |
| Image | `/AQUA_SLAM/img_with_info` | Left camera with feature overlay |

SLAM initialization requires:
- Feature count > 500 in the current frame
- At least one DVL measurement received (DVL data starts ~3 s into the bag)

---

## Topic Overview

| Topic | Type | Description |
|---|---|---|
| `/AQUA_SLAM/sparse_map` | `sensor_msgs/PointCloud2` | Visual map points |
| `/AQUA_SLAM/orb_path` | `nav_msgs/Path` | ORB-SLAM camera path |
| `/AQUA_SLAM/orb_pose` | `geometry_msgs/PoseStamped` | Current ORB-SLAM pose |
| `/AQUA_SLAM/orb_odom` | `nav_msgs/Odometry` | Current ORB-SLAM odometry |
| `/AQUA_SLAM/integration_cur` | `geometry_msgs/PoseStamped` | Current DVL-integrated pose |
| `/AQUA_SLAM/integration_path` | `nav_msgs/Path` | DVL-integrated trajectory |
| `/AQUA_SLAM/img_with_info` | `sensor_msgs/Image` | Left image with feature overlay |

TF tree: `AQUA_SLAM` → `/bluerov/base_link` (broadcast by `aqua_slam_node`)
