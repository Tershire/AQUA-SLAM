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
xhost +local:root
docker compose up -d
```

`xhost +local:root` allows the container (running as root) to open GUI windows on the host X server. This resets on reboot, so run it every time before starting the container.

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
- `dvl_converter` — converts `/dvl/data` (WaterLinked A50) → `/bluerov2/dvl` (nav_msgs/Odometry)
- `robot_state_publisher` — publishes robot URDF
- `static_transform_publisher` — `odom` → `orb_slam` static TF
- `rviz2` — visualizer with pre-configured layout

To disable RViz2:

```bash
ros2 launch aqua_slam blue_gx5_StructureEasy.launch.py use_rviz:=false
```

### 4a. Launch SLAM + Record Results (for analysis)

To automatically record SLAM output to a result bag under `results/`:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch aqua_slam blue_gx5_StructureEasy_record.launch.py
```

This runs everything in Step 4, plus records the following topics to `results/slam_YYYYMMDD_HHMMSS/`:

| Topic | Description |
|---|---|
| `/aqua_slam/orb_odom` | SLAM pose + velocity (camera frame) |
| `/aqua_slam/orb_path` | SLAM trajectory (camera frame) |
| `/aqua_slam/orb_odom_body` | SLAM pose + velocity (body FLU frame) |
| `/aqua_slam/orb_path_body` | SLAM trajectory (body FLU frame) |
| `/aqua_slam/dvl_imu_pose` | DVL+IMU dead-reckoning pose |
| `/aqua_slam/dvl_imu_path` | DVL+IMU dead-reckoning trajectory |
| `/aqua_slam/dvl_imu_pose_ref` | DVL+IMU reference pose |
| `/aqua_slam/dvl_imu_path_ref` | DVL+IMU reference trajectory |
| `/apriltag_slam/GT` | Ground truth (from input bag) |

> **Note:** To ensure the result bag is properly finalized (metadata written), **close the RViz2 window** before pressing Ctrl+C to stop the launch. If the launch is killed while RViz2 is still open, `ros2 bag record` may not finish writing and the bag will be corrupt.

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
| PointCloud2 (sparse map) | `/aqua_slam/sparse_map` | Appears after SLAM initialization (~3 s into bag) |
| Path (camera trajectory) | `/aqua_slam/orb_path` | Drawn as SLAM tracks keyframes |
| Pose (DVL+IMU dead-reckoning) | `/aqua_slam/dvl_imu_pose` | Current DVL+IMU integrated pose |
| Path (DVL+IMU dead-reckoning) | `/aqua_slam/dvl_imu_path` | Full DVL+IMU integrated trajectory |
| Image | `/aqua_slam/image/features` | Left camera with feature overlay |

SLAM initialization requires:
- Feature count > 500 in the current frame
- At least one DVL measurement received (DVL data starts ~3 s into the bag)

---

## Topic Overview

| Topic | Type | Description |
|---|---|---|
| `/aqua_slam/sparse_map` | `sensor_msgs/PointCloud2` | Visual map points |
| `/aqua_slam/orb_path` | `nav_msgs/Path` | ORB-SLAM camera path |
| `/aqua_slam/orb_pose` | `geometry_msgs/PoseStamped` | Current ORB-SLAM pose |
| `/aqua_slam/orb_odom` | `nav_msgs/Odometry` | Current ORB-SLAM odometry |
| `/aqua_slam/dvl_imu_pose` | `geometry_msgs/PoseStamped` | Current DVL+IMU dead-reckoning pose |
| `/aqua_slam/dvl_imu_path` | `nav_msgs/Path` | DVL+IMU dead-reckoning trajectory |
| `/aqua_slam/image/features` | `sensor_msgs/Image` | Left image with feature overlay |

TF tree: `aqua_slam` → `/bluerov/base_link` (broadcast by `aqua_slam_node`)

---

## 7. Plot Results

After running Step 4a and Step 5, plot the result bag from the **host** (outside the container):

```bash
cd src/AQUA-SLAM
python3 tools/plot_ros2_bag_metrics.py results/slam_YYYYMMDD_HHMMSS/
```

Plots and CSVs are saved to `results/slam_YYYYMMDD_HHMMSS_plots/`.
