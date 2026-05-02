# AQUA-SLAM Topic Structure

Comparison of original ROS1 (`main` branch) and current ROS2 (`ros2_jazzy` branch) topic structures.

---

## Published Topics

| ROS1 (`/AQUA_SLAM/…`) | ROS2 (`/aqua_slam/…`) | Message Type | Rate | Notes |
|---|---|---|---|---|
| `left/image_raw` | `left/image_raw` | `sensor_msgs/Image` | frame rate | Left camera raw image |
| `right/image_raw` | `right/image_raw` | `sensor_msgs/Image` | frame rate | Right camera raw image |
| `img_with_info` | `image/features` | `sensor_msgs/Image` | frame rate | Left image with feature overlay |
| `img_merge_cand` | `image/map_merge` | `sensor_msgs/Image` | on event | Map merge candidate image |
| `orb_pose` | `orb_pose` | `geometry_msgs/PoseStamped` | frame rate | Per-frame tracking pose, camera frame — **visual-only** (`PoseOptimization`); per-frame tightly coupled tracking is `todo_tightly` |
| `orb_odom` | `orb_odom` | `nav_msgs/Odometry` | frame rate | Same pose as `orb_pose`; ROS2 adds `twist.twist.linear` = world-frame body velocity (`mVw`) from DVL+IMU propagation |
| `orb_path` | `orb_path` | `nav_msgs/Path` | ~4 Hz | Keyframe trajectory rebuilt after `LocalDVLIMUBundleAdjustment` — **tightly coupled** DVL+IMU+visual BA result |
| `camera_pose` | `camera_pose` | ~~`nav_msgs/Odometry`~~ → `geometry_msgs/PoseStamped` | — | **Not published** — publisher registered but function body is `#if 0`; type changed in ROS2 migration |
| — | `orb_odom_body` | `nav_msgs/Odometry` | frame rate | `orb_odom` transformed to body FLU frame: `T_w_b = T_w_c * T_b_c⁻¹`; `twist` in body frame |
| — | `orb_path_body` | `nav_msgs/Path` | ~4 Hz | `orb_path` keyframes transformed to body FLU frame |
| `sparse_map` | `sparse_map` | `sensor_msgs/PointCloud2` | ~4 Hz | Visual map points |
| `octomap` | `octomap` | `octomap_msgs/Octomap` | ~4 Hz | 3D occupancy map |
| `integration_path` | `dvl_imu_path` | `nav_msgs/Path` | ~4 Hz | DVL+IMU dead-reckoning trajectory |
| `ref_integration_path` | `dvl_imu_path_ref` | `nav_msgs/Path` | ~4 Hz | DVL+IMU reference trajectory |
| `integration_cur` | `dvl_imu_pose` | `geometry_msgs/PoseStamped` | ~4 Hz | Current DVL+IMU integrated pose |
| `integration_ref` | `dvl_imu_pose_ref` | `geometry_msgs/PoseStamped` | ~4 Hz | Reference DVL+IMU pose |
| `markers` | `markers` | `visualization_msgs/MarkerArray` | ~4 Hz | Map visualization markers |
| `ekf_path` | `ekf_path` | `nav_msgs/Path` | — | EKF trajectory (placeholder, unused) |

### TF Broadcast

| ROS1 | ROS2 | Rate |
|---|---|---|
| `AQUA_SLAM` → `/bluerov/base_link` | `aqua_slam` → `/bluerov/base_link` | frame rate |

---

## Services

| ROS1 (`/AQUA_SLAM/…`) | ROS2 (`/aqua_slam/…`) | Type | Description |
|---|---|---|---|
| `save` | `save` | `std_srvs/Empty` | Save keyframe trajectory to file |
| `load_map` | `load_map` | `std_srvs/Empty` | Load a previously saved map |
| `calibrate` | `calibrate` | `std_srvs/Empty` | Calibrate DVL/gyro bias |
| `fullBA` | `full_ba` | `std_srvs/Empty` | Trigger full bundle adjustment |

---

## Key Differences: ROS1 → ROS2

1. **Namespace**: `/AQUA_SLAM/` → `/aqua_slam/` (lowercase, ROS2 convention)
2. **Renamed topics** for clarity:
   - `img_with_info` → `image/features`
   - `img_merge_cand` → `image/map_merge`
   - `integration_path` / `ref_integration_path` → `dvl_imu_path` / `dvl_imu_path_ref`
   - `integration_cur` / `integration_ref` → `dvl_imu_pose` / `dvl_imu_pose_ref`
   - Service `fullBA` → `full_ba`
3. **`orb_odom` twist**: ROS1 has no twist field populated. ROS2 adds `twist.twist.linear` = world-frame body velocity (`mVw`) from IMU preintegration propagation.
4. **`camera_pose`**: ROS1 type was `nav_msgs/Odometry`; ROS2 changed to `geometry_msgs/PoseStamped`, but function body is disabled — topic is not published in either version.
5. **TF frame ID**: `AQUA_SLAM` → `aqua_slam`

---

## Tight Coupling: Tracking vs. LocalMapping

AQUA-SLAM uses `DVL_STEREO` sensor mode. The "tightly coupled" claim applies at two different levels:

| Stage | Optimization | Sensors used | Topics |
|---|---|---|---|
| Per-frame tracking | `PoseOptimization` (visual only) | Camera | `orb_pose`, `orb_odom` (pose) |
| LocalMapping BA | `LocalDVLIMUBundleAdjustment` | Camera + DVL + IMU | `orb_path` (keyframes) |

The per-frame DVL+gyro tightly coupled tracker (`TrackLocalMapWithDvlGyro`) is marked `//todo_tightly` and is not active. For the current pose published in `orb_pose`/`orb_odom`, the **pose** is visual-only; only the **velocity** in `orb_odom.twist` comes from DVL+IMU propagation.

---

## Note on Dataset Distribution Topics

The [underwater tank dataset page](https://senseroboticslab.github.io/underwater-tank-dataset/format/) lists `/aqua_slam/pose` (2.5 Hz, `nav_msgs/Odometry`). This topic does **not** exist in the codebase (ROS1 or ROS2) — it was remapped or post-processed for dataset distribution and is not produced by `aqua_slam_node` directly.

---

## Coordinate Frame Convention

All SLAM pose topics (`orb_pose`, `orb_odom`, `orb_path`) are in the **camera frame** (`c`):

```
T_w_c  — world origin at first keyframe camera position
```

For robot control applications requiring the **body frame** (`b`), apply:

```
T_w_b = T_w_c * T_gyro_c⁻¹
```

where `T_imu_c` is defined in the sensor YAML config file, and `T_body_imu` is an optional YAML parameter (default identity if absent) for cases where the body and IMU frames differ.
