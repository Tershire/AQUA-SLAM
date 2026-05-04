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

> Full per-sensor axis definitions and the complete transform chain are in [coordinate_frames.md](coordinate_frames.md).

All SLAM pose topics (`orb_pose`, `orb_odom`, `orb_path`) express camera pose in a **world frame** `w` defined at SLAM initialization:

```
T_w_cj  =  mT_w_c0  *  T_c0_cj
```

| Symbol | Meaning |
|---|---|
| `T_c0_cj` | Camera pose from ORB-SLAM3 (raw internal frame, `c0` = first keyframe) |
| `mT_w_c0` | Fixed transform from ORB-SLAM3 raw frame to world frame, set once at map initialization |
| `T_w_cj` | Camera pose in world frame, published in output topics |

`mT_w_c0` is computed in `PublishMap` from:
- **Rotation**: aligns ORB-SLAM3 z-axis with gravity (`R_b0_w` from IMU initialization) and yaw-aligns with the initial forward direction
- **Translation**: `R_b0_w⁻¹ * T_imu_c.translation()` — offsets world origin to account for the IMU-to-camera lever arm

### orb_odom vs orb_path pose source

| Topic | Pose source | Notes |
|---|---|---|
| `orb_odom`, `orb_odom_body` | `mCurrentFrame.GetPoseInverse()` | Per-frame tracking; **visual-only** (`PoseOptimization`), not BA-refined |
| `orb_path`, `orb_path_body` | `pKF->GetPoseInverse()` | Keyframe poses; **BA-refined** by `LocalDVLIMUBundleAdjustment` |

Expected position difference at the same timestamp: **5–20 mm** (pre-BA tracking vs. BA-refined keyframe). Occasional spikes up to ~50 mm near keyframe creation or loop closure events.

### Body frame topics

`orb_odom_body` and `orb_path_body` apply an additional transform to express pose in the body FLU frame:

```
T_w_bj  =  T_w_cj  *  T_imu_c⁻¹  *  T_body_imu⁻¹
```

where `T_imu_c` and `T_body_imu` are read from the sensor YAML config.

---

## Timestamp Convention

All output pose topic header stamps use the **left camera hardware timestamp**, matching the original ROS1 design:

| Topic | Timestamp source |
|---|---|
| `orb_odom`, `orb_odom_body` | `mCurrentFrame.mTimeStamp` = `tImLeft` = `imgLeft.header.stamp` |
| `orb_path`, `orb_path_body` | `pKF->mTimeStamp` = `KeyFrame::mTimeStamp` = `Frame::mTimeStamp` = same chain |
| `dvl_imu_pose`, `dvl_imu_path` | `pKF->mTimeStamp` (same) |

For **bag replay**, all output timestamps match the bag's sensor time (e.g. 2022 for `short_test.bag`).  
For **live camera**, timestamps match the camera's system-synced clock.

In both cases `orb_odom` and `orb_path` share the same time reference and can be directly compared on the same axis.

### orb_odom vs orb_path time offset in recorded bags

`orb_path` is rebuilt on every `PublishMap` call and contains the **full keyframe history** since SLAM start.  
`orb_odom` is only recorded into the bag from the moment bag recording begins.

If recording starts a few seconds after SLAM initialization, `orb_path` will have an earlier first timestamp than `orb_odom`. The `plot_ros2_bag_metrics.py` script normalizes all topics sharing the same time base to a **common t0** (= minimum first timestamp in the group), so this gap is visible as empty space at the left of the orb_odom time series.
