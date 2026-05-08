# AQUA-SLAM Coordinate Frames

Axis orientations and transform conventions for all sensor and output frames.

---

## Transform Naming Convention

All transform variables follow the subscript convention:

```
T_A_B  ≡  T_{target}_{source}
```

Transforms a point from frame B to frame A:

```
p_A = T_A_B * p_B
```

Examples: `T_imu_c` transforms from camera to IMU; `T_body_imu` transforms from IMU to body.

---

## Sensor Frames

### Camera frame (`c`) — RDF

Standard computer-vision / ORB-SLAM3 convention:

```
      Z (Forward, into scene)
     /
    /
   +--------> X (Right)
   |
   |
   v
   Y (Down)
```

| Axis | Direction |
|------|-----------|
| X    | Right     |
| Y    | Down      |
| Z    | Forward (out of lens toward scene) |

### DVL frame (`d`) — FRD

WaterLinked A50 instrument frame (derived from `T_dvl_c`):

```
       Z (Down)
       |
       |
       +--------> Y (Right)
      /
     /
    X (Forward)
```

| Axis | Direction | Cam equivalent |
|------|-----------|----------------|
| X    | Forward   | +cam Z         |
| Y    | Right     | +cam X         |
| Z    | Down      | +cam Y         |

### IMU frame (`imu`) — as mounted

The GX5 IMU is physically mounted so that (derived from `T_imu_c`):

| IMU axis | Physical direction | Cam equivalent |
|----------|--------------------|----------------|
| X        | Backward           | −cam Z         |
| Y        | Down               | +cam Y         |
| Z        | Right              | +cam X         |

> `T_imu_c` in the YAML captures this mount geometry exactly.  
> The native GX5 sensor frame (before mounting rotation) is FRD.

---

## Robot Frames

### Body frame (`b`) — FLU

Standard ROS body convention (REP-103), defined by `T_body_imu`:

```
       Z (Up)
       |
       |
       +--------> Y (Left)
      /
     /
    X (Forward)
```

| Axis | Direction |
|------|-----------|
| X    | Forward   |
| Y    | Left      |
| Z    | Up        |

Derived from cam→body mapping:
- cam X (Right) → −body Y → body Y = Left ✓
- cam Y (Down) → −body Z → body Z = Up ✓
- cam Z (Forward) → +body X = Forward ✓

---

## SLAM Output Frame

### World frame (`w`) — local ENU-like

Set once at SLAM initialization by `mT_w_c0` (see `RosHandling::PublishMap`):

| Axis | Direction |
|------|-----------|
| X    | Initial forward direction (yaw-aligned to body heading at init) |
| Y    | Left (right-hand rule) |
| Z    | Up (gravity-aligned via IMU initialization) |

Not tied to Earth's north — purely a local frame anchored to the robot's heading at startup.

Origin: approximately at the IMU position at initialization (includes `T_imu_c` lever-arm offset).

### ORB-SLAM3 internal frame (`c0`)

Raw output frame of ORB-SLAM3, origin at the first keyframe's camera position.  
Converted to world frame `w` via `mT_w_c0`:

```
T_w_cj = mT_w_c0 * T_c0_cj
```

`mT_w_c0` is rotation-only (plus lever-arm translation) and is computed from the gravity vector and initial heading at SLAM initialization.

---

## Full Transform Chain

```
world (w)
  ↑  mT_w_c0  (set at SLAM init from gravity + yaw alignment)
c0 (ORB-SLAM3 internal)
  ↑  T_c0_cj  (ORB-SLAM3 pose output, GetPoseInverse())
camera (c)
  ↑  T_imu_c  (calibration YAML)
IMU (imu)
  ↑  T_body_imu  (calibration YAML, optional; identity if absent)
body (b)
```

For the DVL:

```
camera (c)
  ↑  T_dvl_c  (calibration YAML)
DVL (d)
```

Body-frame pose from camera-frame pose:

```
T_w_bj = T_w_cj * T_imu_c⁻¹ * T_body_imu⁻¹
```

---

## Initial Body-Frame Attitude at SLAM Start

At SLAM initialization the world frame is set by `mT_w_c0` (see `topic_structure.md`). Even so, the body-frame RPY reported in `orb_odom_body` / `orb_path_body` at the first keyframe is generally **not** (0°, 0°, 0°) for three reasons:

1. **Actual robot tilt** — World Z is gravity-aligned via the IMU, so any physical roll/pitch of the robot at initialization shows up directly in the body-frame attitude. A few degrees of roll/pitch is normal in underwater operation (buoyancy not yet settled, uneven tank floor, etc.).

2. **Yaw residual** — `mT_w_c0` yaw-aligns the ORB-SLAM3 internal frame (first camera keyframe) to the world X axis. After the cam→body transform (`T_imu_c`, `T_body_imu`) is applied, a small yaw residual can remain if the camera's initial forward direction is not exactly collinear with the body's X axis.

3. **IMU initialization latency** — IMU gravity alignment and the first ORB keyframe may not be perfectly co-timed; any attitude change in that window propagates to the initial pose.

This is normal behavior, not a bug. The world frame is an absolute reference anchored to gravity and the robot's heading at startup — it does not force the robot to start at identity.

---

## Calibration Parameters (from YAML)

| Parameter    | Meaning                                 | Convention |
|--------------|-----------------------------------------|------------|
| `Tbc`        | ORB-SLAM3 standard: camera → body (IMU) | T_body_cam |
| `T_imu_c`    | camera → IMU (explicit)                 | T_imu_cam  |
| `T_body_imu` | IMU → body FLU (optional)               | T_body_imu |
| `T_dvl_c`    | camera → DVL                            | T_dvl_cam  |

> `Tbc` and `T_imu_c` describe the same geometric relationship when `T_body_imu` is identity.  
> AQUA-SLAM uses `T_imu_c` directly in the SLAM pipeline; `T_body_imu` is applied only for body-frame output topics.
