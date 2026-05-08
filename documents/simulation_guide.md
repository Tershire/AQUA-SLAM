# Simulation Guide (Stonefish)

AQUA-SLAM simulation uses two Docker containers that communicate over ROS2 DDS:

| Container | Image | Role |
|-----------|-------|------|
| `aqua_slam_testbed` | `aqua/stonefish-sim:jazzy-dev` | Stonefish simulator + DVL converter |
| `aqua_slam_ros2_sim_dev` | `tershire/aqua-slam:ros2-jazzy-sim-dev` | AQUA-SLAM node + RViz2 |

Both containers use `network_mode: host` and `ROS_DOMAIN_ID=0`.

Companion repository: [aqua-slam-testbed](https://github.com/Tershire/aqua-slam-testbed)

---

## Prerequisites

- GPU and Docker set up: see [installation_guide.md](installation_guide.md)
- Both Docker images built (see each repo's docker directory)

Allow X11 from Docker once per session:

```bash
xhost +local:docker
```

---

## Scenario: simple_tank.scn

- **Tank**: 20 m (x/forward) × 8 m (y) × 4 m (z/depth), NED frame
- **Robot**: Girona500 simplified box body, starts at (2, 0, 2) facing forward
- **Stereo baseline**: 0.12 m
- **Cameras**: forward-facing, `rpy="1.5708 0.0 1.5708"` in body frame
- **Textures**: checker pattern on all walls and floor for ORB feature detection
- **Sonar**: omitted pending FLS vs MSIS decision

---

## Sensor topics

| Sensor | Topic | Type | Rate |
|--------|-------|------|------|
| Left camera | `/girona500/camera_left/image_color` | `sensor_msgs/Image` | 20 Hz |
| Right camera | `/girona500/camera_right/image_color` | `sensor_msgs/Image` | 20 Hz |
| IMU | `/girona500/imu/data` | `sensor_msgs/Imu` | 200 Hz |
| DVL (raw) | `/girona500/dvl` | `stonefish_ros2/DVL` | 5 Hz |
| DVL (SLAM) | `/bluerov2/dvl` | `nav_msgs/Odometry` | 5 Hz |
| Pressure | `/girona500/pressure` | `sensor_msgs/FluidPressure` | 10 Hz |

`/bluerov2/dvl` is published by `sim_dvl_converter` in the testbed container.

---

## Run

### Terminal 1 — Simulator (testbed container)

```bash
cd aqua-slam-testbed/docker
docker compose up -d
docker exec -it aqua_slam_testbed bash

ros2 launch /ros2_ws/launch/sim.launch.py
```

Optional launch arguments:

```bash
ros2 launch /ros2_ws/launch/sim.launch.py \
  simulation_rate:=1000.0 \
  window_width:=1280 \
  window_height:=720 \
  quality:=medium
```

### Terminal 2 — AQUA-SLAM (SLAM container)

```bash
cd aqua_slam_ws/src/AQUA-SLAM/docker/ros2_jazzy
docker compose up -d
docker exec -it aqua_slam_ros2_sim_dev bash

source ~/ros2_ws/install/setup.bash
ros2 launch aqua_slam stonefish_sim.launch.py
```

> `source ~/ros2_ws/install/setup.bash` is required every new shell session.

### Terminal 3 — Verify topic flow

```bash
docker exec -it aqua_slam_ros2_sim_dev bash

ros2 topic hz /girona500/camera_left/image_color --window 5
ros2 topic hz /girona500/imu/data              --window 5
ros2 topic hz /bluerov2/dvl                    --window 5
ros2 topic hz /aqua_slam/image/features        --window 5
```

`/aqua_slam/image/features` publishing at any rate confirms SLAM is tracking.
RViz shows `/aqua_slam/orb_path` (blue line) and `/aqua_slam/sparse_map` (point cloud) once initialized.

---

## Forward straight-line test

Drives the robot with a trapezoidal surge profile to trigger SLAM initialization:

```
thrust
  0.3 |      ___________
      |     /           \
    0 |____/             \____
         |ramp| cruise  |ramp|
```

```bash
# Terminal 4 — testbed container (after SLAM shows features in RViz)
docker exec -it aqua_slam_testbed bash
python3 /ros2_ws/scripts/forward_test.py --thrust 0.3 --cruise 20 --ramp 3
```

| Argument | Default | Notes |
|----------|---------|-------|
| `--thrust` | 0.3 | Normalized thrust 0–1 |
| `--cruise` | 20 s | Cruise duration; reduce if tank wall is reached |
| `--ramp` | 3 s | Ramp up/down duration |

The robot drifts slightly left due to single-propeller reaction torque — expected behavior.

---

## Recording a bag

```bash
# SLAM container
ros2 bag record /tf /bluerov2/dvl \
  /girona500/camera_left/image_color \
  /girona500/imu/data \
  -o ~/slam_test_$(date +%Y%m%d_%H%M%S)
```

---

## Known issues / TODO

- [ ] Replace Girona500 with BlueROV2 model
- [ ] Sonar sensor type/format for Stonefish 1.6 not yet resolved → sensor omitted
- [ ] `T_dvl_c` sign convention needs verification against stonefish_ros2 DVL output frame
- [ ] Camera actual rate ~5–20 Hz (GPU load dependent), not the specified 20 Hz
